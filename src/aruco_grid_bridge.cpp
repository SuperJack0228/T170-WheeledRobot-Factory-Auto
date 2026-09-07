#include "aruco_grid_bridge.h"

#include "function.h"
#include "seg_pose_bridge.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <pwd.h>
#include <sstream>

#include <Python.h>
#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace
{

#ifndef SEG_POSE_CONDA_PREFIX
#define SEG_POSE_CONDA_PREFIX "/home/ti5robot/anaconda3/envs/human_interaction_env"
#endif

std::string resolve_user_home()
{
    const char *home = std::getenv("HOME");
    const char *sudo_user = std::getenv("SUDO_USER");
    if (sudo_user != nullptr && sudo_user[0] != '\0' &&
        (home == nullptr || std::string(home) == "/root"))
    {
        if (const passwd *pw = getpwnam(sudo_user))
            return pw->pw_dir;
    }
    return (home != nullptr) ? home : "";
}

void prepend_path_env(const char *key, const std::string &prefix)
{
    if (prefix.empty())
        return;
    const char *existing = std::getenv(key);
    std::string merged = prefix;
    if (existing != nullptr && existing[0] != '\0')
    {
        merged += ':';
        merged += existing;
    }
    setenv(key, merged.c_str(), 1);
}

void setup_embedded_python_env()
{
    const std::string home = resolve_user_home();
    if (!home.empty())
        setenv("HOME", home.c_str(), 1);

    const std::string conda = SEG_POSE_CONDA_PREFIX;
    setenv("PYTHONHOME", conda.c_str(), 1);

    const std::string site = conda + "/lib/python3." + std::to_string(PY_MAJOR_VERSION) + "." +
                             std::to_string(PY_MINOR_VERSION) + "/site-packages";
    prepend_path_env("PYTHONPATH", site);
    prepend_path_env("PATH", conda + "/bin");
}

py::array bgr_vector_to_numpy(int h, int w, const std::vector<uint8_t> &bgr)
{
    const size_t expect = static_cast<size_t>(h) * static_cast<size_t>(w) * 3u;
    if (static_cast<size_t>(bgr.size()) != expect)
        throw std::runtime_error("color_bgr size mismatch");
    return py::array_t<uint8_t>(
        {h, w, 3},
        {static_cast<py::ssize_t>(w * 3), static_cast<py::ssize_t>(3), static_cast<py::ssize_t>(1)},
        bgr.data());
}

py::array K_to_numpy(const std::array<double, 9> &K)
{
    py::array_t<double> arr({3, 3});
    auto buf = arr.mutable_unchecked<2>();
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            buf(r, c) = K[static_cast<size_t>(r * 3 + c)];
    return arr;
}

py::array dist_to_numpy(const std::array<double, 5> &dist)
{
    py::array_t<double> arr(5);
    auto buf = arr.mutable_unchecked<1>();
    for (int i = 0; i < 5; ++i)
        buf(i) = dist[static_cast<size_t>(i)];
    return arr;
}

Eigen::Matrix<double, 4, 4> row_major_pose_to_matrix4(const std::array<double, 16> &pose)
{
    Eigen::Matrix<double, 4, 4> T;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            T(r, c) = pose[static_cast<size_t>(r * 4 + c)];
    return T;
}

std::vector<std::array<double, 16>> numpy_poses_to_vectors(const py::object &poses_obj)
{
    py::array_t<double> arr = poses_obj.cast<py::array_t<double>>();
    if (arr.ndim() != 3 || arr.shape(1) != 4 || arr.shape(2) != 4)
        throw std::runtime_error("poses_4x4 must be (N,4,4)");

    const py::ssize_t n = arr.shape(0);
    auto buf = arr.unchecked<3>();
    std::vector<std::array<double, 16>> out(static_cast<size_t>(n));
    for (py::ssize_t i = 0; i < n; ++i)
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                out[static_cast<size_t>(i)][static_cast<size_t>(r * 4 + c)] = buf(i, r, c);
    return out;
}

struct TrialPoses
{
    bool ok = false;
    ArucoGridDetectResult detect;
    std::vector<Eigen::Matrix<double, 1, 6>> robot6;
};

bool robot_poses_close(
    const std::vector<Eigen::Matrix<double, 1, 6>> &a,
    const std::vector<Eigen::Matrix<double, 1, 6>> &b,
    double pos_tol_m,
    double rot_tol_rad)
{
    if (a.size() != b.size() || a.empty())
        return false;
    for (size_t i = 0; i < a.size(); ++i)
    {
        for (int d = 0; d < 3; ++d)
            if (std::abs(a[i](d) - b[i](d)) > pos_tol_m)
                return false;
        for (int d = 3; d < 6; ++d)
            if (std::abs(a[i](d) - b[i](d)) > rot_tol_rad)
                return false;
    }
    return true;
}

std::vector<Eigen::Matrix<double, 1, 6>> cam_poses_to_robot6(
    const std::array<double, 16> &cam2robot,
    const std::vector<std::array<double, 16>> &poses_cam)
{
    std::vector<Eigen::Matrix<double, 1, 6>> out;
    out.reserve(poses_cam.size());
    for (const auto &pose_cam : poses_cam)
        out.push_back(aruco_pose_cam4x4_to_robot6(cam2robot, pose_cam));
    return out;
}

void show_aruco_place_last_trial_vis(
    const CameraFrameData &frame,
    int trial_index,
    bool detect_ok,
    const std::string &detect_message)
{
    if (!frame.ok || frame.color_bgr.empty() || frame.width <= 0 || frame.height <= 0)
        return;

    try
    {
        py::gil_scoped_acquire gil;
        py::module_ api = py::module_::import("aruco_grid_api");
        py::array_t<uint8_t> vis_arr = api.attr("draw_grid_detect_vis")(
                                              bgr_vector_to_numpy(frame.height, frame.width, frame.color_bgr),
                                              K_to_numpy(frame.K),
                                              dist_to_numpy(frame.dist),
                                              trial_index,
                                              detect_ok,
                                              detect_message)
                                          .cast<py::array_t<uint8_t>>();

        if (vis_arr.ndim() != 3 || vis_arr.shape(2) != 3)
            return;

        const int h = static_cast<int>(vis_arr.shape(0));
        const int w = static_cast<int>(vis_arr.shape(1));
        const auto *ptr = vis_arr.data();
        const size_t nbytes = static_cast<size_t>(h) * static_cast<size_t>(w) * 3u;
        pose_vis_set_panel_bgr(PoseVisPanel::HeadPlace, h, w, ptr, nbytes);
        pose_vis_save_debug_pair_bgr(
            PoseVisPanel::HeadPlace,
            frame.height,
            frame.width,
            frame.color_bgr.data(),
            frame.color_bgr.size(),
            h,
            w,
            ptr,
            nbytes);
        pose_vis_refresh_composite();
    }
    catch (const py::error_already_set &e)
    {
        std::cerr << "[aruco_grid] 放货调试图失败: " << e.what() << std::endl;
        PyErr_Clear();
    }
    catch (const std::exception &e)
    {
        std::cerr << "[aruco_grid] 放货调试图失败: " << e.what() << std::endl;
    }
}

} // namespace

struct ArucoGridBridge::Impl
{
    std::unique_ptr<py::scoped_interpreter> interpreter;
    py::object detect_fn;
    bool owns_interpreter = false;
};

ArucoGridBridge::ArucoGridBridge() : impl_(std::make_unique<Impl>()) {}

ArucoGridBridge::~ArucoGridBridge()
{
    shutdown();
}

void ArucoGridBridge::shutdown()
{
    if (!impl_)
        return;
    if (Py_IsInitialized())
    {
        py::gil_scoped_acquire gil;
        impl_->detect_fn = py::none();
        if (impl_->owns_interpreter)
            impl_->interpreter.reset();
    }
    impl_->owns_interpreter = false;
}

bool ArucoGridBridge::init(const std::string &seg_root, std::string &err)
{
    err.clear();
    try
    {
        setup_embedded_python_env();
        if (!Py_IsInitialized())
        {
            impl_->interpreter = std::make_unique<py::scoped_interpreter>();
            impl_->owns_interpreter = true;
        }
        else
        {
            impl_->owns_interpreter = false;
        }

        py::module_ sys = py::module_::import("sys");
        sys.attr("path").attr("insert")(0, seg_root);

        py::module_ api = py::module_::import("aruco_grid_api");
        impl_->detect_fn = api.attr("detect_grid_poses_4x4_cam");

        std::cout << "[aruco_grid] API 已加载, seg_root=" << seg_root
                  << (impl_->owns_interpreter ? " (自有解释器)" : " (共用解释器)") << std::endl;
        return true;
    }
    catch (const py::error_already_set &e)
    {
        err = std::string("ArucoGridBridge 初始化失败: ") + e.what();
        PyErr_Clear();
        impl_->detect_fn = py::none();
        if (impl_->owns_interpreter)
            impl_->interpreter.reset();
        impl_->owns_interpreter = false;
        return false;
    }
    catch (const std::exception &e)
    {
        err = std::string("ArucoGridBridge 初始化失败: ") + e.what();
        impl_->detect_fn = py::none();
        if (impl_->owns_interpreter)
            impl_->interpreter.reset();
        impl_->owns_interpreter = false;
        return false;
    }
}

ArucoGridDetectResult ArucoGridBridge::detect(const CameraFrameData &frame)
{
    py::gil_scoped_acquire gil;
    ArucoGridDetectResult result;
    if (!impl_->detect_fn || impl_->detect_fn.is_none())
    {
        result.message = "ArucoGridBridge 未初始化";
        return result;
    }
    if (!frame.ok || frame.color_bgr.empty() || frame.width <= 0 || frame.height <= 0)
    {
        result.message = "无效相机帧: " + frame.message;
        return result;
    }

    try
    {
        py::dict out = impl_->detect_fn(
                              bgr_vector_to_numpy(frame.height, frame.width, frame.color_bgr),
                              K_to_numpy(frame.K),
                              dist_to_numpy(frame.dist))
                          .cast<py::dict>();

        result.ok = out["ok"].cast<bool>();
        result.message = py::str(out["message"]);
        result.num_grid = out["num_grid"].cast<int>();
        result.num_corners = out["num_corners"].cast<int>();
        result.reproj_px = out["reproj_px"].cast<double>();
        result.marker_ids = out["marker_ids"].cast<std::vector<int>>();
        result.poses_cam_4x4 = numpy_poses_to_vectors(out["poses_4x4"]);
        return result;
    }
    catch (const py::error_already_set &e)
    {
        result.message = std::string("detect 异常: ") + e.what();
        PyErr_Clear();
        return result;
    }
    catch (const std::exception &e)
    {
        result.message = std::string("detect 异常: ") + e.what();
        return result;
    }
}

Eigen::Matrix<double, 1, 6> aruco_pose_cam4x4_to_robot6(
    const std::array<double, 16> &cam2robot,
    const std::array<double, 16> &pose_cam_4x4)
{
    const std::array<double, 16> pose_robot = transform_pose_cam_to_robot(cam2robot, pose_cam_4x4);
    return T2PosEulerAngles(row_major_pose_to_matrix4(pose_robot));
}

ArucoGridRobotPoses detect_aruco_grid_robot_poses_with_consensus(
    ArucoGridBridge &bridge,
    RealSenseMultiCam &cameras,
    const std::array<double, 16> &cam2robot,
    int num_trials,
    double pos_tol_m,
    double rot_tol_rad,
    bool show_place_visualization)
{
    ArucoGridRobotPoses out;
    out.num_trials = num_trials;

    if (num_trials <= 0)
    {
        out.message = "num_trials 必须 > 0";
        return out;
    }

    std::vector<TrialPoses> trials;
    trials.reserve(static_cast<size_t>(num_trials));

    CameraFrameData last_vis_frame;
    int last_vis_trial_index = 0;
    bool last_vis_detect_ok = false;
    std::string last_vis_detect_message;

    for (int t = 0; t < num_trials; ++t)
    {
        CameraFrameData frame = cameras.grab(CameraSlot::Head);
        if (!frame.ok)
        {
            std::cerr << "[aruco_grid] 第 " << (t + 1) << " 次取帧失败: " << frame.message << "\n";
            continue;
        }
        frame = RealSenseMultiCam::prepare_frame_for_slot(std::move(frame), CameraSlot::Head);

        TrialPoses trial;
        trial.detect = bridge.detect(frame);

        last_vis_frame = frame;
        last_vis_trial_index = t + 1;
        last_vis_detect_ok = trial.detect.ok && !trial.detect.poses_cam_4x4.empty();
        last_vis_detect_message = trial.detect.message;

        if (!trial.detect.ok || trial.detect.poses_cam_4x4.empty())
        {
            std::cerr << "[aruco_grid] 第 " << (t + 1) << " 次检测失败: " << trial.detect.message
                      << "\n";
            continue;
        }

        trial.robot6 = cam_poses_to_robot6(cam2robot, trial.detect.poses_cam_4x4);
        trial.ok = true;
        std::cout << "[aruco_grid] 第 " << (t + 1) << " 次 OK: markers="
                  << trial.detect.marker_ids.size()
                  << " corners=" << trial.detect.num_corners
                  << " reproj=" << trial.detect.reproj_px << "px\n";
        trials.push_back(std::move(trial));
    }

    if (show_place_visualization && last_vis_trial_index > 0)
    {
        show_aruco_place_last_trial_vis(
            last_vis_frame,
            last_vis_trial_index,
            last_vis_detect_ok,
            last_vis_detect_message);
    }

    if (trials.empty())
    {
        out.message = "7 次检测均无有效结果";
        return out;
    }

    int best_idx = 0;
    int best_cluster = 0;
    for (size_t i = 0; i < trials.size(); ++i)
    {
        int cluster = 0;
        for (size_t j = 0; j < trials.size(); ++j)
        {
            if (robot_poses_close(trials[i].robot6, trials[j].robot6, pos_tol_m, rot_tol_rad))
                ++cluster;
        }
        if (cluster > best_cluster)
        {
            best_cluster = cluster;
            best_idx = static_cast<int>(i);
        }
    }

    if (best_cluster < 2)
    {
        out.message = "有效次数=" + std::to_string(trials.size()) +
                      "，但各次结果差异过大，无共识簇";
        return out;
    }

    out.ok = true;
    out.cluster_size = best_cluster;
    out.picked_trial_index = best_idx;
    out.poses_robot = trials[static_cast<size_t>(best_idx)].robot6;
    std::ostringstream oss;
    oss << "共识 OK: " << trials.size() << " 次有效, 最大簇=" << best_cluster
        << ", 选用第 " << (best_idx + 1) << " 组";
    out.message = oss.str();
    return out;
}
