#include "seg_pose_bridge.h"
#include "head_cam2robot.h"
#include "move_box_config.h"
#include "function.h"
#include "Ti5_socketcan.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <pwd.h>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>

#include <Python.h>
#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace
{

std::atomic<bool> g_app_stop{false};

void on_sigint(int)
{
    _Exit(130);
}

} // namespace

void request_app_stop()
{
    g_app_stop.store(true, std::memory_order_relaxed);
}

bool app_stop_requested()
{
    return g_app_stop.load(std::memory_order_relaxed);
}

void install_app_sigint_handler()
{
    struct sigaction sa{};
    sa.sa_handler = on_sigint;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
}

void interruptible_sleep_ms(int ms)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (!app_stop_requested() && !hardware_abort_requested() &&
           std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

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

std::string conda_python_site_packages()
{
    const std::string pyver = std::string("python") + std::to_string(PY_MAJOR_VERSION) + "." +
                              std::to_string(PY_MINOR_VERSION);
    return std::string(SEG_POSE_CONDA_PREFIX) + "/lib/" + pyver + "/site-packages";
}

/** 进程内写死 human_interaction_env，不依赖 shell 里 conda activate。 */
void setup_embedded_python_env()
{
    const std::string home = resolve_user_home();
    if (!home.empty())
        setenv("HOME", home.c_str(), 1);

    const std::string conda = SEG_POSE_CONDA_PREFIX;
    const std::string site = conda_python_site_packages();

    setenv("PYTHONHOME", conda.c_str(), 1);
    setenv("PYTHONPATH", site.c_str(), 1);
    setenv("PYTHONNOUSERSITE", "1", 1);
    setenv("CONDA_PREFIX", conda.c_str(), 1);
    setenv("CONDA_DEFAULT_ENV", "human_interaction_env", 1);
    prepend_path_env("PATH", conda + "/bin");
    prepend_path_env("LD_LIBRARY_PATH", conda + "/lib");
}

void sanitize_embedded_sys_path(const std::string &seg_root)
{
    py::module_ sys = py::module_::import("sys");
    py::list cleaned;
    for (py::handle item : sys.attr("path"))
    {
        const std::string p = py::str(item);
        if (p.find("/.local/") != std::string::npos)
            continue;
        if (p.find("/opt/ros/") != std::string::npos)
            continue;
        cleaned.append(item);
    }
    sys.attr("path") = cleaned;
    const std::string site = conda_python_site_packages();
    sys.attr("path").attr("insert")(0, site);
    sys.attr("path").attr("insert")(0, seg_root);
}

constexpr const char *kPoseVisWindow = "pose_vis_all";
constexpr int kPanelDispW = 640;
constexpr int kPanelDispH = 480;

std::array<py::object, PoseVisPanel::Count> g_vis_panel_images{
    py::none(), py::none(), py::none()};
bool g_vis_composite_window_ready = false;
int g_vis_layout_mode = PoseVisLayout::Single;
int g_vis_single_panel_index = PoseVisPanel::Head;
int g_vis_window_w = 0;
int g_vis_window_h = 0;
bool g_pose_vis_save_debug = false;
bool g_pose_vis_gui_enabled = true;

const char *debug_subdir_for_panel(int panel_index)
{
    switch (panel_index)
    {
    case PoseVisPanel::Head:
        return "head";
    case PoseVisPanel::RightHand:
        return "right_hand";
    case PoseVisPanel::LeftHand:
        return "left_hand";
    default:
        return "unknown";
    }
}

struct DebugImagePaths
{
    std::string filename;
    std::string processed;
    std::string original;
};

DebugImagePaths make_debug_image_paths(int panel_index)
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t sec = std::chrono::system_clock::to_time_t(now);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) %
                    1000;
    std::tm tm_local{};
    localtime_r(&sec, &tm_local);

    std::ostringstream name;
    name << std::put_time(&tm_local, "%Y%m%d_%H%M%S")
         << '_' << std::setw(3) << std::setfill('0') << ms.count() << ".jpg";

    const std::string sub = debug_subdir_for_panel(panel_index);
    const std::string root = project_root_dir() + "/picture_debug/";
    return {name.str(), root + sub + "/" + name.str(), root + "original/" + sub + "/" + name.str()};
}

bool py_imwrite_bgr(const std::string &path, const py::object &bgr)
{
    if (bgr.is_none())
        return false;
    py::module_ cv2 = py::module_::import("cv2");
    if (!cv2.attr("imwrite")(path, bgr).cast<bool>())
    {
        std::cerr << "[debug_vis] 保存失败 " << path << std::endl;
        return false;
    }
    return true;
}

void save_debug_image_pair_impl(
    int panel_index,
    const py::object &original_bgr,
    const py::object &processed_bgr)
{
    if (!g_pose_vis_save_debug || panel_index < 0 || panel_index >= PoseVisPanel::Count)
        return;
    if (original_bgr.is_none() && processed_bgr.is_none())
        return;

    try
    {
        const DebugImagePaths paths = make_debug_image_paths(panel_index);
        const std::filesystem::path proc_dir = std::filesystem::path(paths.processed).parent_path();
        const std::filesystem::path orig_dir = std::filesystem::path(paths.original).parent_path();
        std::error_code ec;
        if (!processed_bgr.is_none())
            std::filesystem::create_directories(proc_dir, ec);
        if (!original_bgr.is_none())
            std::filesystem::create_directories(orig_dir, ec);

        bool saved = false;
        if (!processed_bgr.is_none() && py_imwrite_bgr(paths.processed, processed_bgr))
        {
            std::cout << "[debug_vis] 调试图 " << paths.processed << std::endl;
            saved = true;
        }
        if (!original_bgr.is_none() && py_imwrite_bgr(paths.original, original_bgr))
        {
            std::cout << "[debug_vis] 原图 " << paths.original << std::endl;
            saved = true;
        }
        (void)saved;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[debug_vis] 保存异常: " << e.what() << std::endl;
    }
}

size_t camera_slot_index(CameraSlot slot)
{
    return static_cast<size_t>(slot);
}

py::object make_vis_panel(
    const py::object &src,
    int disp_w,
    int disp_h,
    const char *label,
    py::module_ &cv2,
    py::module_ &np)
{
    py::object panel;
    if (!src.is_none())
        panel = cv2.attr("resize")(src, py::make_tuple(disp_w, disp_h));
    else
        panel = np.attr("zeros")(
            py::make_tuple(disp_h, disp_w, 3), py::arg("dtype") = np.attr("uint8"));
    cv2.attr("putText")(
        panel,
        label,
        py::make_tuple(10, 28),
        cv2.attr("FONT_HERSHEY_SIMPLEX"),
        0.8,
        py::make_tuple(0, 255, 255),
        2,
        cv2.attr("LINE_AA"));
    return panel;
}

void ensure_vis_window_size(int win_w, int win_h, py::module_ &cv2)
{
    if (!g_vis_composite_window_ready)
    {
        cv2.attr("namedWindow")(kPoseVisWindow, cv2.attr("WINDOW_NORMAL"));
        g_vis_composite_window_ready = true;
    }
    if (!g_vis_composite_window_ready || win_w != g_vis_window_w || win_h != g_vis_window_h)
    {
        cv2.attr("resizeWindow")(kPoseVisWindow, win_w, win_h);
        g_vis_window_w = win_w;
        g_vis_window_h = win_h;
    }
}

const char *label_for_panel(size_t panel_idx)
{
    switch (panel_idx)
    {
    case PoseVisPanel::Head:
        return "Head";
    case PoseVisPanel::RightHand:
        return "RightHand";
    case PoseVisPanel::LeftHand:
        return "LeftHand";
    default:
        return "?";
    }
}

void show_pose_visualization_composite()
{
    if (!g_pose_vis_gui_enabled)
        return;
    py::module_ cv2 = py::module_::import("cv2");
    py::module_ np = py::module_::import("numpy");

    py::object composite;
    if (g_vis_layout_mode == PoseVisLayout::DualHand)
    {
        ensure_vis_window_size(kPanelDispW * 2, kPanelDispH, cv2);
        py::list panels;
        panels.append(make_vis_panel(
            g_vis_panel_images[PoseVisPanel::LeftHand],
            kPanelDispW,
            kPanelDispH,
            label_for_panel(PoseVisPanel::LeftHand),
            cv2,
            np));
        panels.append(make_vis_panel(
            g_vis_panel_images[PoseVisPanel::RightHand],
            kPanelDispW,
            kPanelDispH,
            label_for_panel(PoseVisPanel::RightHand),
            cv2,
            np));
        composite = np.attr("hstack")(panels);
    }
    else
    {
        const size_t idx = (g_vis_single_panel_index >= 0 &&
                            g_vis_single_panel_index < PoseVisPanel::Count)
                               ? static_cast<size_t>(g_vis_single_panel_index)
                               : static_cast<size_t>(PoseVisPanel::Head);
        ensure_vis_window_size(kPanelDispW, kPanelDispH, cv2);
        composite = make_vis_panel(
            g_vis_panel_images[idx],
            kPanelDispW,
            kPanelDispH,
            label_for_panel(idx),
            cv2,
            np);
    }

    cv2.attr("imshow")(kPoseVisWindow, composite);
    cv2.attr("waitKey")(1);
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

py::array depth_vector_to_numpy(int h, int w, const std::vector<float> &depth_m)
{
    const size_t expect = static_cast<size_t>(h) * static_cast<size_t>(w);
    if (static_cast<size_t>(depth_m.size()) != expect)
        throw std::runtime_error("depth_m size mismatch");
    return py::array_t<float>(
        {h, w},
        {static_cast<py::ssize_t>(w * sizeof(float)), static_cast<py::ssize_t>(sizeof(float))},
        depth_m.data());
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

py::array pose16_to_numpy(const std::array<double, 16> &pose)
{
    py::array_t<double> arr({4, 4});
    auto buf = arr.mutable_unchecked<2>();
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            buf(r, c) = pose[static_cast<size_t>(r * 4 + c)];
    return arr;
}

std::string cam2robot_yaml_for_slot(CameraSlot slot)
{
    switch (slot)
    {
    case CameraSlot::RightHand:
        return default_right_hand_to_robot_yaml_path();
    case CameraSlot::LeftHand:
        return default_left_hand_to_robot_yaml_path();
    default:
        return default_camera_to_robot_yaml_path();
    }
}

std::array<double, 16> numpy_pose_to_array(const py::object &pose_obj)
{
    py::array_t<double> arr = pose_obj.cast<py::array_t<double>>();
    if (arr.ndim() != 2 || arr.shape(0) != 4 || arr.shape(1) != 4)
        throw std::runtime_error("pose_4x4 must be 4x4");

    std::array<double, 16> out{};
    auto buf = arr.unchecked<2>();
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            out[static_cast<size_t>(r * 4 + c)] = buf(r, c);
    return out;
}

void pose_vis_set_panel_bgr_impl(int panel_index, int height, int width, const uint8_t *bgr, size_t bgr_bytes)
{
    if (panel_index < 0 || panel_index >= PoseVisPanel::Count || bgr == nullptr)
        return;
    const size_t expect = static_cast<size_t>(height) * static_cast<size_t>(width) * 3u;
    if (height <= 0 || width <= 0 || bgr_bytes != expect)
        return;

    std::vector<uint8_t> copy(bgr, bgr + bgr_bytes);
    py::object vis = bgr_vector_to_numpy(height, width, copy);
    g_vis_panel_images[static_cast<size_t>(panel_index)] = vis;
}

void pose_vis_refresh_composite_impl()
{
    show_pose_visualization_composite();
}

void pose_vis_set_layout_impl(int layout, int single_panel_index)
{
    g_vis_layout_mode = (layout == PoseVisLayout::DualHand) ? PoseVisLayout::DualHand
                                                           : PoseVisLayout::Single;
    if (single_panel_index >= 0 && single_panel_index < PoseVisPanel::Count)
        g_vis_single_panel_index = single_panel_index;
}

void pose_vis_clear_panels_impl(const std::initializer_list<int> &panels)
{
    for (int p : panels)
    {
        if (p >= 0 && p < PoseVisPanel::Count)
            g_vis_panel_images[static_cast<size_t>(p)] = py::none();
    }
}

void show_pose_visualization(
    const CameraFrameData &frame,
    const py::object &algorithm_output,
    int panel_index)
{
    py::module_ vis_mod = py::module_::import("visualization");

    py::array rgb = bgr_vector_to_numpy(frame.height, frame.width, frame.color_bgr);
    py::object vis = vis_mod.attr("draw_results")(
        rgb,
        algorithm_output,
        K_to_numpy(frame.K),
        dist_to_numpy(frame.dist),
        false);

    if (panel_index >= 0 && panel_index < PoseVisPanel::Count)
    {
        g_vis_panel_images[static_cast<size_t>(panel_index)] = vis;
        save_debug_image_pair_impl(panel_index, rgb, vis);
        if (panel_index == PoseVisPanel::RightHand || panel_index == PoseVisPanel::LeftHand)
            pose_vis_set_layout_impl(PoseVisLayout::DualHand, PoseVisPanel::Head);
        else
            pose_vis_set_layout_impl(PoseVisLayout::Single, panel_index);
    }
    show_pose_visualization_composite();
}

void show_aruco_visualization(
    const CameraFrameData &frame,
    const py::object &vis,
    int panel_index)
{
    py::array rgb = bgr_vector_to_numpy(frame.height, frame.width, frame.color_bgr);
    if (panel_index >= 0 && panel_index < PoseVisPanel::Count)
    {
        g_vis_panel_images[static_cast<size_t>(panel_index)] = vis;
        save_debug_image_pair_impl(panel_index, rgb, vis);
        if (panel_index == PoseVisPanel::RightHand || panel_index == PoseVisPanel::LeftHand)
            pose_vis_set_layout_impl(PoseVisLayout::DualHand, PoseVisPanel::Head);
        else
            pose_vis_set_layout_impl(PoseVisLayout::Single, panel_index);
    }
    show_pose_visualization_composite();
}

} // namespace

void pose_vis_set_save_debug(bool enable)
{
    g_pose_vis_save_debug = enable;
}

void pose_vis_set_gui_enabled(bool enable)
{
    g_pose_vis_gui_enabled = enable;
}

void pose_vis_save_debug_pair_bgr(
    int panel_index,
    int orig_h,
    int orig_w,
    const uint8_t *orig_bgr,
    size_t orig_bytes,
    int proc_h,
    int proc_w,
    const uint8_t *proc_bgr,
    size_t proc_bytes)
{
    py::gil_scoped_acquire gil;
    py::object orig = py::none();
    py::object proc = py::none();
    const size_t orig_expect =
        static_cast<size_t>(orig_h) * static_cast<size_t>(orig_w) * 3u;
    const size_t proc_expect =
        static_cast<size_t>(proc_h) * static_cast<size_t>(proc_w) * 3u;
    if (orig_bgr != nullptr && orig_h > 0 && orig_w > 0 && orig_bytes == orig_expect)
    {
        std::vector<uint8_t> copy(orig_bgr, orig_bgr + orig_bytes);
        orig = bgr_vector_to_numpy(orig_h, orig_w, copy);
    }
    if (proc_bgr != nullptr && proc_h > 0 && proc_w > 0 && proc_bytes == proc_expect)
    {
        std::vector<uint8_t> copy(proc_bgr, proc_bgr + proc_bytes);
        proc = bgr_vector_to_numpy(proc_h, proc_w, copy);
    }
    save_debug_image_pair_impl(panel_index, orig, proc);
}

void pose_vis_set_panel_bgr(int panel_index, int height, int width, const uint8_t *bgr, size_t bgr_bytes)
{
    py::gil_scoped_acquire gil;
    pose_vis_set_panel_bgr_impl(panel_index, height, width, bgr, bgr_bytes);
}

void pose_vis_set_layout(int layout, int single_panel_index)
{
    py::gil_scoped_acquire gil;
    pose_vis_set_layout_impl(layout, single_panel_index);
    pose_vis_refresh_composite_impl();
}

void pose_vis_clear_panels(const std::initializer_list<int> panels)
{
    py::gil_scoped_acquire gil;
    pose_vis_clear_panels_impl(panels);
    pose_vis_refresh_composite_impl();
}

void pose_vis_begin_phase(
    int layout,
    int single_panel_index,
    const std::initializer_list<int> clear_panels)
{
    py::gil_scoped_acquire gil;
    pose_vis_clear_panels_impl(clear_panels);
    pose_vis_set_layout_impl(layout, single_panel_index);
    pose_vis_refresh_composite_impl();
}

void pose_vis_refresh_composite()
{
    py::gil_scoped_acquire gil;
    pose_vis_refresh_composite_impl();
}

struct SegPoseBridge::Impl
{
    std::unique_ptr<py::scoped_interpreter> interpreter;
    py::object engine;
    py::object algorithm_input_cls;
    py::object engine_cls;
    py::object load_pose_params;
    py::object aruco_detect_fn;
    bool aruco_ready = false;
    py::object tray_detect_fn;
    py::object tray_fuse_fn;
    bool tray_ready = false;
    py::object belt_detect_fn;
    bool belt_ready = false;
};

SegPoseBridge::SegPoseBridge() : impl_(std::make_unique<Impl>()) {}

SegPoseBridge::~SegPoseBridge() = default;

bool SegPoseBridge::init(const std::string &pose_config_path, const std::string &seg_root, std::string &err)
{
    err.clear();
    try
    {
        setup_embedded_python_env();
        impl_->interpreter = std::make_unique<py::scoped_interpreter>();
        sanitize_embedded_sys_path(seg_root);

        py::module_ np = py::module_::import("numpy");
        std::cout << "[python] 锁定 " << SEG_POSE_CONDA_PREFIX
                  << "（无需 conda activate），numpy="
                  << std::string(py::str(np.attr("__version__"))) << " @ "
                  << std::string(py::str(np.attr("__file__"))) << std::endl;

        py::module_ config_mod = py::module_::import("algorithm.config_loader");
        py::module_ engine_mod = py::module_::import("algorithm.engine");
        py::module_ types_mod = py::module_::import("algorithm.types");

        impl_->load_pose_params = config_mod.attr("load_pose_params");
        impl_->engine_cls = engine_mod.attr("CirclePoseEngine");
        py::object params = impl_->load_pose_params(pose_config_path);
        impl_->engine = impl_->engine_cls(params);
        impl_->aruco_detect_fn = py::none();
        impl_->aruco_ready = false;
        impl_->tray_detect_fn = py::none();
        impl_->tray_fuse_fn = py::none();
        impl_->tray_ready = false;
        impl_->belt_detect_fn = py::none();
        impl_->belt_ready = false;
        impl_->algorithm_input_cls = types_mod.attr("AlgorithmInput");

        std::cout << "CirclePoseEngine 已加载 (feeding_cylindrical_parts_alg), 配置: "
                  << pose_config_path << std::endl;
        return true;
    }
    catch (const std::exception &e)
    {
        err = std::string("SegPoseBridge 初始化失败: ") + e.what();
        impl_->engine = py::none();
        impl_->aruco_detect_fn = py::none();
        impl_->aruco_ready = false;
        impl_->tray_detect_fn = py::none();
        impl_->tray_fuse_fn = py::none();
        impl_->tray_ready = false;
        return false;
    }
}

bool SegPoseBridge::init_aruco_engine(const std::string &config_path, std::string &err)
{
    err.clear();
    if (!impl_->interpreter)
    {
        err = "先初始化抓取引擎后再加载 ArUco";
        return false;
    }
    try
    {
        py::gil_scoped_acquire gil;
        py::module_ sys = py::module_::import("sys");
        sys.attr("path").attr("insert")(0, default_online_pose_root());
        py::module_ api = py::module_::import("aruco_detect_api");
        const std::string info = py::str(api.attr("init")(config_path));
        impl_->aruco_detect_fn = api.attr("detect_frame");
        impl_->aruco_ready = true;
        std::cout << "ArUco 单码引擎已加载, 配置: " << config_path << " (" << info << ")"
                  << std::endl;
        return true;
    }
    catch (const std::exception &e)
    {
        err = std::string("ArUco 引擎初始化失败: ") + e.what();
        impl_->aruco_detect_fn = py::none();
        impl_->aruco_ready = false;
        return false;
    }
}

bool SegPoseBridge::aruco_engine_ready() const
{
    return impl_ && impl_->aruco_ready;
}

ArucoDetectResult SegPoseBridge::run_aruco(
    const CameraFrameData &frame,
    bool show_visualization,
    CameraSlot vis_slot)
{
    py::gil_scoped_acquire gil;
    ArucoDetectResult result;
    if (!impl_->aruco_ready || !impl_->aruco_detect_fn || impl_->aruco_detect_fn.is_none())
    {
        result.message = "ArUco 引擎未初始化";
        return result;
    }
    if (!frame.ok || frame.color_bgr.empty() || frame.width <= 0 || frame.height <= 0)
    {
        result.message = "无效相机帧: " + frame.message;
        return result;
    }

    try
    {
        py::object cam2robot_arg = py::none();
        std::array<double, 16> cam2robot{};
        std::string ext_err;
        const bool got_ext = (vis_slot == CameraSlot::Head)
                                 ? load_head_cam2robot(cam2robot, ext_err)
                                 : load_cam2robot_matrix(cam2robot_yaml_for_slot(vis_slot), cam2robot, ext_err);
        if (got_ext)
            cam2robot_arg = pose16_to_numpy(cam2robot);

        py::object out = impl_->aruco_detect_fn(
            bgr_vector_to_numpy(frame.height, frame.width, frame.color_bgr),
            K_to_numpy(frame.K),
            dist_to_numpy(frame.dist),
            cam2robot_arg);
        py::list markers = out.attr("get")("markers");
        result.markers.reserve(static_cast<size_t>(markers.size()));
        for (py::handle h : markers)
        {
            py::object m = py::reinterpret_borrow<py::object>(h);
            ArucoMarkerResult one;
            one.marker_id = m.attr("get")("id").cast<int>();
            one.side_m = m.attr("get")("side_m").cast<double>();
            one.reproj_px = m.attr("get")("reproj_px").cast<double>();
            const auto t = m.attr("get")("t_m").cast<std::vector<double>>();
            const auto rpy = m.attr("get")("rpy_deg").cast<std::vector<double>>();
            for (int i = 0; i < 3; ++i)
            {
                one.t_m[i] = (i < static_cast<int>(t.size())) ? t[static_cast<size_t>(i)] : 0.0;
                one.rpy_deg[i] =
                    (i < static_cast<int>(rpy.size())) ? rpy[static_cast<size_t>(i)] : 0.0;
            }
            one.pose_4x4 = numpy_pose_to_array(m.attr("get")("pose_4x4"));
            result.markers.push_back(std::move(one));
        }
        result.ok = true;
        result.message = "ok";

        if (show_visualization)
        {
            const int panel = static_cast<int>(camera_slot_index(vis_slot));
            show_aruco_visualization(frame, out.attr("get")("vis"), panel);
        }
    }
    catch (const py::error_already_set &e)
    {
        if (PyErr_ExceptionMatches(PyExc_KeyboardInterrupt))
        {
            PyErr_Clear();
            request_app_stop();
            result.message = "interrupted";
            return result;
        }
        result.message = std::string("aruco 异常: ") + e.what();
    }
    catch (const std::exception &e)
    {
        result.message = std::string("aruco 异常: ") + e.what();
    }
    return result;
}

bool SegPoseBridge::init_tray_engine(const std::string &config_path, std::string &err)
{
    err.clear();
    if (!impl_->interpreter)
    {
        err = "先初始化抓取引擎后再加载料盘 ArUco";
        return false;
    }
    try
    {
        py::gil_scoped_acquire gil;
        py::module_ sys = py::module_::import("sys");
        sys.attr("path").attr("insert")(0, default_board_yf100_root());
        py::module_ api = py::module_::import("tray_detect_api");
        const std::string info = py::str(api.attr("init")(config_path));
        impl_->tray_detect_fn = api.attr("detect_and_annotate");
        impl_->tray_fuse_fn = api.attr("fuse_and_annotate");
        impl_->tray_ready = true;
        std::cout << "料盘 ArUco 引擎已加载, 配置: " << config_path << " (" << info << ")"
                  << std::endl;
        return true;
    }
    catch (const std::exception &e)
    {
        err = std::string("料盘引擎初始化失败: ") + e.what();
        impl_->tray_detect_fn = py::none();
        impl_->tray_fuse_fn = py::none();
        impl_->tray_ready = false;
        return false;
    }
}

bool SegPoseBridge::tray_engine_ready() const
{
    return impl_ && impl_->tray_ready;
}

bool SegPoseBridge::init_belt_engine(const std::string &config_path, std::string &err)
{
    err.clear();
    if (!impl_->interpreter)
    {
        err = "先初始化抓取引擎后再加载传送带 ArUco";
        return false;
    }
    try
    {
        py::gil_scoped_acquire gil;
        py::module_ sys = py::module_::import("sys");
        py::module_ util = py::module_::import("importlib.util");
        const std::string api_py = default_board_belt_root() + "/belt_detect_api.py";
        py::object spec = util.attr("spec_from_file_location")("t170c_belt_detect_api", api_py);
        if (spec.is_none())
        {
            err = "找不到 " + api_py;
            impl_->belt_detect_fn = py::none();
            impl_->belt_ready = false;
            return false;
        }
        py::object mod = util.attr("module_from_spec")(spec);
        sys.attr("modules")["t170c_belt_detect_api"] = mod;
        spec.attr("loader").attr("exec_module")(mod);
        const std::string info = py::str(mod.attr("init")(config_path));
        impl_->belt_detect_fn = mod.attr("detect_frame");
        impl_->belt_ready = true;
        std::cout << "传送带 ArUco 引擎已加载, 配置: " << config_path << " (" << info << ")"
                  << std::endl;
        return true;
    }
    catch (const std::exception &e)
    {
        err = std::string("传送带引擎初始化失败: ") + e.what();
        impl_->belt_detect_fn = py::none();
        impl_->belt_ready = false;
        return false;
    }
}

bool SegPoseBridge::belt_engine_ready() const
{
    return impl_ && impl_->belt_ready;
}

BeltDetectResult SegPoseBridge::run_belt_detect(
    const CameraFrameData &frame,
    const std::string &prefer_name,
    bool show_visualization,
    CameraSlot vis_slot,
    const double *grasp_offset_left,
    const double *grasp_offset_right)
{
    py::gil_scoped_acquire gil;
    BeltDetectResult result;
    if (!impl_->belt_ready || !impl_->belt_detect_fn || impl_->belt_detect_fn.is_none())
    {
        result.message = "传送带引擎未初始化";
        return result;
    }
    if (!frame.ok || frame.color_bgr.empty() || frame.width <= 0 || frame.height <= 0)
    {
        result.message = "无效相机帧: " + frame.message;
        return result;
    }

    try
    {
        py::object cam2robot_arg = py::none();
        std::array<double, 16> cam2robot{};
        std::string ext_err;
        const bool got_ext = (vis_slot == CameraSlot::Head)
                                 ? load_head_cam2robot(cam2robot, ext_err)
                                 : load_cam2robot_matrix(cam2robot_yaml_for_slot(vis_slot), cam2robot, ext_err);
        if (got_ext)
            cam2robot_arg = pose16_to_numpy(cam2robot);

        py::object belt_api = py::module_::import("sys").attr("modules")["t170c_belt_detect_api"];
        const std::string save_path =
            py::str(belt_api.attr("default_save_path")(project_root_dir()));

        auto make_off = [](const double *src) -> py::object {
            if (src == nullptr)
                return py::none();
            py::list off;
            off.append(src[0]);
            off.append(src[1]);
            off.append(src[2]);
            return off;
        };
        py::object out = impl_->belt_detect_fn(
            bgr_vector_to_numpy(frame.height, frame.width, frame.color_bgr),
            K_to_numpy(frame.K),
            dist_to_numpy(frame.dist),
            cam2robot_arg,
            save_path,
            prefer_name,
            py::none(),
            make_off(grasp_offset_left),
            make_off(grasp_offset_right));

        result.ok = out.attr("get")("ok").cast<bool>();
        result.message = py::str(out.attr("get")("message"));
        result.name = py::str(out.attr("get")("name"));
        result.reproj_px = out.attr("get")("reproj_px").cast<double>();
        result.save_path = py::str(out.attr("get")("save_path"));
        py::object ids_obj = out.attr("get")("used_ids");
        if (!ids_obj.is_none())
        {
            const auto ids = ids_obj.cast<std::vector<int>>();
            result.used_ids = ids;
        }

        auto read_xyz = [](const py::object &obj, double &x, double &y, double &z) -> bool {
            if (obj.is_none())
                return false;
            const auto v = obj.cast<std::vector<double>>();
            if (v.size() < 3)
                return false;
            x = v[0];
            y = v[1];
            z = v[2];
            return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
        };
        read_xyz(out.attr("get")("origin_cam"), result.cam_x, result.cam_y, result.cam_z);
        result.in_robot = read_xyz(
            out.attr("get")("origin_robot"), result.origin_x, result.origin_y, result.origin_z);
        result.have_grasp_l = read_xyz(
            out.attr("get")("grasp_left_robot"), result.grasp_lx, result.grasp_ly, result.grasp_lz);
        result.have_grasp_r = read_xyz(
            out.attr("get")("grasp_right_robot"), result.grasp_rx, result.grasp_ry, result.grasp_rz);
        py::object R_obj = out.attr("get")("R_robot");
        if (!R_obj.is_none())
        {
            const auto R = R_obj.cast<std::vector<double>>();
            if (R.size() >= 9)
            {
                result.ax_x = R[0];
                result.ax_y = R[3];
                result.ax_z = R[6];
                result.ay_x = R[1];
                result.ay_y = R[4];
                result.ay_z = R[7];
                result.az_x = R[2];
                result.az_y = R[5];
                result.az_z = R[8];
                result.have_axes = std::isfinite(result.ay_x) && std::isfinite(result.ay_y) &&
                                   std::isfinite(result.ay_z);
            }
        }

        if (show_visualization)
        {
            const int panel = static_cast<int>(camera_slot_index(vis_slot));
            show_aruco_visualization(frame, out.attr("get")("vis_bgr"), panel);
        }
    }
    catch (const py::error_already_set &e)
    {
        if (PyErr_ExceptionMatches(PyExc_KeyboardInterrupt))
        {
            PyErr_Clear();
            request_app_stop();
            result.message = "interrupted";
            return result;
        }
        result.message = std::string("belt aruco 异常: ") + e.what();
    }
    catch (const std::exception &e)
    {
        result.message = std::string("belt aruco 异常: ") + e.what();
    }
    return result;
}

TrayDetectResult SegPoseBridge::run_tray_annotate(
    const CameraFrameData &frame,
    const PoseDetectionRecords &yolo,
    bool show_visualization,
    CameraSlot vis_slot)
{
    return run_tray_annotate_multiframe({frame}, yolo, show_visualization, vis_slot);
}

TrayDetectResult SegPoseBridge::run_tray_annotate_multiframe(
    const std::vector<CameraFrameData> &frames,
    const PoseDetectionRecords &yolo,
    bool show_visualization,
    CameraSlot vis_slot,
    bool tray2_markers)
{
    py::gil_scoped_acquire gil;
    TrayDetectResult result;
    if (!impl_->tray_ready || !impl_->tray_detect_fn || impl_->tray_detect_fn.is_none())
    {
        result.message = "料盘引擎未初始化";
        return result;
    }
    if (frames.empty())
    {
        result.message = "没有料盘识别帧";
        return result;
    }
    const CameraFrameData &frame = frames.back();
    if (!frame.ok || frame.color_bgr.empty() || frame.width <= 0 || frame.height <= 0)
    {
        result.message = "无效相机帧: " + frame.message;
        return result;
    }

    struct TrayBoardReset
    {
        bool tray2 = false;
        ~TrayBoardReset()
        {
            if (!tray2)
                return;
            try
            {
                py::module_::import("tray_detect_api").attr("set_active_board")("tray");
            }
            catch (...)
            {
            }
        }
    } board_reset;
    if (tray2_markers)
    {
        py::module_::import("tray_detect_api").attr("set_active_board")("tray2");
        board_reset.tray2 = true;
    }

    try
    {
        py::object cam2robot_arg = py::none();
        std::array<double, 16> cam2robot{};
        std::string ext_err;
        const bool got_ext = (vis_slot == CameraSlot::Head)
                                 ? load_head_cam2robot(cam2robot, ext_err)
                                 : load_cam2robot_matrix(cam2robot_yaml_for_slot(vis_slot), cam2robot, ext_err);
        if (got_ext)
            cam2robot_arg = pose16_to_numpy(cam2robot);

        py::object depth_arg = py::none();
        if (!frame.depth_m.empty())
            depth_arg = depth_vector_to_numpy(frame.height, frame.width, frame.depth_m);

        py::list dets;
        for (const PoseDetectionRecord &rec : yolo)
        {
            const PoseTargetResult &t = rec.target;
            py::dict d;
            d["class_id"] = t.class_id;
            d["class_name"] = t.class_name;
            d["conf"] = t.confidence;
            d["xyz_cam"] = py::make_tuple(t.pose_4x4[3], t.pose_4x4[7], t.pose_4x4[11]);
            dets.append(d);
        }

        const std::string save_path =
            py::str(py::module_::import("tray_detect_api").attr("default_save_path")(project_root_dir()));

        py::object out;
        const bool use_fuse = frames.size() > 1 && impl_->tray_fuse_fn && !impl_->tray_fuse_fn.is_none();
        if (use_fuse)
        {
            py::list bgrs;
            for (const CameraFrameData &f : frames)
            {
                if (!f.ok || f.color_bgr.empty())
                    continue;
                bgrs.append(bgr_vector_to_numpy(f.height, f.width, f.color_bgr));
            }
            out = impl_->tray_fuse_fn(
                bgrs,
                depth_arg,
                K_to_numpy(frame.K),
                dist_to_numpy(frame.dist),
                cam2robot_arg,
                dets,
                save_path);
        }
        else
        {
            out = impl_->tray_detect_fn(
                bgr_vector_to_numpy(frame.height, frame.width, frame.color_bgr),
                depth_arg,
                K_to_numpy(frame.K),
                dist_to_numpy(frame.dist),
                cam2robot_arg,
                dets,
                save_path);
        }

        result.ok = out.attr("get")("ok").cast<bool>();
        result.message = py::str(out.attr("get")("message"));
        result.reproj_px = out.attr("get")("reproj_px").cast<double>();
        result.save_path = py::str(out.attr("get")("save_path"));
        py::object tilt_obj = out.attr("get")("tilt_deg");
        if (!tilt_obj.is_none())
            result.tilt_deg = tilt_obj.cast<double>();
        py::object origin_obj = out.attr("get")("origin_xyz");
        if (!origin_obj.is_none())
        {
            try
            {
                py::sequence origin = origin_obj.cast<py::sequence>();
                if (origin.size() >= 3)
                {
                    result.origin_x = origin[0].cast<double>();
                    result.origin_y = origin[1].cast<double>();
                    result.origin_z = origin[2].cast<double>();
                }
            }
            catch (...)
            {
            }
        }
        py::object ids_obj = out.attr("get")("used_ids");
        if (!ids_obj.is_none())
        {
            for (py::handle h : ids_obj.cast<py::list>())
                result.used_ids.push_back(h.cast<int>());
        }
        py::object holes_obj = out.attr("get")("holes");
        if (!holes_obj.is_none())
        {
            py::list holes = holes_obj.cast<py::list>();
            result.holes.reserve(static_cast<size_t>(holes.size()));
            for (py::handle h : holes)
            {
                py::object m = py::reinterpret_borrow<py::object>(h);
                TrayHoleResult one;
                one.id = m.attr("get")("id").cast<int>();
                one.row = m.attr("get")("row").cast<int>();
                one.col = m.attr("get")("col").cast<int>();
                auto get_d = [](py::object obj, const char *key) -> double {
                    py::object v = obj.attr("get")(key, py::none());
                    if (v.is_none())
                        return std::numeric_limits<double>::quiet_NaN();
                    try
                    {
                        return v.cast<double>();
                    }
                    catch (...)
                    {
                        return std::numeric_limits<double>::quiet_NaN();
                    }
                };
                one.x = get_d(m, "x");
                one.y = get_d(m, "y");
                one.x_level = get_d(m, "x_level");
                one.y_level = get_d(m, "y_level");
                one.tray_z = get_d(m, "tray_z");
                one.tray_z_level = get_d(m, "tray_z_level");
                one.top_z = get_d(m, "top_z");
                one.top_z_level = get_d(m, "top_z_level");
                one.height_on_tray = get_d(m, "height_on_tray");
                one.class_id = m.attr("get")("class_id").cast<int>();
                one.class_name = py::str(m.attr("get")("class_name"));
                one.conf = m.attr("get")("conf").cast<double>();
                one.dxy = m.attr("get")("dxy").cast<double>();
                one.depth_pts = m.attr("get")("depth_pts").cast<int>();
                one.in_robot = m.attr("get")("in_robot").cast<bool>();
                result.holes.push_back(std::move(one));
            }
        }

        if (show_visualization)
        {
            const int panel = static_cast<int>(camera_slot_index(vis_slot));
            show_aruco_visualization(frame, out.attr("get")("vis_bgr"), panel);
        }
    }
    catch (const py::error_already_set &e)
    {
        if (PyErr_ExceptionMatches(PyExc_KeyboardInterrupt))
        {
            PyErr_Clear();
            request_app_stop();
            result.message = "interrupted";
            return result;
        }
        result.message = std::string("tray 异常: ") + e.what();
    }
    catch (const std::exception &e)
    {
        result.message = std::string("tray 异常: ") + e.what();
    }
    return result;
}

PoseRunResult SegPoseBridge::run(
    const CameraFrameData &frame,
    int algorithm_id,
    bool show_visualization,
    CameraSlot vis_slot,
    int vis_panel,
    SegEngineId engine_id)
{
    py::gil_scoped_acquire gil;
    PoseRunResult result;
    (void)engine_id;
    py::object engine = impl_->engine;
    const char *missing = "引擎未初始化";
    if (!engine || engine.is_none())
    {
        result.message = missing;
        return result;
    }
    if (!frame.ok || frame.color_bgr.empty() || frame.width <= 0 || frame.height <= 0)
    {
        result.message = "无效相机帧: " + frame.message;
        return result;
    }

    try
    {
        const bool need_depth = (algorithm_id == 1);
        py::object depth_arg;
        if (need_depth)
        {
            if (frame.depth_m.empty())
            {
                result.message = "算法 1 需要深度图";
                return result;
            }
            depth_arg = depth_vector_to_numpy(frame.height, frame.width, frame.depth_m);
        }
        else if (!frame.depth_m.empty())
        {
            depth_arg = depth_vector_to_numpy(frame.height, frame.width, frame.depth_m);
        }
        else
        {
            depth_arg = py::none();
        }

        py::object algo_arg = (algorithm_id < 0) ? py::object(py::none()) : py::int_(algorithm_id);

        py::object inp = impl_->algorithm_input_cls(
            bgr_vector_to_numpy(frame.height, frame.width, frame.color_bgr),
            depth_arg,
            K_to_numpy(frame.K),
            dist_to_numpy(frame.dist),
            algo_arg);

        py::object out = engine.attr("run")(inp);
        py::list targets = out.attr("targets");

        const py::ssize_t n = targets.size();
        result.targets.reserve(static_cast<size_t>(n));
        for (py::ssize_t i = 0; i < n; ++i)
        {
            py::object t = targets[i];
            PoseTargetResult one;
            one.class_id = t.attr("class_id").cast<int>();
            one.class_name = py::str(t.attr("class_name"));
            one.algorithm_id = t.attr("algorithm_id").cast<int>();
            one.confidence = t.attr("confidence").cast<float>();
            one.radius_m = t.attr("radius_m").cast<float>();
            one.success = t.attr("success").cast<bool>();
            one.message = py::str(t.attr("message"));
            one.pose_4x4 = numpy_pose_to_array(t.attr("pose_4x4"));
            result.targets.push_back(std::move(one));
        }

        result.ok = true;
        result.message = "ok";

        if (show_visualization)
        {
            const int panel = (vis_panel >= 0 && vis_panel < PoseVisPanel::Count)
                                  ? vis_panel
                                  : static_cast<int>(camera_slot_index(vis_slot));
            show_pose_visualization(frame, out, panel);
        }
    }
    catch (const py::error_already_set &e)
    {
        if (PyErr_ExceptionMatches(PyExc_KeyboardInterrupt))
        {
            PyErr_Clear();
            request_app_stop();
            result.message = "interrupted";
            return result;
        }
        result.message = std::string("run 异常: ") + e.what();
    }
    catch (const std::exception &e)
    {
        result.message = std::string("run 异常: ") + e.what();
    }
    return result;
}

void SegPoseBridge::close_visualization()
{
    py::gil_scoped_acquire gil;
    g_vis_composite_window_ready = false;
    g_vis_window_w = 0;
    g_vis_window_h = 0;
    g_vis_layout_mode = PoseVisLayout::Single;
    g_vis_single_panel_index = PoseVisPanel::Head;
    for (py::object &panel : g_vis_panel_images)
        panel = py::none();
    if (!impl_->interpreter || !g_pose_vis_gui_enabled)
        return;
    try
    {
        py::module_ cv2 = py::module_::import("cv2");
        cv2.attr("destroyAllWindows")();
    }
    catch (...)
    {
    }
}

std::string project_root_dir()
{
    if (const char *env = std::getenv("MARKET_SIMPLE_ROOT"))
        return env;

    const std::filesystem::path cwd = std::filesystem::current_path();
    if (std::filesystem::exists(cwd / "config" / "realsense_cameras.yaml"))
        return cwd.string();
    if (std::filesystem::exists(cwd / ".." / "config" / "realsense_cameras.yaml"))
        return std::filesystem::canonical(cwd / "..").string();
    return cwd.string();
}

std::string default_cameras_yaml_path()
{
    return project_root_dir() + "/config/realsense_cameras.yaml";
}

std::string default_pose_yaml_path()
{
    return project_root_dir() + "/feeding_cylindrical_parts_alg/config/pose_params.yaml";
}

std::string default_cylinder_pose_root()
{
    return project_root_dir() + "/feeding_cylindrical_parts_alg";
}

std::string default_online_pose_root()
{
    return project_root_dir() + "/online_pose";
}

std::string default_aruco_pose_yaml_path()
{
    return project_root_dir() + "/online_pose/config.yaml";
}

std::string default_board_yf100_root()
{
    return project_root_dir() + "/board_yf100_aruco";
}

std::string default_board_yf100_yaml_path()
{
    return project_root_dir() + "/board_yf100_aruco/config.yaml";
}

std::string default_board_belt_root()
{
    return project_root_dir() + "/board_belt_aruco";
}

std::string default_board_belt_yaml_path()
{
    return project_root_dir() + "/board_belt_aruco/config.yaml";
}

std::string default_camera_to_robot_yaml_path()
{
    return project_root_dir() + "/config/camera_to_base_result.yaml";
}

std::string default_left_hand_to_robot_yaml_path()
{
    return project_root_dir() + "/config/left_hand_to_base_result.yaml";
}

std::string default_right_hand_to_robot_yaml_path()
{
    return project_root_dir() + "/config/right_hand_to_base_result.yaml";
}

namespace
{

bool parse_matrix_doubles(const std::string &text, std::array<double, 16> &out)
{
    std::vector<double> vals;
    const char *p = text.c_str();
    while (*p != '\0')
    {
        char *end = nullptr;
        const double v = std::strtod(p, &end);
        if (end == p)
        {
            ++p;
            continue;
        }
        vals.push_back(v);
        p = end;
    }
    if (vals.size() != 16)
        return false;
    for (int i = 0; i < 16; ++i)
        out[static_cast<size_t>(i)] = vals[static_cast<size_t>(i)];
    return true;
}

std::array<double, 16> mat4_mul_row_major(const std::array<double, 16> &a, const std::array<double, 16> &b)
{
    std::array<double, 16> c{};
    for (int r = 0; r < 4; ++r)
    {
        for (int ccol = 0; ccol < 4; ++ccol)
        {
            double sum = 0.0;
            for (int k = 0; k < 4; ++k)
                sum += a[static_cast<size_t>(r * 4 + k)] * b[static_cast<size_t>(k * 4 + ccol)];
            c[static_cast<size_t>(r * 4 + ccol)] = sum;
        }
    }
    return c;
}

/** yaml 标定矩阵平移 mm → m（仅平移列） */
void cam2robot_translation_mm_to_m(std::array<double, 16> &cam2robot)
{
    cam2robot[3] /= 1000.0;
    cam2robot[7] /= 1000.0;
    cam2robot[11] /= 1000.0;
}

Eigen::Matrix<double, 4, 4> row_major_pose_to_matrix4(const std::array<double, 16> &pose)
{
    Eigen::Matrix<double, 4, 4> T;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            T(r, c) = pose[static_cast<size_t>(r * 4 + c)];
    return T;
}

void print_head_target_cam4x4_robot6(
    const char *label,
    const PoseTargetResult &t,
    const std::array<double, 16> &cam2robot)
{
    std::cout << "\n[" << label << "] 相机系 pose_4x4 (row-major, m):\n";
    for (int r = 0; r < 4; ++r)
    {
        for (int c = 0; c < 4; ++c)
            std::cout << std::setw(11) << std::fixed << std::setprecision(6)
                      << t.pose_4x4[static_cast<size_t>(r * 4 + c)] << " ";
        std::cout << "\n";
    }
    const std::array<double, 16> pose_robot = mat4_mul_row_major(cam2robot, t.pose_4x4);
    const Eigen::Matrix<double, 1, 6> pos6 =
        T2PosEulerAngles(row_major_pose_to_matrix4(pose_robot));
    std::cout << "[" << label << "] 基座系 1x6 (x,y,z m; rx,ry,rz rad): "
              << std::setprecision(6) << pos6(0) << ", " << pos6(1) << ", " << pos6(2)
              << ", " << pos6(3) << ", " << pos6(4) << ", " << pos6(5) << "\n";
}

} // namespace

bool load_cam2robot_matrix(const std::string &yaml_path, std::array<double, 16> &out, std::string &err)
{
    err.clear();
    std::ifstream in(yaml_path);
    if (!in)
    {
        err = "无法打开: " + yaml_path;
        return false;
    }

    std::string line;
    if (!std::getline(in, line))
    {
        err = "文件为空: " + yaml_path;
        return false;
    }

    const auto pos = line.find("matrix:");
    if (pos == std::string::npos)
    {
        err = "未找到 matrix 字段";
        return false;
    }

    if (!parse_matrix_doubles(line.substr(pos), out))
    {
        err = "matrix 需要 16 个数";
        return false;
    }
    cam2robot_translation_mm_to_m(out);
    return true;
}

std::array<double, 16> transform_pose_cam_to_robot(
    const std::array<double, 16> &cam2robot,
    const std::array<double, 16> &det_pose_cam_m)
{
    return mat4_mul_row_major(cam2robot, det_pose_cam_m);
}

namespace
{

bool project_cam_xy_to_uv(
    const std::array<double, 16> &pose_cam,
    const CameraFrameData &frame,
    double &u,
    double &v)
{
    const double z = pose_cam[11];
    if (!(z > 1e-6) || frame.width <= 1 || frame.height <= 1)
        return false;
    const double fx = frame.K[0];
    const double fy = frame.K[4];
    const double cx = frame.K[2];
    const double cy = frame.K[5];
    u = fx * (pose_cam[3] / z) + cx;
    v = fy * (pose_cam[7] / z) + cy;
    return std::isfinite(u) && std::isfinite(v);
}

bool reject_head_image_edge(
    const PoseTargetResult &t,
    const CameraFrameData &frame,
    double margin_frac,
    std::string &why)
{
    if (margin_frac <= 1e-9)
        return false;
    double u = 0.0;
    double v = 0.0;
    if (!project_cam_xy_to_uv(t.pose_4x4, frame, u, v))
        return false;
    const double mu = margin_frac * static_cast<double>(frame.width);
    const double mv = margin_frac * static_cast<double>(frame.height);
    if (u >= mu && u <= static_cast<double>(frame.width) - 1.0 - mu &&
        v >= mv && v <= static_cast<double>(frame.height) - 1.0 - mv)
        return false;
    std::ostringstream os;
    os << std::fixed << std::setprecision(1)
       << "画面边缘 u=" << u << " v=" << v
       << " 边距=" << margin_frac;
    why = os.str();
    return true;
}

} // namespace

void assign_hand_targets_in_robot_frame(
    const PoseDetectionRecords &records,
    const std::array<double, 16> &cam2robot,
    double right_hand_pos[3],
    double left_hand_pos[3],
    HeadHandAssignZones *out_zones)
{
    if (out_zones)
    {
        out_zones->r_from_side = false;
        out_zones->l_from_side = false;
        out_zones->r_from_middle = false;
        out_zones->l_from_middle = false;
    }
    auto set_invalid = [](double pos[3]) {
        pos[0] = -1.0;
        pos[1] = 0.0;
        pos[2] = 0.0;
    };
    auto set_pos = [](double pos[3], double x, double y, double z) {
        pos[0] = x;
        pos[1] = y;
        pos[2] = z;
    };

    set_invalid(right_hand_pos);
    set_invalid(left_hand_pos);

    struct Candidate
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        float confidence = 0.f;
        size_t index = 0;
    };

    const auto &valid = g_move_cfg.grasp_valid;
    const double kSplitY = column_split_y();
    const double kCamXyOverZ = g_move_cfg.grasp_zone.cam_xy_over_z_max;

    std::vector<Candidate> left_zone;
    std::vector<Candidate> right_zone;
    left_zone.reserve(records.size());
    right_zone.reserve(records.size());

    std::cout << "\n--- robot 基座系 → 列分区 (右列1-3 y<=" << kSplitY
              << ", 左列4-6 y>" << kSplitY
              << ", 策略=基座x最近, 同排容差="
              << g_move_cfg.grasp_zone.front_row_tolerance_m
              << "m, 同时最小列差=" << g_move_cfg.grasp_zone.min_simultaneous_col_delta
              << ") ---\n";

    for (size_t i = 0; i < records.size(); ++i)
    {
        const auto &t = records[i].target;
        const std::string name =
            t.class_name.empty() ? ("cls" + std::to_string(t.class_id)) : t.class_name;

        if (!t.success)
        {
            std::cout << "[" << i << "] " << name << " 跳过(FAIL: " << t.message << ")\n";
            continue;
        }
        if (t.class_id != kGraspDetectClassId)
        {
            std::cout << "[" << i << "] " << name << " 跳过(非抓取类 class=" << t.class_id << ")\n";
            continue;
        }

        const std::array<double, 16> pose_robot = transform_pose_cam_to_robot(cam2robot, t.pose_4x4);
        const double x = pose_robot[3];
        const double y = pose_robot[7];
        const double z = pose_robot[11];
        const double cx = t.pose_4x4[3];
        const double cy = t.pose_4x4[7];
        const double cz = t.pose_4x4[11];

        auto log_drop = [&](const char *reason)
        {
            std::cout << "[" << i << "] " << name
                      << " 过滤(" << reason << " x=" << std::fixed << std::setprecision(4) << x
                      << " y=" << y << " z=" << z
                      << " cam=" << cx << "," << cy << "," << cz << ")\n";
        };

        if (x < valid.x_min || x > valid.x_max)
        {
            log_drop("工作包络x");
            continue;
        }
        if (kCamXyOverZ > 1e-9 && cz > 1e-6)
        {
            const double ax = std::abs(cx / cz);
            const double ay = std::abs(cy / cz);
            if (ax > kCamXyOverZ || ay > kCamXyOverZ)
            {
                log_drop("画面张角");
                continue;
            }
        }

        Candidate c{x, y, z, t.confidence, i};
        const int col = tray_column_index_from_y(y);
        if (arm_y_allowed_right(y))
        {
            if (y < valid.right_y_min || y > valid.right_y_max)
            {
                log_drop("工作包络y");
                continue;
            }
            right_zone.push_back(c);
        }
        else if (arm_y_allowed_left(y))
        {
            if (y < valid.left_y_min || y > valid.left_y_max)
            {
                log_drop("工作包络y");
                continue;
            }
            left_zone.push_back(c);
        }
        else
        {
            log_drop("列分区");
            continue;
        }

        std::cout << "[" << i << "] " << name
                  << " conf=" << std::setprecision(3) << t.confidence
                  << " x=" << std::setprecision(4) << x
                  << " y=" << y
                  << " z=" << z
                  << " col=" << col
                  << (arm_y_allowed_right(y) ? " 右区\n" : " 左区\n");
    }

    auto pick_best = [](const std::vector<Candidate> &zone, int *used_index) -> const Candidate * {
        const double row_tol = std::max(0.0, g_move_cfg.grasp_zone.front_row_tolerance_m);
        double front_x = std::numeric_limits<double>::infinity();
        for (const Candidate &c : zone)
        {
            if (used_index != nullptr && *used_index == static_cast<int>(c.index))
                continue;
            front_x = std::min(front_x, c.x);
        }

        const Candidate *best = nullptr;
        for (const Candidate &c : zone)
        {
            if (used_index != nullptr && *used_index == static_cast<int>(c.index))
                continue;
            if (c.x > front_x + row_tol)
                continue;
            if (best == nullptr || c.confidence > best->confidence ||
                (std::abs(c.confidence - best->confidence) < 1e-6f && c.x < best->x))
                best = &c;
        }
        return best;
    };

    int right_index = -1;
    int left_index = -1;

    if (const Candidate *pick = pick_best(right_zone, nullptr))
    {
        set_pos(right_hand_pos, pick->x, pick->y, pick->z);
        right_index = static_cast<int>(pick->index);
        if (out_zones)
        {
            out_zones->r_from_side = true;
            out_zones->r_from_middle = false;
        }
    }

    if (const Candidate *pick = pick_best(left_zone, nullptr))
    {
        set_pos(left_hand_pos, pick->x, pick->y, pick->z);
        left_index = static_cast<int>(pick->index);
        if (out_zones)
        {
            out_zones->l_from_side = true;
            out_zones->l_from_middle = false;
        }
    }

    std::cout << "右手: ";
    if (right_hand_pos[0] < 0.0)
        std::cout << "无效\n";
    else
    {
        std::cout << "x=" << std::setprecision(4) << right_hand_pos[0]
                  << " y=" << right_hand_pos[1]
                  << " z=" << right_hand_pos[2]
                  << " col=" << tray_column_index_from_y(right_hand_pos[1]) << " (右三列)\n";
    }

    std::cout << "左手: ";
    if (left_hand_pos[0] < 0.0)
        std::cout << "无效\n";
    else
    {
        std::cout << "x=" << std::setprecision(4) << left_hand_pos[0]
                  << " y=" << left_hand_pos[1]
                  << " z=" << left_hand_pos[2]
                  << " col=" << tray_column_index_from_y(left_hand_pos[1]) << " (左三列)\n";
    }

    std::cout << "\n--- 头部分配目标：相机 4x4 / 基座 1x6 ---\n";
    if (right_index >= 0)
        print_head_target_cam4x4_robot6("右手目标", records[static_cast<size_t>(right_index)].target, cam2robot);
    else
        std::cout << "[右手目标] 无有效分配\n";
    if (left_index >= 0)
        print_head_target_cam4x4_robot6("左手目标", records[static_cast<size_t>(left_index)].target, cam2robot);
    else
        std::cout << "[左手目标] 无有效分配\n";
}

bool transform_grasp_detection_to_pose(
    const PoseDetectionRecord &record,
    const std::array<double, 16> &cam2robot,
    Eigen::Matrix<double, 1, 6> &out_pose)
{
    out_pose << -1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    const PoseTargetResult &t = record.target;
    if (!t.success || t.class_id != kGraspDetectClassId)
        return false;
    const std::array<double, 16> pose_robot =
        transform_pose_cam_to_robot(cam2robot, t.pose_4x4);
    out_pose = T2PosEulerAngles(row_major_pose_to_matrix4(pose_robot));
    return true;
}

PosePipeline::PosePipeline() = default;

PosePipeline::~PosePipeline()
{
    shutdown(false);
}

bool PosePipeline::init(std::string &err)
{
    err.clear();
    root_ = project_root_dir();

    std::cout << "========== 位姿检测 ==========\n";
    std::cout << "工程根: " << root_ << "\n";

    std::cout << "\n[1] 初始化相机 ... ";
    cameras_ = std::make_unique<RealSenseMultiCam>(
        default_cameras_yaml_path(),
        g_move_cfg.cameras.width,
        g_move_cfg.cameras.height,
        g_move_cfg.cameras.head_fps,
        g_move_cfg.cameras.hand_fps);
    if (!cameras_->init(err))
    {
        std::cout << "失败\n";
        cameras_.reset();
        return false;
    }
    std::cout << "OK\n";

    std::cout << "[2] 初始化算法引擎 ... ";
    bridge_ = std::make_unique<SegPoseBridge>();
    if (!bridge_->init(default_pose_yaml_path(), default_cylinder_pose_root(), err))
    {
        std::cout << "失败\n";
        cameras_->stop();
        cameras_.reset();
        bridge_.reset();
        return false;
    }
    std::cout << "OK\n";

    std::string aerr;
    std::cout << "[3] 初始化头相机 ArUco 位姿 (online_pose) ... ";
    if (!bridge_->init_aruco_engine(default_aruco_pose_yaml_path(), aerr))
    {
        std::cout << "跳过\n";
        std::cerr << "[aruco] " << aerr << "（网页「二维码位姿」不可用，抓取不受影响）\n";
    }
    else
    {
        std::cout << "OK\n";
    }

    std::string terr;
    std::cout << "[4] 初始化料盘 ArUco+孔位 (board_yf100) ... ";
    if (!bridge_->init_tray_engine(default_board_yf100_yaml_path(), terr))
    {
        std::cout << "跳过\n";
        std::cerr << "[tray] " << terr << "（料盘孔位不可用时不抓，不回退 YOLO）\n";
    }
    else
    {
        std::cout << "OK\n";
    }

    std::string berr;
    std::cout << "[5] 初始化传送带 ArUco (board_belt_aruco 6x6，独立于料盘) ... ";
    if (!bridge_->init_belt_engine(default_board_belt_yaml_path(), berr))
    {
        std::cout << "跳过\n";
        std::cerr << "[belt] " << berr << "（传送带二维码不可用，料盘抓取不受影响）\n";
    }
    else
    {
        std::cout << "OK\n";
    }

    install_app_sigint_handler();
    return true;
}

void PosePipeline::shutdown(bool close_visualization)
{
    if (bridge_)
    {
        if (close_visualization)
            bridge_->close_visualization();
        bridge_.reset();
    }
    if (cameras_)
    {
        cameras_->stop();
        cameras_.reset();
    }
}

RealSenseMultiCam &PosePipeline::cameras()
{
    return *cameras_;
}

SegPoseBridge &PosePipeline::bridge()
{
    return *bridge_;
}

void print_pose_run_result(const PoseRunResult &result, const char *slot_label, int algorithm_id)
{
    std::cout << "\n========== " << slot_label << " | 算法 " << algorithm_id << " ==========\n";
    if (!result.ok)
    {
        std::cout << "失败: " << result.message << "\n";
        return;
    }

    if (result.targets.empty())
    {
        std::cout << "未检测到目标\n";
        return;
    }

    for (size_t i = 0; i < result.targets.size(); ++i)
    {
        const auto &t = result.targets[i];
        const double tx = t.pose_4x4[3];
        const double ty = t.pose_4x4[7];
        const double tz = t.pose_4x4[11];
        const std::string status = t.success ? "OK" : ("FAIL(" + t.message + ")");
        const std::string name = t.class_name.empty() ? ("cls" + std::to_string(t.class_id)) : t.class_name;

        std::cout << "[" << i << "] " << name
                  << " algo=" << t.algorithm_id
                  << " conf=" << std::fixed << std::setprecision(3) << t.confidence
                  << " r=" << std::setprecision(4) << t.radius_m << "m "
                  << status
                  << " pos_m=(" << std::setprecision(4) << tx << "," << ty << "," << tz << ")\n";

        std::cout << "pose_4x4 (row-major, m):\n";
        for (int r = 0; r < 4; ++r)
        {
            std::cout << "  ";
            for (int c = 0; c < 4; ++c)
                std::cout << std::setw(11) << std::setprecision(6) << t.pose_4x4[static_cast<size_t>(r * 4 + c)] << " ";
            std::cout << "\n";
        }
    }
}

int algorithm_id_for_slot(CameraSlot slot)
{
    switch (slot)
    {
    case CameraSlot::Head:
        return g_move_cfg.vision_detect.head_grasp;
    case CameraSlot::RightHand:
        return g_move_cfg.vision_detect.right_hand_grasp;
    case CameraSlot::LeftHand:
        return g_move_cfg.vision_detect.left_hand_grasp;
    default:
        return -1;
    }
}

PoseDetectionRecords pose_records_from_run(
    const PoseRunResult &result,
    CameraSlot slot,
    const CameraFrameData &frame,
    int filter_class_id,
    bool apply_head_edge_filter)
{
    PoseDetectionRecords records;
    if (!result.ok)
        return records;

    const char *slot_label = RealSenseMultiCam::slot_name(slot);
    const int algorithm_id = algorithm_id_for_slot(slot);
    const double edge_frac =
        (apply_head_edge_filter && slot == CameraSlot::Head)
            ? g_move_cfg.grasp_zone.edge_margin_frac
            : 0.0;
    for (const PoseTargetResult &t : result.targets)
    {
        if (filter_class_id >= 0 && t.class_id != filter_class_id)
            continue;
        if (!t.success)
            continue;
        if (slot == CameraSlot::Head && edge_frac > 1e-9)
        {
            std::string why;
            if (reject_head_image_edge(t, frame, edge_frac, why))
            {
                const std::string name =
                    t.class_name.empty() ? ("cls" + std::to_string(t.class_id)) : t.class_name;
                std::cout << "[head] " << name << " 过滤(" << why << ")\n";
                continue;
            }
        }
        PoseDetectionRecord rec;
        rec.slot = slot;
        rec.slot_name = slot_label ? slot_label : "";
        rec.frame_algorithm_id = (algorithm_id >= 0) ? algorithm_id : t.algorithm_id;
        rec.target = t;
        records.push_back(std::move(rec));
    }
    return records;
}

PoseDetectionRecords detect_pose_at_slot(
    RealSenseMultiCam &cameras,
    SegPoseBridge &bridge,
    CameraSlot slot,
    bool show_visualization,
    int filter_class_id,
    SegEngineId engine_id)
{
    if (app_stop_requested() || hardware_abort_requested())
        return {};

    const char *slot_label = RealSenseMultiCam::slot_name(slot);

    CameraFrameData frame;
    if (slot == CameraSlot::Head)
    {
        frame = cameras.grab(slot);
    }
    else
    {
        cameras.flush(slot);
        frame = cameras.grab_fresh(slot);
    }
    if (!frame.ok)
    {
        if (!app_stop_requested())
            std::cerr << "[" << slot_label << "] 取帧失败: " << frame.message << std::endl;
        return {};
    }

    frame = RealSenseMultiCam::prepare_frame_for_slot(std::move(frame), slot);

    const int algorithm_id = algorithm_id_for_slot(slot);
    const PoseRunResult result = bridge.run(
        frame, algorithm_id, show_visualization, slot, -1, engine_id);
    if (!result.ok)
    {
        if (!app_stop_requested())
            std::cerr << "[" << slot_label << "] 算法失败: " << result.message << std::endl;
        return {};
    }

    PoseDetectionRecords records = pose_records_from_run(
        result, slot, frame, filter_class_id, slot == CameraSlot::Head);

    if (records.empty())
    {
        if (filter_class_id >= 0)
            std::cout << "[" << slot_label << "] 未检测到 class=" << filter_class_id << " 目标\n";
        else
            std::cout << "[" << slot_label << "] 未检测到目标\n";
    }

    return records;
}

void append_pose_records(
    PoseDetectionRecords &records,
    const char *slot_name,
    int frame_algorithm_id,
    const PoseRunResult &result)
{
    if (!result.ok)
        return;

    for (const PoseTargetResult &t : result.targets)
    {
        PoseDetectionRecord rec;
        rec.slot_name = slot_name ? slot_name : "";
        rec.frame_algorithm_id = frame_algorithm_id;
        rec.target = t;
        records.push_back(std::move(rec));
    }
}

void print_pose_records_summary(const char *slot_title, const PoseDetectionRecords &records)
{
    std::cout << "\n--- " << slot_title << " (" << records.size() << " 条) ---\n";
    if (records.empty())
    {
        std::cout << "(无)\n";
        return;
    }

    for (size_t i = 0; i < records.size(); ++i)
    {
        const auto &rec = records[i];
        const auto &t = rec.target;
        const double tx = t.pose_4x4[3];
        const double ty = t.pose_4x4[7];
        const double tz = t.pose_4x4[11];
        const std::string name = t.class_name.empty() ? ("cls" + std::to_string(t.class_id)) : t.class_name;
        const std::string status = t.success ? "OK" : ("FAIL(" + t.message + ")");

        std::cout << "[" << i << "] " << name
                  << " cls=" << t.class_id
                  << " algo=" << t.algorithm_id
                  << " frame_algo=" << rec.frame_algorithm_id
                  << " conf=" << std::fixed << std::setprecision(3) << t.confidence
                  << " r=" << std::setprecision(4) << t.radius_m << "m "
                  << status
                  << " pos_m=(" << std::setprecision(4) << tx << "," << ty << "," << tz << ")\n";

        std::cout << "  pose_4x4:\n";
        for (int r = 0; r < 4; ++r)
        {
            std::cout << "    ";
            for (int c = 0; c < 4; ++c)
                std::cout << std::setw(11) << std::setprecision(6)
                          << t.pose_4x4[static_cast<size_t>(r * 4 + c)] << " ";
            std::cout << "\n";
        }
    }
}
