#include "seg_pose_bridge.h"
#include "place_grid_correct.h"

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

/** 使用 human_interaction_env（与 seg_circle_pose/main.py 相同 conda 环境） */
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

constexpr const char *kPoseVisWindow = "pose_vis_all";
constexpr int kPanelDispW = 640;
constexpr int kPanelDispH = 480;

std::array<py::object, PoseVisPanel::Count> g_vis_panel_images{
    py::none(), py::none(), py::none(), py::none()};
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
    case PoseVisPanel::HeadPlace:
        return "head_place";
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
    case PoseVisPanel::HeadPlace:
        return "HeadPlace(cls1)";
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
    py::object factory_engine;
    py::object jindi_engine;
    py::object algorithm_input_cls;
    py::object engine_cls;
    py::object load_pose_params;
    py::object aruco_detect_fn;
    bool factory_ready = false;
    bool jindi_ready = false;
    bool aruco_ready = false;
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

        py::module_ sys = py::module_::import("sys");
        sys.attr("path").attr("insert")(0, seg_root);

        py::module_ config_mod = py::module_::import("algorithm.config_loader");
        py::module_ engine_mod = py::module_::import("algorithm.engine");
        py::module_ types_mod = py::module_::import("algorithm.types");

        impl_->load_pose_params = config_mod.attr("load_pose_params");
        impl_->engine_cls = engine_mod.attr("CirclePoseEngine");
        py::object params = impl_->load_pose_params(pose_config_path);
        impl_->engine = impl_->engine_cls(params);
        impl_->factory_engine = py::none();
        impl_->factory_ready = false;
        impl_->jindi_engine = py::none();
        impl_->jindi_ready = false;
        impl_->aruco_detect_fn = py::none();
        impl_->aruco_ready = false;
        impl_->algorithm_input_cls = types_mod.attr("AlgorithmInput");

        std::cout << "CirclePoseEngine 已加载 (feeding_cylindrical_parts_alg), 配置: "
                  << pose_config_path << std::endl;
        return true;
    }
    catch (const std::exception &e)
    {
        err = std::string("SegPoseBridge 初始化失败: ") + e.what();
        impl_->engine = py::none();
        impl_->factory_engine = py::none();
        impl_->factory_ready = false;
        impl_->jindi_engine = py::none();
        impl_->jindi_ready = false;
        impl_->aruco_detect_fn = py::none();
        impl_->aruco_ready = false;
        return false;
    }
}

bool SegPoseBridge::init_factory_engine(const std::string &pose_config_path, std::string &err)
{
    err.clear();
    if (!impl_->interpreter || !impl_->engine_cls || impl_->engine_cls.is_none())
    {
        err = "先初始化抓取引擎后再加载工厂检测模型";
        return false;
    }
    try
    {
        py::gil_scoped_acquire gil;
        py::object params = impl_->load_pose_params(pose_config_path);
        impl_->factory_engine = impl_->engine_cls(params);
        impl_->factory_ready = true;
        std::cout << "工厂 CirclePoseEngine 已加载 (单类 best.pt), 配置: "
                  << pose_config_path << std::endl;
        return true;
    }
    catch (const std::exception &e)
    {
        err = std::string("工厂检测引擎初始化失败: ") + e.what();
        impl_->factory_engine = py::none();
        impl_->factory_ready = false;
        return false;
    }
}

bool SegPoseBridge::factory_engine_ready() const
{
    return impl_ && impl_->factory_ready;
}

bool SegPoseBridge::init_jindi_engine(const std::string &pose_config_path, std::string &err)
{
    err.clear();
    if (!impl_->interpreter || !impl_->engine_cls || impl_->engine_cls.is_none())
    {
        err = "先初始化抓取引擎后再加载金帝四类模型";
        return false;
    }
    try
    {
        py::gil_scoped_acquire gil;
        py::object params = impl_->load_pose_params(pose_config_path);
        impl_->jindi_engine = impl_->engine_cls(params);
        impl_->jindi_ready = true;
        std::cout << "金帝四类 CirclePoseEngine 已加载 (seg_model/best.pt), 配置: "
                  << pose_config_path << std::endl;
        return true;
    }
    catch (const std::exception &e)
    {
        err = std::string("金帝四类引擎初始化失败: ") + e.what();
        impl_->jindi_engine = py::none();
        impl_->jindi_ready = false;
        return false;
    }
}

bool SegPoseBridge::jindi_engine_ready() const
{
    return impl_ && impl_->jindi_ready;
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
        if (load_cam2robot_matrix(cam2robot_yaml_for_slot(vis_slot), cam2robot, ext_err))
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
    py::object engine = impl_->engine;
    const char *missing = "引擎未初始化";
    if (engine_id == SegEngineId::Factory)
    {
        engine = impl_->factory_engine;
        missing = "工厂检测引擎未初始化";
    }
    else if (engine_id == SegEngineId::Jindi)
    {
        engine = impl_->jindi_engine;
        missing = "金帝四类引擎未初始化";
    }
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

std::string default_factory_pose_yaml_path()
{
    return project_root_dir() + "/feeding_cylindrical_parts_alg/config/pose_params_factory.yaml";
}

std::string default_jindi_pose_yaml_path()
{
    return project_root_dir() + "/feeding_cylindrical_parts_alg/config/pose_params_jindi.yaml";
}

std::string default_seg_circle_pose_root()
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
        double dist_sq_cam = 0.0; // 头相机坐标系下距光心距离平方
    };

    const double kMinX = g_move_cfg.grasp_zone.x_min;
    const double kMaxX = g_move_cfg.grasp_zone.x_max;
    const double kMaxAbsY = g_move_cfg.grasp_zone.y_max_abs;
    const double kZoneSplit = g_move_cfg.grasp_zone.y_side_split;
    const double kCamXyOverZ = g_move_cfg.grasp_zone.cam_xy_over_z_max;
    const bool kUseXMin = kMinX < kMaxX;

    std::vector<Candidate> left_zone;
    std::vector<Candidate> right_zone;
    std::vector<Candidate> middle_r_zone;
    std::vector<Candidate> middle_l_zone;
    left_zone.reserve(records.size());
    right_zone.reserve(records.size());
    middle_r_zone.reserve(records.size());
    middle_l_zone.reserve(records.size());

    std::cout << "\n--- robot 基座系 → 左右手分配 (cfg: 侧区|y|>" << kZoneSplit
              << ", 中间[-" << kZoneSplit << "," << kZoneSplit << "] y=0分左右) ---\n";

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

        if (x > kMaxX || (kUseXMin && x < kMinX) || y > kMaxAbsY || y < -kMaxAbsY)
        {
            log_drop("工作区");
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

        const double dist_sq_cam = cx * cx + cy * cy + cz * cz;
        Candidate c{x, y, z, t.confidence, i, dist_sq_cam};
        if (y > kZoneSplit)
            left_zone.push_back(c);
        else if (y < -kZoneSplit)
            right_zone.push_back(c);
        else if (y <= 0.0)
            middle_r_zone.push_back(c);
        else
            middle_l_zone.push_back(c);

        std::cout << "[" << i << "] " << name
                  << " conf=" << std::setprecision(3) << t.confidence
                  << " x=" << std::setprecision(4) << x
                  << " y=" << y
                  << " z=" << z << "\n";
    }

    auto pick_best = [](const std::vector<Candidate> &zone, int *used_index) -> const Candidate * {
        const Candidate *best = nullptr;
        for (const Candidate &c : zone)
        {
            if (used_index != nullptr && *used_index == static_cast<int>(c.index))
                continue;
            if (best == nullptr || c.dist_sq_cam < best->dist_sq_cam)
                best = &c;
        }
        return best;
    };

    int used_index = -1;
    int right_index = -1;
    int left_index = -1;

    if (const Candidate *pick = pick_best(right_zone, nullptr))
    {
        set_pos(right_hand_pos, pick->x, pick->y, pick->z);
        used_index = static_cast<int>(pick->index);
        right_index = used_index;
        if (out_zones)
            out_zones->r_from_side = true;
    }
    else if (const Candidate *pick = pick_best(middle_r_zone, nullptr))
    {
        set_pos(right_hand_pos, pick->x, pick->y, pick->z);
        used_index = static_cast<int>(pick->index);
        right_index = used_index;
        if (out_zones)
            out_zones->r_from_middle = true;
    }

    if (const Candidate *pick = pick_best(left_zone, nullptr))
    {
        set_pos(left_hand_pos, pick->x, pick->y, pick->z);
        left_index = static_cast<int>(pick->index);
        if (used_index < 0)
            used_index = left_index;
        if (out_zones)
            out_zones->l_from_side = true;
    }
    else if (const Candidate *pick = pick_best(middle_l_zone, used_index >= 0 ? &used_index : nullptr))
    {
        set_pos(left_hand_pos, pick->x, pick->y, pick->z);
        left_index = static_cast<int>(pick->index);
        if (out_zones)
            out_zones->l_from_middle = true;
    }

    std::cout << "右手: ";
    if (right_hand_pos[0] < 0.0)
        std::cout << "无效\n";
    else
    {
        const bool right_middle =
            out_zones != nullptr && out_zones->r_from_middle && !out_zones->r_from_side;
        std::cout << "x=" << std::setprecision(4) << right_hand_pos[0]
                  << " y=" << right_hand_pos[1]
                  << " z=" << right_hand_pos[2]
                  << (right_middle ? " (中间区)\n" : " (右区)\n");
    }

    std::cout << "左手: ";
    if (left_hand_pos[0] < 0.0)
        std::cout << "无效\n";
    else
    {
        const bool left_middle =
            out_zones != nullptr && out_zones->l_from_middle && !out_zones->l_from_side;
        std::cout << "x=" << std::setprecision(4) << left_hand_pos[0]
                  << " y=" << left_hand_pos[1]
                  << " z=" << left_hand_pos[2]
                  << (left_middle ? " (中间区)\n" : " (左区)\n");
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

void assign_nearest_hand_cam_target_in_robot_frame(
    const PoseDetectionRecords &records,
    const std::array<double, 16> &cam2robot,
    Eigen::Matrix<double, 1, 6> &out_pose)
{
    out_pose << -1.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    const auto *best = static_cast<const PoseDetectionRecord *>(nullptr);
    double best_dist_sq = 0.0;

    for (const PoseDetectionRecord &rec : records)
    {
        const auto &t = rec.target;
        if (!t.success || t.class_id != kGraspDetectClassId)
            continue;

        const double x = t.pose_4x4[3];
        const double y = t.pose_4x4[7];
        const double z = t.pose_4x4[11];
        const double dist_sq = x * x + y * y + z * z;

        if (best == nullptr || dist_sq < best_dist_sq)
        {
            best = &rec;
            best_dist_sq = dist_sq;
        }
    }

    if (best == nullptr)
    {
        std::cout << "[hand_cam] 无有效目标\n";
        return;
    }

    const std::array<double, 16> pose_robot =
        transform_pose_cam_to_robot(cam2robot, best->target.pose_4x4);
    out_pose = T2PosEulerAngles(row_major_pose_to_matrix4(pose_robot));

    const auto &t = best->target;
    const std::string name =
        t.class_name.empty() ? ("cls" + std::to_string(t.class_id)) : t.class_name;
    std::cout << "[hand_cam] " << name
              << " conf=" << std::fixed << std::setprecision(3) << t.confidence
              << " robot=(" << std::setprecision(4) << out_pose(0) << ","
              << out_pose(1) << "," << out_pose(2) << ","
              << out_pose(3) << "," << out_pose(4) << "," << out_pose(5) << ")\n";
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
    if (!bridge_->init(default_pose_yaml_path(), default_seg_circle_pose_root(), err))
    {
        std::cout << "失败\n";
        cameras_->stop();
        cameras_.reset();
        bridge_.reset();
        return false;
    }
    std::cout << "OK\n";

    std::string ferr;
    std::cout << "[3] 初始化工厂检测引擎 (根目录单类 best.pt) ... ";
    if (!bridge_->init_factory_engine(default_factory_pose_yaml_path(), ferr))
    {
        std::cout << "跳过\n";
        std::cerr << "[factory] " << ferr << "（网页「视觉检测」不可用，抓取不受影响）\n";
    }
    else
    {
        std::cout << "OK\n";
    }

    std::string jerr;
    std::cout << "[4] 初始化金帝四类引擎 (seg_model/best.pt) ... ";
    if (!bridge_->init_jindi_engine(default_jindi_pose_yaml_path(), jerr))
    {
        std::cout << "跳过\n";
        std::cerr << "[jindi] " << jerr << "（网页「视觉检测(金帝四类)」不可用，其它识别不受影响）\n";
    }
    else
    {
        std::cout << "OK\n";
    }

    std::string aerr;
    std::cout << "[5] 初始化 ArUco 单码位姿 (online_pose) ... ";
    if (!bridge_->init_aruco_engine(default_aruco_pose_yaml_path(), aerr))
    {
        std::cout << "跳过\n";
        std::cerr << "[aruco] " << aerr << "（网页「二维码位姿」不可用，抓取不受影响）\n";
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

int algorithm_id_for_place_holes()
{
    return g_move_cfg.vision_detect.place_holes;
}

int place_pose_row_from_x(double x_robot_m)
{
    const auto &bounds = g_move_cfg.place.row_x_bounds;
    for (size_t r = 0; r < bounds.size(); ++r)
    {
        if (x_robot_m >= bounds[r].x_min && x_robot_m < bounds[r].x_max)
            return static_cast<int>(r);
    }
    return -1;
}

namespace
{

Eigen::Matrix<double, 1, 6> pose_target_to_robot6(
    const std::array<double, 16> &cam2robot,
    const PoseTargetResult &t)
{
    const std::array<double, 16> pose_robot = transform_pose_cam_to_robot(cam2robot, t.pose_4x4);
    return T2PosEulerAngles(row_major_pose_to_matrix4(pose_robot));
}

std::vector<Eigen::Matrix<double, 1, 6>> place_targets_to_robot6(
    const std::array<double, 16> &cam2robot,
    const PoseRunResult &result)
{
    std::vector<Eigen::Matrix<double, 1, 6>> out;
    out.reserve(result.targets.size());
    for (const PoseTargetResult &t : result.targets)
    {
        if (!t.success || t.class_id != kPlaceDetectClassId)
            continue;
        out.push_back(pose_target_to_robot6(cam2robot, t));
    }
    return out;
}

void sort_poses_xy(std::vector<Eigen::Matrix<double, 1, 6>> &poses)
{
    std::sort(
        poses.begin(),
        poses.end(),
        [](const Eigen::Matrix<double, 1, 6> &a, const Eigen::Matrix<double, 1, 6> &b)
        {
            if (a(0) != b(0))
                return a(0) < b(0);
            return a(1) < b(1);
        });
}

} // namespace

PlaceHoleDetectResult detect_place_holes_with_consensus(
    SegPoseBridge &bridge,
    RealSenseMultiCam &cameras,
    const std::array<double, 16> &cam2robot,
    int num_trials,
    double /*pos_tol_m*/,
    double /*rot_tol_rad*/,
    bool show_place_visualization)
{
    PlaceHoleDetectResult out;
    out.num_trials = 1;
    (void)num_trials;

    if (show_place_visualization)
        pose_vis_begin_phase(PoseVisLayout::Single, PoseVisPanel::HeadPlace, {PoseVisPanel::HeadPlace});

    CameraFrameData frame = cameras.grab(CameraSlot::Head);
    if (!frame.ok)
    {
        out.message = std::string("取帧失败: ") + frame.message;
        std::cerr << "[place_det] " << out.message << "\n";
        return out;
    }
    frame = RealSenseMultiCam::prepare_frame_for_slot(std::move(frame), CameraSlot::Head);

    const int algorithm_id = algorithm_id_for_place_holes();
    PoseRunResult detect = bridge.run(
        frame,
        algorithm_id,
        show_place_visualization,
        CameraSlot::Head,
        PoseVisPanel::HeadPlace);

    if (!detect.ok)
    {
        out.message = std::string("算法失败: ") + detect.message;
        std::cerr << "[place_det] " << out.message << "\n";
        return out;
    }

    out.poses_robot = place_targets_to_robot6(cam2robot, detect);
    sort_poses_xy(out.poses_robot);

    place_grid_session_update_from_pose_run(detect, cam2robot, "head_place");

    if (out.poses_robot.empty())
    {
        out.message = "未检测到空位(class 1)";
        std::cerr << "[place_det] " << out.message << "\n";
        return out;
    }

    out.ok = true;
    out.cluster_size = 1;
    out.picked_trial_index = 0;
    out.message = "空位检测 OK: 空位=" + std::to_string(out.poses_robot.size());
    std::cout << "[place_det] " << out.message << "\n";
    return out;
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

    int algorithm_id = algorithm_id_for_slot(slot);
    if (engine_id == SegEngineId::Factory)
        algorithm_id = 1;
    else if (engine_id == SegEngineId::Jindi)
        algorithm_id = -1;
    const PoseRunResult result = bridge.run(
        frame, algorithm_id, show_visualization, slot, -1, engine_id);
    if (!result.ok)
    {
        if (!app_stop_requested())
            std::cerr << "[" << slot_label << "] 算法失败: " << result.message << std::endl;
        return {};
    }

    PoseDetectionRecords records;
    const double edge_frac =
        (slot == CameraSlot::Head) ? g_move_cfg.grasp_zone.edge_margin_frac : 0.0;
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
