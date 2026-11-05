#include "t170/vision/visualization.h"
#include "t170/common/paths.h"
#include "python_arrays.h"
#include "python_visualization.h"
#include <pybind11/embed.h>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <ctime>
#include <algorithm>
namespace py = pybind11;
namespace vision_bridge_detail {
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
    py::module_ vis_mod = py::module_::import("t170_vision.visualization.detection");

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

void close_visualization(bool have_interpreter)
{
    py::gil_scoped_acquire gil;
    g_vis_composite_window_ready = false;
    g_vis_window_w = 0;
    g_vis_window_h = 0;
    g_vis_layout_mode = PoseVisLayout::Single;
    g_vis_single_panel_index = PoseVisPanel::Head;
    for (py::object &panel : g_vis_panel_images)
        panel = py::none();
    if (!have_interpreter || !g_pose_vis_gui_enabled)
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
}

using namespace vision_bridge_detail;
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

