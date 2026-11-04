#pragma once
#include "t170/vision/perception_types.h"
#include <initializer_list>
#include <cstddef>

/** 切换布局；Single 时 single_panel_index 指定显示的相机。 */
void pose_vis_set_layout(int layout, int single_panel_index = PoseVisPanel::Head);

/** 清空指定窗格为黑图（py::none），并刷新当前布局窗口 */
void pose_vis_clear_panels(std::initializer_list<int> panels);

/** 新一轮识别开始前：清空旧图 → 切换布局 → 刷新（等待本次识别结果再显示） */
void pose_vis_begin_phase(
    int layout,
    int single_panel_index,
    std::initializer_list<int> clear_panels);

/** 与 T170_VISION_DEBUG / vision_debug_save_pairs 联动：保存原图与标注成对图。 */
void pose_vis_set_save_debug(bool enable);

/** 是否弹出 OpenCV 窗口（仅 T170_VISION_DEBUG=2）。无显示器勿开，imshow 会 abort */
void pose_vis_set_gui_enabled(bool enable);

/** 保存一对调试图+原图（同名、各分子目录）；某一侧 ptr 为空或 bytes=0 则只存另一侧 */
void pose_vis_save_debug_pair_bgr(
    int panel_index,
    int orig_h,
    int orig_w,
    const uint8_t *orig_bgr,
    size_t orig_bytes,
    int proc_h,
    int proc_w,
    const uint8_t *proc_bgr,
    size_t proc_bytes);

/** 设置某一窗格 BGR 图（H×W×3 行连续）并刷新调试窗口 */
void pose_vis_set_panel_bgr(int panel_index, int height, int width, const uint8_t *bgr, size_t bgr_bytes);
void pose_vis_refresh_composite();

