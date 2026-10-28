#pragma once

/** 视觉调试图 / 重可视化统一开关与路径。
 *
 *  环境变量 T170_VISION_DEBUG：
 *    0 或未设置：业务路径不写 picture_debug、不弹窗（默认）
 *    1：写标注图 + 原图/标注成对图
 *    2：在 1 基础上允许 OpenCV GUI（本机无显示器勿开）
 *
 *  兼容旧名 kDebugVisualize：见 move_box_runtime.h。
 */

#include <string>

/** 进程启动时读环境变量；可重复调用。 */
void vision_debug_init_from_env();

/** 是否给 detector 开标注叠加、并把 show_visualization 置真。 */
bool vision_debug_annotate();

/** 是否写入 picture_debug 下的原图/标注成对文件（pose_vis_save_*）。 */
bool vision_debug_save_pairs();

/** 是否允许 imshow 窗口。 */
bool vision_debug_gui();

/** 当前级别 0/1/2。 */
int vision_debug_level();

/** 工程根下 picture_debug 目录（无尾斜杠）。 */
std::string vision_debug_root_dir();

/** 统一命名：picture_debug/<slot>/<stem>_YYYYMMDD_HHMMSS_mmm.jpg
 *  slot 例：head / left_hand / right_hand；stem 例：tray_yolo / belt_aruco。 */
std::string vision_debug_make_path(const char *slot, const char *stem);

/** 有标注时返回路径，否则空串（传给 Python 时用空=不写盘）。 */
std::string vision_debug_annotate_path(const char *slot, const char *stem);
