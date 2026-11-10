#pragma once

/** 视觉调试图路径。业务识别图写到 debug_picture，不看环境变量。
 *
 *  环境变量 T170_VISION_DEBUG：
 *    0 或未设置：不弹窗，成对重可视化不写盘（默认）
 *    1：额外写原图/标注成对图
 *    2：在 1 基础上允许 OpenCV GUI（本机无显示器勿开）
 *
 *  目录：debug_picture/head、right、left，原图在 debug_picture/original/<槽位>。
 *  文件名以日期开头：YYYYMMDD_HHMMSS_mmm_<内容>.jpg
 *
 *  兼容旧名 kDebugVisualize：见 move_box_runtime.h。
 */

#include <string>

/** 进程启动时读环境变量；可重复调用。 */
void vision_debug_init_from_env();

/** 是否给 detector 开标注叠加、并把 show_visualization 置真。 */
bool vision_debug_annotate();

/** 是否额外写入原图/标注成对文件（pose_vis_save_*）。 */
bool vision_debug_save_pairs();

/** 是否允许 imshow 窗口。 */
bool vision_debug_gui();

/** 当前级别 0/1/2。 */
int vision_debug_level();

/** 工程根下 debug_picture 目录（无尾斜杠）。 */
std::string vision_debug_root_dir();

/** 业务图路径，不受 T170_VISION_DEBUG 影响。slot：head / right / left，或 original/head。 */
std::string vision_debug_record_path(const char *slot, const char *stem);

/** 仅在 T170_VISION_DEBUG≥1 时返回成对调试图路径，否则空串。 */
std::string vision_debug_make_path(const char *slot, const char *stem);

/** 有额外标注开关时返回路径，否则空串。 */
std::string vision_debug_annotate_path(const char *slot, const char *stem);
