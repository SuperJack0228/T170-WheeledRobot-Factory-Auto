#pragma once

/** 无底盘调试运行时：仅保留圆柱抓取、头相机 ArUco 与安全归位。 */

#include "Ti5_Arm.h"
#include "gripper_interface.hpp"
#include "seg_pose_bridge.h"
#include "waist.h"

#include <array>
#include <atomic>
#include <limits>
#include <string>
#include <vector>

class PosePipeline;

namespace orch
{

struct ArucoHit
{
    std::string slot;
    int id = -1;
    double side_m = 0.0;
    double reproj_px = 0.0;
    double cam_x = 0.0, cam_y = 0.0, cam_z = 0.0;
    double cam_yaw = 0.0, cam_pitch = 0.0, cam_roll = 0.0;
    bool robot_ok = false;
    double robot_x = 0.0, robot_y = 0.0, robot_z = 0.0;
    double robot_rx_deg = 0.0, robot_ry_deg = 0.0, robot_rz_deg = 0.0;
    std::array<double, 16> pose_robot_4x4{};
};

struct HwSession
{
    gripper::Gripper *g = nullptr;
    Robot_Arm *arm_r = nullptr;
    Robot_Arm *arm_l = nullptr;
    WaistRobot *waist = nullptr;
    Eigen::Matrix<double, 1, 6> *layer3 = nullptr;
    PosePipeline *pipeline = nullptr;
    std::atomic<bool> *abort = nullptr;
    bool last_grasped_r = false;
    bool last_grasped_l = false;
    int last_grasp_status = 0;
    int last_grasp_count = 0;
    int last_aruco_head = 0;
    int last_aruco_status = 0;
    std::vector<ArucoHit> last_aruco_hits;
    bool last_tray_ok = false;
    std::string last_tray_message;
    std::string last_tray_save_path;
    double last_tray_reproj_px = -1.0;
    double last_tray_tilt_deg = std::numeric_limits<double>::quiet_NaN();
    std::vector<int> last_tray_used_ids;
    std::vector<TrayHoleResult> last_tray_holes;
    std::string last_debug_photo;
    bool last_belt_ok = false;
    std::string last_belt_name;
    std::string last_belt_message;
    double last_belt_x = std::numeric_limits<double>::quiet_NaN();
    double last_belt_y = std::numeric_limits<double>::quiet_NaN();
    double last_belt_z = std::numeric_limits<double>::quiet_NaN();
    bool last_chassis_ok = false;
    std::string last_chassis_station;
    std::string last_chassis_message;
};

extern HwSession *g_hw;

int go_home(Robot_Arm &arm_r, Robot_Arm &arm_l);
/** 调试：仅平移腰 X。1=layer3_home.x，2=home_x + far_row_forward_m（前伸位，并低头 far_pitch 后截一帧）。 */
int go_waist_debug(int pose);
/** 传送带准备：腰/头到 conveyor，手臂到 tcp。 */
int go_belt_ready(Robot_Arm &arm_r, Robot_Arm &arm_l);
/** 传送带放置末端预览：腰/头到 conveyor，手臂到 place_tcp。 */
int go_belt_place(Robot_Arm &arm_r, Robot_Arm &arm_l);
/** 第一传送带抓取手腕预览：腰/头同 belt_ready，手臂用 tcp 的 XYZ + grasp_rpy_deg。不拍照、不夹取、不动底盘。 */
int go_belt_grasp_rpy(Robot_Arm &arm_r, Robot_Arm &arm_l);
/** 第一传送带放置：去 AP6 → 准备 → 认码 → 左右依次放置 → 停准备 tcp。不回 home。 */
int go_belt();
/** 仅放置流程（底盘已到位）。cycle 用。 */
int go_belt_station();
/** 传送带抓取。from_place=true：已在放置后的 tcp，只拍照→抓，不去第一传送带站、不重走准备。 */
int go_belt_grasp(bool from_place = false);
/** 第二传送带：去 out_station（AP5）→ conveyor2 准备姿态。 */
int go_belt2_ready(Robot_Arm &arm_r, Robot_Arm &arm_l);
/** 第二传送带抓取手腕预览：腰/头同 belt2 观察位，手臂用 conveyor2.tcp XYZ + grasp_rpy_deg。不拍照、不夹取、不动底盘。 */
int go_belt2_grasp_rpy(Robot_Arm &arm_r, Robot_Arm &arm_l);
/** 第二传送带放置：去 AP5 → 准备 → 认码放置 → 停准备 tcp。belt2 与 belt2_place 相同。 */
int go_belt2();
int go_belt2_place();
/** 第二传送带：去 AP5 → 准备 → 抓取（不转腰）→ 回 grasp_tcp。 */
int go_belt2_grasp();
/** 料盘 AP7 抓 → home → AP5 放置 → belt_grasp → AP6 放置 → belt2_grasp → AP9 空孔放置 → 回 AP7 再抓，闭环到 abort。 */
int vision_grasp_then_belt();
/** 只在料盘1和一号传送带之间循环：AP7 抓 → home → AP5 放置 → 回 AP7。不抓传送带、不去 AP6、不去 AP9。 */
int vision_grasp_belt1();
/** 二号传送带抓取 → 二号料盘放置，放完停在 AP9，不升腰、不回 AP7。 */
int vision_belt2_then_tray2();
/** 腰关节点动。joint=1..5（脚踝/膝盖/髋/侧倾/回转），dq_rad 限幅 ±0.12。不经笛卡尔 IK。 */
int go_waist_jog(int joint, double dq_rad);
/** 去抓取准备：头标定角、腰同原 ready，手臂 standby XY/Z + ready1/2/3/6 的抓取 RPY。 */
int go_grasp_ready(Robot_Arm &arm_r, Robot_Arm &arm_l, int row_group);
/** 料盘2放置准备：腰/头同 grasp ready，手臂用 tray2_place 独立 RPY。 */
int go_tray2_ready(Robot_Arm &arm_r, Robot_Arm &arm_l, int row_group);
/** AP9 → 下蹲同 grasp → 空孔放置。allow=false 的手不分配、不运动。
 *  return_to_tray 为真时放完升腰并回 AP7（完整 cycle）。为假时停在 AP9，不升腰。 */
int vision_place_tray2(bool allow_right = true, bool allow_left = true, bool return_to_tray = true);
/** 料盘2精度测试：到 AP9，先合爪，不松爪、不起身，先前三排再后三排，直到没有空孔或 abort。 */
int vision_place_tray2_precision();
int vision_grasp_existing_pipeline();
/** 调度对接：先到 AP7 等任务。转运/下料若已在对应传送带且手臂在准备位，不再回 home。下料放到 AP9 后停在原地等下一条任务。 */
int run_dispatch_mode();
int vision_aruco_detect_pipeline();
int vision_tray_holes_pipeline();

} // namespace orch

