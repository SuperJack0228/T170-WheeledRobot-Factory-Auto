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
/** 第一传送带单独放置测试：不导航。先转到 belt1_waist 的腰角度，再拍照放置，结束后底盘原地不动。 */
int go_belt();
/** 仅放置流程（底盘已到位）。cycle 用。 */
int go_belt_station();
/** 第一传送带单独抓取测试：不导航、不再摆腰。人已在观察位时原地拍照，再按准备 tcp 抓取。
 *  from_place=true：已在放置后的 tcp，只拍照→抓，连准备 tcp 也不再走。 */
int go_belt_grasp(bool from_place = false);
/** 第二传送带准备：不导航。腰/头到 conveyor2，手臂到 tcp。 */
int go_belt2_ready(Robot_Arm &arm_r, Robot_Arm &arm_l);
/** 第二传送带抓取手腕预览：腰/头同 belt2 观察位，手臂用 conveyor2.tcp XYZ + grasp_rpy_deg。不拍照、不夹取、不动底盘。 */
int go_belt2_grasp_rpy(Robot_Arm &arm_r, Robot_Arm &arm_l);
/** 第二传送带放置：不导航。先转到 conveyor2 的腰角度，再拍照放置。belt2 与 belt2_place 相同。 */
int go_belt2();
int go_belt2_place();
/** 第二传送带单独抓取：不导航、不再摆腰。人已在观察位时原地拍照，再按 conveyor2 准备 tcp 抓取。 */
int go_belt2_grasp();
/** 演示整圈，不和调度通讯。AP6：料盘抓 → 传送带一放 → 传送带一抓。
 *  再到 AP7：传送带二放 → 传送带二抓（手回 conveyor2.tcp）→ 起身并转到 180° → 保持该朝向降腰，料盘二放。然后下一圈。abort 才停。 */
int vision_grasp_then_belt();
/** cycle1：同一站点（默认 AP6）上料盘抓取和一号传送带放置循环。底盘只在防碰姿态之后导航一次。 */
int vision_grasp_belt1();
/** 二号传送带抓取 → 二号料盘放置。两段都在 out_station / tray2_station（现为 AP7），放完不升腰。 */
int vision_belt2_then_tray2();
/** 腰关节点动。joint=1..5（脚踝/膝盖/髋/侧倾/回转），dq_rad 限幅 ±0.12。不经笛卡尔 IK。 */
int go_waist_jog(int joint, double dq_rad);
/** 传送带一同一点调试：先到 conveyor 拍照 x/z，头到 conveyor 头角，再 LowerBody 联动 yaw/pitch，最后双手到 conveyor.tcp。没到位不算成功。不动底盘。 */
int go_belt1_same_spot_waist();
/** 去抓取准备：头标定角、腰同原 ready，手臂 standby XY/Z + ready1/2/3/6 的抓取 RPY。 */
int go_grasp_ready(Robot_Arm &arm_r, Robot_Arm &arm_l, int row_group);
/** 料盘2放置准备：腰/头同 grasp ready，手臂用 tray2_place 独立 RPY。 */
int go_tray2_ready(Robot_Arm &arm_r, Robot_Arm &arm_l, int row_group);
/** 去料盘二放置点，下蹲后认空孔放置。allow=false 的手不分配、不运动。
 *  return_to_tray 为真时放完升腰并回 tray_station。为假时停在原地，不升腰。
 *  face_yaw 为真时降腰保持 face_yaw_deg，不把回转拧回 0。
 *  station 为空时用 yaml 的 tray2_station；调度传入时用调度下发的点。 */
int vision_place_tray2(bool allow_right = true, bool allow_left = true, bool return_to_tray = true,
                       bool face_yaw = false, double face_yaw_deg = 0.0,
                       const std::string &station = {});
/** 料盘2精度测试：到 AP9，先合爪，不松爪、不起身，先前三排再后三排，直到没有空孔或 abort。 */
int vision_place_tray2_precision();
int vision_grasp_existing_pipeline();
/** 调度对接。三个子任务的底盘点由调度下发。顺着上料→转运、转运→下料时原地接着做；否则收成开车姿态，回上料抓取点待命。 */
int run_dispatch_mode();
/** 程序启动后后台以 2Hz 把底盘电量和位姿写入调度客户端的上报变量。 */
void start_scheduler_status_writer();
void stop_scheduler_status_writer();
int vision_aruco_detect_pipeline();
int vision_tray_holes_pipeline();

} // namespace orch

