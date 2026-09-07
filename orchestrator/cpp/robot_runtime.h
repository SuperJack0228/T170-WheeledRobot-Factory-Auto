#pragma once

/**
 * 编排台硬件动作封装（无底盘）。
 *
 * 假设：
 * - 手臂目标为 [x,y,z,rx,ry,rz]，xyz 米，欧拉角弧度（与 arm_line_move 一致）。
 *   Web JSON 用角度，守护进程边界处 deg→rad。
 * - 底盘 / Lanxin / VMR：禁止包含、禁止链接、禁止调用。
 */

#include "Ti5_Arm.h"
#include "function.h"
#include "gripper_interface.hpp"
#include "waist.h"

#include <atomic>
#include <array>
#include <functional>
#include <string>
#include <vector>

class PosePipeline;

namespace orch
{

struct PoseMrd
{
    double x = 0, y = 0, z = 0;
    double rx = 0, ry = 0, rz = 0; // rad
};

using AbortFn = std::function<bool()>;

struct ArucoHit
{
    std::string slot;
    int id = -1;
    double side_m = 0.0;
    double reproj_px = 0.0;
    double cam_x = 0.0;
    double cam_y = 0.0;
    double cam_z = 0.0;
    double cam_yaw = 0.0;
    double cam_pitch = 0.0;
    double cam_roll = 0.0;
    bool robot_ok = false;
    double robot_x = 0.0;
    double robot_y = 0.0;
    double robot_z = 0.0;
    double robot_rx_deg = 0.0;
    double robot_ry_deg = 0.0;
    double robot_rz_deg = 0.0;
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
    int last_detect_head = 0;
    int last_detect_right = 0;
    int last_detect_left = 0;
    int last_detect_status = 0;
    /** 金帝四类合计：0 毛胚 1 半加工 2 加工 3 料盘孔 */
    int last_detect_cls[4] = {0, 0, 0, 0};
    std::string last_detect_detail;
    int last_aruco_head = 0;
    int last_aruco_right = 0;
    int last_aruco_left = 0;
    int last_aruco_status = 0;
    std::vector<ArucoHit> last_aruco_hits;
    /** 当前哪只手拿着料：抓取成功置位，该侧夹爪松开清掉 */
    bool holding_r = false;
    bool holding_l = false;
    /** 最近一次「持料去传送带」的手臂：'r'/'l'/0。放完张开后双手都空时，取料走另一只 */
    char last_placed_side = 0;
    /** 最近一次走到传送带上方的手臂，供后续手相机只动这一只 */
    bool last_conveyor_r = false;
    bool last_conveyor_l = false;
    int last_hover_marker_id = -1;
    int last_hover_marker_id_b = -1;
    double last_dual_pos_m = -1.0;
    double last_dual_rot_deg = -1.0;
    bool last_dual_corrected = false;
    int last_place_status = 0;
    int last_place_holes = 0;
    bool last_released_r = false;
    bool last_released_l = false;
};

extern HwSession *g_hw;

inline Eigen::Matrix<double, 1, 6> to_row(const PoseMrd &p)
{
    Eigen::Matrix<double, 1, 6> m;
    m << p.x, p.y, p.z, p.rx, p.ry, p.rz;
    return m;
}

int move_arm_line(Robot_Arm &arm, const PoseMrd &goal, double vel_mps);
ArmLineMoveResult move_arms_line(
    Robot_Arm &arm_r,
    const PoseMrd *goal_r,
    Robot_Arm &arm_l,
    const PoseMrd *goal_l,
    double vel_mps);

int gripper_open(gripper::Gripper &g, gripper::Side side);
int gripper_close(gripper::Gripper &g, gripper::Side side);
int waist_rotate_deg(double yaw_deg, double speed_deg_s);
int waist_fold_deg(double pitch_deg, double speed_deg_s);
int waist_delta(WaistRobot &waist, double dx, double dz, double vel_mps, const AbortFn &abort);
int waist_goto(WaistRobot &waist, const PoseMrd &goal, double vel_mps, const AbortFn &abort);
int go_home(Robot_Arm &arm_r, Robot_Arm &arm_l);

/** 复用 move_box::run_grasp_to_standby；0=抓到，1=无目标，2=未抓住，负=失败/中止 */
int vision_grasp_existing_pipeline();

/** 同上，但用工厂单类 best.pt */
int vision_grasp_factory_pipeline();

/** 工厂单类模型三相机检测（不抓取）；0=有目标，1=无目标，负=失败/中止 */
int vision_detect_factory_pipeline();

/** 金帝四类 seg_model：识别毛胚/半加工/加工/料盘孔，不抓取 */
int vision_detect_jindi_pipeline();

/** 金帝四类抓取：只抓 class0 毛胚 */
int vision_grasp_jindi_pipeline();

/** online_pose 单码 ArUco 三相机检测；0=有码，1=无码，负=失败/中止 */
int vision_aruco_detect_pipeline();

/** 码坐标系下的相对路径点（xyz 米，rpy 弧度）。运行时 T_tcp = T_marker * T_rel */
struct RelWaypoint
{
    std::string name;
    PoseMrd rel;
};

/**
 * 传送带/码旁悬停附加参数。
 * waypoints 相对主码；双码误差低于阈值则不修正主码位姿。
 */
struct ArucoHoverExtras
{
    std::vector<RelWaypoint> waypoints;
    bool include_target = true;
    int marker_id = 0;
    int marker_id_b = 0;
    bool dual_rel_valid = false;
    PoseMrd dual_rel{};
    double dual_pos_thresh_m = 0.015;
    double dual_rot_thresh_deg = 3.0;
};

struct PathRecordResult
{
    int status = 0;
    PoseMrd tcp{};
    PoseMrd rel{};
    PoseMrd marker{};
    PoseMrd marker_b{};
    PoseMrd dual_rel{};
    int marker_id = -1;
    int marker_id_b = -1;
    bool have_b = false;
};

/** arm_force: 0=自动(持料/空手)，'r'/'l'=指定臂（单步测试用，不看持料）。0=到位，1=无码，负=失败 */
int conveyor_goto_pipeline(
    bool is_right, bool want_holding, char arm_force = 0, const ArucoHoverExtras *extras = nullptr);

/** 码正上方。arm_force 同上 */
int aruco_above_pipeline(bool want_holding, char arm_force = 0, const ArucoHoverExtras *extras = nullptr);

/** 头相机拍码，把当前 TCP 记成主码坐标系下的相对位姿（示教）。arm_side 'r'/'l' */
int path_record_pipeline(char arm_side, int marker_id, int marker_id_b, PathRecordResult &out);

/** 仅手相机识别+抓取+抬起（不回头相机、不回待命）。0=抓到，1=无目标，2=未抓住，负=失败 */
int vision_hand_grasp_pipeline();

/** 同上，工厂单类 best.pt（小毛胚） */
int vision_hand_grasp_factory_pipeline();

/** 头相机 class1 空位，持料手放回料盘。0=已放，1=无空位/无持料，2=运动失败，负=失败/中止 */
int vision_place_pipeline();

} // namespace orch
