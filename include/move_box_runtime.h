#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "function.h"
#include "gripper_interface.hpp"
#include "move_box_config.h"
#include "realsense_get.h"
#include "seg_pose_bridge.h"
#include "Ti5_Arm.h"
#include "waist.h"
#ifndef ORCH_NO_CHASSIS
#include "lanxincontrol.h"
#endif

// true=OpenCV 可视化调试；false=仅终端打印
constexpr bool kDebugVisualize = true;

namespace move_box
{

struct HeadAssignState
{
    double right_hand_pos[3] = {-1.0, 0.0, 0.0};
    double left_hand_pos[3] = {-1.0, 0.0, 0.0};
    Eigen::Matrix<double, 1, 6> goal_last_r;
    Eigen::Matrix<double, 1, 6> goal_last_l;
    bool have_r = false;
    bool have_l = false;
    bool move_r = false;
    bool move_l = false;
    HeadHandAssignZones zones;
};

/** 本轮抓取腰进已用步数（与 stagger_max_steps 合计，双手共享；头相机不重拍，只补偿 goal x） */
struct GraspStaggerWaistState
{
    int steps_used = 0;
};

struct GraspedGoalSnapshot
{
    bool grasped_r = false;
    bool grasped_l = false;
    Eigen::Matrix<double, 1, 6> goal_last_r;
    Eigen::Matrix<double, 1, 6> goal_last_l;
};

struct HandMoveState
{
    bool move_r = false;
    bool move_l = false;
    bool steady_timed_out = false;
};

void aruco_test_exit(int code);

Eigen::Matrix<double, 1, 6> make_standby_pos_right();
Eigen::Matrix<double, 1, 6> make_standby_pos_left();

void log_arm_skip_right();
void log_arm_skip_left();

void grasp_snapshot_from_state(
    const HeadAssignState &st,
    bool grasped_r,
    bool grasped_l,
    GraspedGoalSnapshot &snap);

void apply_head_assign_keep_grasped(HeadAssignState &st, const GraspedGoalSnapshot &snap);
bool head_grasp_simultaneous(const HeadAssignState &st);
void move_arms_to_standby(Robot_Arm &arm_r, Robot_Arm &arm_l);
void refresh_head_move_flags(HeadAssignState &st);

/** layer3 x/z 夹到 yaml waist 行程；有改动返回 true */
bool clamp_waist_layer3_xyz(Eigen::Matrix<double, 1, 6> &pos);

/** 腰下降 drop_m（layer3 z 减小），受 z_min 限制。返回实际下降量，0=没动 */
double waist_layer3_lower_z(
    WaistRobot &waist,
    Eigen::Matrix<double, 1, 6> &layer3,
    double drop_m);

HeadAssignState detect_and_assign_head(
    RealSenseMultiCam &cameras,
    SegPoseBridge &bridge,
    const std::array<double, 16> &cam2robot,
    /** true：至少一侧未分配到位时重拍一次；双手都已分配到则不再重拍 */
    bool retry_if_incomplete,
    SegEngineId engine_id = SegEngineId::Default);

HeadAssignState detect_and_assign_head_keep_grasped(
    RealSenseMultiCam &cameras,
    SegPoseBridge &bridge,
    const std::array<double, 16> &cam2robot,
    bool retry_if_incomplete,
    const GraspedGoalSnapshot *keep_grasped,
    SegEngineId engine_id = SegEngineId::Default);

ArmLineMoveResult run_head_approach_selective(
    Robot_Arm &arm_r,
    Robot_Arm &arm_l,
    HeadAssignState &st,
    bool move_r,
    bool move_l);

bool on_hand_steady_timeout_abort(
    const HandMoveState &hs,
    bool enable_r,
    bool enable_l,
    Robot_Arm &arm_r,
    Robot_Arm &arm_l);

HandMoveState run_hand_detect_and_validate(
    Robot_Arm &arm_r,
    Robot_Arm &arm_l,
    RealSenseMultiCam &cameras,
    SegPoseBridge &bridge,
    HeadAssignState &st,
    bool enable_r,
    bool enable_l,
    std::string &err,
    SegEngineId engine_id = SegEngineId::Default,
    bool hover_z_from_hand = false);

ArmLineMoveResult run_hand_approach_and_grasp(
    gripper::Gripper &g,
    Robot_Arm &arm_r,
    Robot_Arm &arm_l,
    HeadAssignState &st,
    const HandMoveState &hs);

void lift_grasped_selective(
    Robot_Arm &arm_r,
    Robot_Arm &arm_l,
    RealSenseMultiCam &cameras,
    HeadAssignState &st,
    bool move_r,
    bool move_l);

void restore_waist_layer3_x(WaistRobot &waist, Eigen::Matrix<double, 1, 6> &posup_down_3_layer);
/** 抓取腰进 ×mult，返回实际 waist x delta(>0 表示前进) */
double waist_layer3_stagger_forward(
    WaistRobot &waist,
    Eigen::Matrix<double, 1, 6> &posup_down_3_layer,
    int steps,
    const char *log_tag);

/** 为 stagger 侧一次性腰进(×1/×2/×3)；不重拍头相机；整轮步数上限 stagger_max_steps */
bool advance_waist_stagger_side(
    WaistRobot &waist,
    Eigen::Matrix<double, 1, 6> &posup_down_3_layer,
    bool for_right_hand,
    HeadAssignState &st,
    GraspStaggerWaistState &stagger_waist);

/** 双手 x 均过远：按最远 x 一次性腰进，预算同上 */
bool advance_waist_stagger_dual_hands(
    WaistRobot &waist,
    Eigen::Matrix<double, 1, 6> &posup_down_3_layer,
    HeadAssignState &st,
    GraspStaggerWaistState &stagger_waist);

void ensure_waist_layer3_x_home(WaistRobot &waist, Eigen::Matrix<double, 1, 6> &posup_down_3_layer);

enum class GraspToStandbyStatus
{
    Ok = 0,
    NoTarget = 1,
    Aborted = 2,
    GraspNone = 3,
    CamFail = 4,
};

struct GraspToStandbyResult
{
    GraspToStandbyStatus status = GraspToStandbyStatus::NoTarget;
    bool grasped_r = false;
    bool grasped_l = false;
    bool waist_adjusted = false;
    HeadAssignState st;
};

/** 一次视觉抓取：头相机识别 → 到物体上方 → 手相机改 xy → 下压夹取 → 抬起 → 回待命。不含底盘、不含放货。 */
GraspToStandbyResult run_grasp_to_standby(
    gripper::Gripper &g,
    Robot_Arm &arm_r,
    Robot_Arm &arm_l,
    WaistRobot &waist,
    Eigen::Matrix<double, 1, 6> &posup_down_3_layer,
    RealSenseMultiCam &cameras,
    SegPoseBridge &bridge,
    const std::array<double, 16> &cam2robot,
    std::string &err,
    const std::function<bool()> &should_abort = nullptr,
    SegEngineId engine_id = SegEngineId::Default);

/**
 * 仅手相机识别 + 下压夹取 + 抬起，不回头相机、不回待命。
 * 用当前 TCP 作为 goal_last（xy 由手相机更新，z/姿态保持悬停位）。
 */
GraspToStandbyResult run_hand_grasp_only(
    gripper::Gripper &g,
    Robot_Arm &arm_r,
    Robot_Arm &arm_l,
    RealSenseMultiCam &cameras,
    SegPoseBridge &bridge,
    std::string &err,
    const std::function<bool()> &should_abort = nullptr,
    SegEngineId engine_id = SegEngineId::Default,
    bool enable_r = true,
    bool enable_l = true);

/** 头相机 class1 空位 → 持料手放到料盘（无底盘、无 48 格、无第6排弯腰） */
enum class PlaceToTrayStatus
{
    Ok = 0,
    NoTarget = 1,
    Aborted = 2,
    MoveFail = 3,
};

struct PlaceToTrayResult
{
    PlaceToTrayStatus status = PlaceToTrayStatus::NoTarget;
    bool released_r = false;
    bool released_l = false;
    int holes_found = 0;
};

PlaceToTrayResult run_place_holding_to_tray(
    gripper::Gripper &g,
    Robot_Arm &arm_r,
    Robot_Arm &arm_l,
    RealSenseMultiCam &cameras,
    SegPoseBridge &bridge,
    const std::array<double, 16> &cam2robot,
    bool holding_r,
    bool holding_l,
    std::string &err,
    const std::function<bool()> &should_abort = nullptr);

#ifndef ORCH_NO_CHASSIS
bool prepare_chassis_for_grasp(
    LanxinControl &ctrl,
    WaistRobot &waist,
    Eigen::Matrix<double, 1, 6> &posup_down_3_layer,
    bool &need_topo_fang1);

bool ensure_charged_and_at_work_site(LanxinControl &ctrl, bool &need_topo_fang1);

bool run_post_grasp_place_flow(
    gripper::Gripper &g,
    Robot_Arm &arm_r,
    Robot_Arm &arm_l,
    HeadAssignState &st,
    bool grasped_r,
    bool grasped_l,
    WaistRobot &waist,
    Eigen::Matrix<double, 1, 6> &posup_down_3_layer,
    LanxinControl &ctrl,
    RealSenseMultiCam &cameras,
    SegPoseBridge &bridge,
    const std::array<double, 16> &cam2robot,
    bool &waist_adjusted);
#endif

void lift_grasped_after_pick(
    Robot_Arm &arm_r,
    Robot_Arm &arm_l,
    RealSenseMultiCam &cameras,
    HeadAssignState &st,
    bool grasped_r,
    bool grasped_l,
    bool lifted_r,
    bool lifted_l);

bool head_x_needs_stagger(double x);
/** 按超出 threshold 的量算本次应腰进几步(1/2/3)，不超过 steps_remaining */
int stagger_waist_steps_for_excess(double grasp_x, int steps_remaining);

void move_waist_to_place_ready_x(WaistRobot &waist, Eigen::Matrix<double, 1, 6> &posup_down_3_layer);
double configured_row_waist_x(int row_0);
double waist_layer3_apply_place_x_delta(
    WaistRobot &waist,
    Eigen::Matrix<double, 1, 6> &posup_down_3_layer,
    double delta_x,
    const char *log_tag);
double waist_x_arm_goal_comp(double waist_delta_x);

} // namespace move_box

namespace row6_experiment
{

constexpr double kRow6WaistBendDeg = 20.0;
constexpr double kRow6ShoulderLiftDeg = 36.0;
constexpr double kRow6BendSpeedDegPerS = 30.0;

void row6_wait_stage(const char *msg);

Eigen::Matrix<double, 1, 6> compensate_tcp_for_waist_pitch_forward(
    const Eigen::Matrix<double, 1, 6> &goal_upright,
    double pitch_deg);

bool perform_row6_waist_bend_with_shoulder_lift(
    double waist_bend_deg,
    double shoulder_delta_deg = kRow6ShoulderLiftDeg,
    double speed_deg_per_s = kRow6BendSpeedDegPerS,
    int dt_ms = 20);

/** 第六排完整放货实验：夹持→腰进→弯腰→放货 z 段→松爪→回正/回 home/待机 */
bool run_row6_place_experiment(
    gripper::Gripper &g,
    Robot_Arm &arm_r,
    Robot_Arm &arm_l,
    WaistRobot &waist,
    Eigen::Matrix<double, 1, 6> &pos_layer3);

} // namespace row6_experiment
