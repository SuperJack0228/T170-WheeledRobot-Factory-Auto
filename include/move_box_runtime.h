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
    /** true：XY 来自 ArUco 孔心，手相机不得改写 */
    bool xy_from_tray = false;
    /** 离机器人远近行号：1=最远，6=最近（料盘转 180° 时与 ArUco row 对调）。0=未知 */
    int row_r = 0;
    int row_l = 0;
    /** 料盘 ArUco 行号：+X 前=1，-X 后=6 */
    int row_aruco_r = 0;
    int row_aruco_l = 0;
    /** 左右手列号（盘转 180° 已对调）：1–3 右、4–6 左 */
    int col_r = 0;
    int col_l = 0;
    /** 料盘 ArUco 原始列号：-Y 右=1，+Y 左=6 */
    int col_aruco_r = 0;
    int col_aruco_l = 0;
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
    bool have_object_z_r = false;
    bool have_object_z_l = false;
    double object_z_r = 0.0;
    double object_z_l = 0.0;
};

Eigen::Matrix<double, 1, 6> make_standby_pos_right();
Eigen::Matrix<double, 1, 6> make_standby_pos_left();
Eigen::Matrix<double, 1, 6> make_home_tcp_right();
Eigen::Matrix<double, 1, 6> make_home_tcp_left();

void log_arm_skip_right();
void log_arm_skip_left();

/** 本轮抓取是否启用 TCP→目标自适应 pitch/yaw（run_grasp_to_standby 置位） */
extern bool g_grasp_adaptive_rpy;

void grasp_snapshot_from_state(
    const HeadAssignState &st,
    bool grasped_r,
    bool grasped_l,
    GraspedGoalSnapshot &snap);

void apply_head_assign_keep_grasped(HeadAssignState &st, const GraspedGoalSnapshot &snap);
bool head_grasp_simultaneous(const HeadAssignState &st);
/** 双手列号差小于 min_simultaneous_col_delta 时不可同时抓 */
bool dual_grasp_targets_too_close(const HeadAssignState &st);
/** 安全抬离料盘 → 本侧分区内退到 yaml standby（抓取开始前准备姿态）。失败返回 false。
 *  row_pose_group 1/2/3/6：XYZ 仍用 standby，RPY 用对应 ready 姿态；其它值用 standby 原姿态。 */
/** tray2_place_rpy=true 时用 tray2_place 的四组姿态，不读 grasp ready1236。 */
bool move_arms_to_standby(
    Robot_Arm &arm_r,
    Robot_Arm &arm_l,
    int row_pose_group = -1,
    bool tray2_place_rpy = false);

enum class TrayHoleTask
{
    GraspParts,
    PlaceEmpty
};

void set_tray_hole_task(TrayHoleTask task);
TrayHoleTask tray_hole_task();

/** 当前任务类别在工作区内还能分给左右手的孔数。抓取数毛坯，放置数空孔。 */
struct TrayZoneCount
{
    bool ok = false;
    int right = 0;
    int left = 0;
    std::string message;
};

TrayZoneCount count_tray_zone_holes(
    RealSenseMultiCam &cameras,
    SegPoseBridge &bridge,
    const std::array<double, 16> &cam2robot);
/** 料盘2精度测试：先前三排（from_robot 1–3）再后三排，到位不松爪，不回 home。 */
void set_tray2_precision_test(bool on);
bool tray2_precision_test();
/** home 指令：安全抬离后走到 yaml home_tcp（默认 TCP 原点、RPY=0）。不走抓取 standby，也不用料盘左右分区卡 Y。 */
bool move_arms_to_home(Robot_Arm &arm_r, Robot_Arm &arm_l);
/** belt_ready：绝对走到该套 conveyor.tcp（准备姿态）。 */
bool move_arms_to_belt_ready(
    Robot_Arm &arm_r, Robot_Arm &arm_l, const ConveyorStationConfig &c);
bool move_arms_to_belt_ready(Robot_Arm &arm_r, Robot_Arm &arm_l);
/** tcp 的 XYZ + grasp_rpy_deg。只摆手腕，不拍照、不夹取。 */
bool move_arms_to_belt_grasp_rpy(
    Robot_Arm &arm_r, Robot_Arm &arm_l, const ConveyorStationConfig &c);
/** belt_place：绝对走到该套 conveyor.place_tcp（放置末端预览）。 */
bool move_arms_to_belt_place(
    Robot_Arm &arm_r, Robot_Arm &arm_l, const ConveyorStationConfig &c);
bool move_arms_to_belt_place(Robot_Arm &arm_r, Robot_Arm &arm_l);
/** 完整放置：RPY 用 place_tcp，XYZ = 码原点 + R_识别 × offset（识别系 +X前 +Y左 +Z上）。 */
Eigen::Matrix<double, 1, 6> make_belt_goal_right(
    const BeltDetectResult &det, const ConveyorStationConfig &c);
Eigen::Matrix<double, 1, 6> make_belt_goal_left(
    const BeltDetectResult &det, const ConveyorStationConfig &c);
Eigen::Matrix<double, 1, 6> make_belt_goal_right(
    double origin_x, double origin_y, double origin_z, const ConveyorStationConfig &c);
Eigen::Matrix<double, 1, 6> make_belt_goal_left(
    double origin_x, double origin_y, double origin_z, const ConveyorStationConfig &c);
Eigen::Matrix<double, 1, 6> make_belt_goal_right(double origin_x, double origin_y, double origin_z);
Eigen::Matrix<double, 1, 6> make_belt_goal_left(double origin_x, double origin_y, double origin_z);
/** 传送带抓取：该手 XYZ（识别里按该手 offset 算好），RPY 用 grasp_rpy。 */
Eigen::Matrix<double, 1, 6> make_belt_grasp_goal(
    bool is_right, double grasp_x, double grasp_y, double grasp_z, const ConveyorStationConfig &c);
Eigen::Matrix<double, 1, 6> make_belt_grasp_goal(
    bool is_right, double grasp_x, double grasp_y, double grasp_z);
/** 单臂笛卡尔直线到目标。belt 放置：先右手再左手，不走 home 退回逻辑。 */
bool move_one_arm_line_to(
    Robot_Arm &arm,
    bool is_right,
    Eigen::Matrix<double, 1, 6> goal,
    const char *tag,
    bool hold_j2 = false);
/** 夹后回程：一次 TCP 直线到 yaml standby（XYZ+姿态）。不分段抬/平移/落Z/转腕。右臂锁定则只动左。 */
bool move_arms_tcp_to_standby(Robot_Arm &arm_r, Robot_Arm &arm_l);
/** 夹后回程：抬升后一次 TCP 直线到 yaml home_tcp。不分段。 */
bool move_arms_tcp_to_home(Robot_Arm &arm_r, Robot_Arm &arm_l);
/** 仅 grasp 指令入口：保持当前 XYZ，原地转到 yaml standby 末端姿态。 */
bool rotate_arms_in_place_to_standby_rpy(Robot_Arm &arm_r, Robot_Arm &arm_l);
void refresh_head_move_flags(HeadAssignState &st);

/** 当前腰高下的盘面参考 Z（基座系，向上为正） */
double tray_z_ref_now();
double expected_object_top_z();

/** layer3 x/z 夹到 yaml waist 行程；有改动返回 true */
bool clamp_waist_layer3_xyz(Eigen::Matrix<double, 1, 6> &pos);

/**
 * 用腰部编码器实时位姿覆盖 layer3 软件状态。
 * layer3 是绝对笛卡尔位姿，只要软件值和实物不一致，"降腰 5 cm" 就会变成移动到一个
 * 与预期无关的绝对高度（曾把躯干抬高 13.5 cm）。成功返回 true 并置位"已同步"。
 */
bool sync_waist_layer3_from_encoders(
    WaistRobot &waist,
    Eigen::Matrix<double, 1, 6> &layer3,
    const char *tag);

/** layer3 是否可信；未同步时禁止一切基于 layer3 的腰部笛卡尔运动 */
bool waist_layer3_is_synced();
void waist_layer3_set_synced(bool synced);

/** 腰只动 X（前后平移），Y/Z/姿态锁当前值。成功返回 true */
bool waist_layer3_move_x_only(
    WaistRobot &waist,
    Eigen::Matrix<double, 1, 6> &layer3,
    double x_target,
    bool settle);

/** 腰只动 Z（竖直升/降），X/Y 锁当前值。成功返回 true */
bool waist_layer3_move_z_only(
    WaistRobot &waist,
    Eigen::Matrix<double, 1, 6> &layer3,
    double z_target,
    bool settle);

/** 腰下降 drop_m（layer3 z 减小），受 z_min 限制。一次直线到位。返回实际下降量，0=没动 */
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
    SegEngineId engine_id = SegEngineId::Default,
    /** 已持件/锁定的一侧不分配新目标，只给空爪补抓 */
    bool skip_right = false,
    bool skip_left = false);

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
    bool hover_z_from_hand = true);

ArmLineMoveResult run_hand_approach_and_grasp(
    gripper::Gripper &g,
    Robot_Arm &arm_r,
    Robot_Arm &arm_l,
    HeadAssignState &st,
    HandMoveState &hs);

void lift_grasped_selective(
    Robot_Arm &arm_r,
    Robot_Arm &arm_l,
    RealSenseMultiCam &cameras,
    HeadAssignState &st,
    bool move_r,
    bool move_l);

bool restore_waist_layer3_x(WaistRobot &waist, Eigen::Matrix<double, 1, 6> &posup_down_3_layer);
/** 腰向前伸到 home_x + far_row_forward_m（远三排）；已到位则跳过。返回实际前进量（>0）。抓完不收回。 */
double waist_layer3_ensure_far_row_forward(
    WaistRobot &waist,
    Eigen::Matrix<double, 1, 6> &layer3);
void apply_grasp_stagger_waist_comp(HeadAssignState &st, double waist_actual_delta);
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

/** 腰先按编码器同步，再平移回 layer3_home.x。已到位则跳过。失败返回 false。 */
bool ensure_waist_layer3_x_home(WaistRobot &waist, Eigen::Matrix<double, 1, 6> &posup_down_3_layer);
/** ready/grasp 起始：X 到 home_x+ready_forward_m，Z 到 ready_z_m。编码器复核不到位返回 false。 */
bool ensure_waist_ready_start(WaistRobot &waist, Eigen::Matrix<double, 1, 6> &posup_down_3_layer);

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

/**
 * 一次视觉抓取：头相机识别 → TCP 直线 A→B 到上方同时转该行姿态 → TCP 直线 B→C 下压夹取 →
 * 夹后 TCP 抬升，再一次 TCP 直线回 yaml standby（XYZ+姿态）。grasp 入口另做一次分段安全回 standby。
 * adaptive_approach_rpy=true：用当前 TCP→目标水平朝向定 yaw，再下斜 adaptive_pitch_down_deg（默认 45°）。
 * holding_r/l：该侧已持件，本轮不松爪、不分配、不运动该臂，只给空爪补抓。
 */
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
    SegEngineId engine_id = SegEngineId::Default,
    bool adaptive_approach_rpy = false,
    bool holding_r = false,
    bool holding_l = false);

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

} // namespace move_box
