#pragma once
#include <unordered_set>
#include "t170/skills/legacy/runtime.h"
#include "t170/common/units.h"
namespace move_box {
inline constexpr double kWaistPitchLinkM = 0.110;
extern bool g_grasp_adaptive_rpy;
extern bool g_tray1_far_band_memory;
extern double g_current_waist_layer3_z;
extern double g_waist_pitch_rad;
extern double g_waist_roll_rad;
extern double g_waist_yaw_rad;
extern TrayHoleTask g_tray_hole_task;
extern int g_assign_class_override;
extern int g_assign_fixed_from_robot_row;
extern bool g_waist_layer3_synced;
Eigen::Matrix3d rot_x(double a);
Eigen::Matrix3d rot_y(double a);
Eigen::Matrix3d rot_z(double a);
void remember_waist_attitude(const Eigen::MatrixXd &qj);
Eigen::Matrix<double, 1, 6> arm_goal_in_real_waist_frame(const Eigen::Matrix<double, 1, 6> &upright);
int tray_assign_class_id();
bool tray2_place_active();
double active_goal_x_offset(bool is_right);
double active_goal_y_offset(bool is_right);
bool goal_pos_valid_right(double x, double y, double z);
bool goal_pos_valid_left(double x, double y, double z);
bool grasp_goal_side_blocked(bool is_right, double y, bool from_tray);
void drop_if_y_split_blocks(HeadAssignState &st, bool &move_r, bool &move_l, const char *stage);
void drop_if_y_split_blocks(HeadAssignState &st, HandMoveState &hs, const char *stage);
bool head_hand_detected(const double hand_pos[3]);
void fill_goal_last_from_head_assign(
        const double right_hand_pos[3],
        const double left_hand_pos[3],
        int row_r,
        int row_l,
        Eigen::Matrix<double, 1, 6> &goal_last_r,
        Eigen::Matrix<double, 1, 6> &goal_last_l);
void apply_adaptive_grasp_rpy_selective(
        Arm &arm_r,
        Arm &arm_l,
        HeadAssignState &st,
        bool move_r,
        bool move_l);
bool head_move_allowed_right(
        const double right_hand_pos[3],
        const Eigen::Matrix<double, 1, 6> &goal_last_r);
bool head_move_allowed_left(
        const double left_hand_pos[3],
        const Eigen::Matrix<double, 1, 6> &goal_last_l);
bool apply_final_grasp_z_from_hand(
        const char *hand,
        bool is_right,
        int from_robot_row,
        bool have_object_z,
        double object_z,
        Eigen::Matrix<double, 1, 6> &goal_last);
void log_arm_traj_plan_fail(const char *hand, int ret);
}
