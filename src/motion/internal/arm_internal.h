#pragma once
// Private bridges shared by motion implementations. Not a task/hardware API.
#include "t170/common/pose_math.h"
#include "t170/motion/cartesian_tracking.h"
#include "t170/motion/feedback.h"
#include "t170/motion/kinematics_adapter.h"
#include "t170/motion/planning.h"
#include "t170/motion/retreat.h"
#include "t170/motion/servo_handoff.h"
#include "t170/motion/trajectory_executor.h"
#include "t170/motion/arm.h"
#include "t170/motion/servo.h"
#include "t170/motion/trajectory.h"
#include "t170/hardware/device_bus.h"
#include <mutex>
#include <atomic>
#include <functional>
#include <thread>
extern int g_arm_last_cmd_cnt[2][7];
extern bool g_arm_last_cmd_valid[2];
extern const char *g_arm_line_move_debug_stage;
extern std::mutex g_arm_line_move_stage_mu;
extern std::mutex g_arm_kin_mu[2];
extern int g_planned_descent_row[2];
extern wrist_servo::Joints g_servo_joint_origin[2];
namespace t170::motion::detail {
inline constexpr double k_arm_traj_dt = 5e-3;
inline constexpr int kMotorPositionReachTolerance = 550;
int arm_hand_side(const Arm &arm);
Eigen::MatrixXd arm_read_model_joints(int hand);
double encoder_cnt_to_rad(int cnt);
int execute_arm_trajectory(const Eigen::MatrixXd &traj_in, int hand, double dt = k_arm_traj_dt,
                           Arm *retract_arm = nullptr,
                           const std::function<void(const wrist_servo::Joints &)> &observe_progress = {},
                           const std::function<void()> &on_notify = {},
                           int notify_row = -1);
std::function<bool(const Eigen::RowVectorXd &)> lift_connector_validity(
    Arm &arm, const Eigen::RowVectorXd &held);
int plan_arm_line_from(
    Arm &arm,
    Eigen::Matrix<double, 1, 6> goal,
    double vel_m_s,
    const Eigen::MatrixXd &q_start,
    Eigen::MatrixXd &traj,
    bool protect_vertical_start = false);
int plan_line_then_line(
    Arm &arm,
    Eigen::Matrix<double, 1, 6> mid,
    Eigen::Matrix<double, 1, 6> goal,
    double vel_first_m_s,
    double vel_second_m_s,
    Eigen::MatrixXd &traj_out,
    const char *tag,
    const Eigen::MatrixXd *q_override = nullptr,
    bool lift_then_retract = false);
int rad_to_encoder_cnt(double q_rad);
wrist_servo::ConnectorLimits retract_connector_limits();
bool take_prepared_line(int hand, Eigen::MatrixXd &traj, int &ret);
}
