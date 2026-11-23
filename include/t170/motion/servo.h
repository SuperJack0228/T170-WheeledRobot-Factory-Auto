#pragma once
#include "t170/motion/differential_ik.h"
#include "t170/vision/servo_observer_port.h"
#include "t170/control/visual_servo.h"
class Arm;

namespace wrist_servo {

// Reset only selected hands: the peer may still be holding/retracting a part.
void prepare_group(bool move_r = true, bool move_l = true);
bool group_is_failed();
std::string last_failure(int hand);
int hand_index(const Arm &arm);
Eigen::Matrix4d tool_transform(const Arm &arm);
void send_joint_reference(int hand, const Joints &q);
bool read_joint_feedback(int hand, Joints &q);
bool begin_session(Arm &arm, const Eigen::Matrix<double,1,6> &goal, int target_class,
                   const char *task_tag = "descent");
void sample_approach_state(Arm &arm);
void end_session(int hand, bool success);

void stage_tray_target(int hand,const TrayTarget &target);
// Explicit tray-grasp scope; NaN clears it. Consumed by the next session.
void stage_tray_grasp(int hand,double feature_plane_z);
// Belt grasp may reuse the tray-grasp reference pixel and its taught height.
// The tray XY compensation stays on the tray goal. Consumed by the next session.
void stage_belt_grasp_reference(int hand,bool enable);
// Measured state interpolated at the frame timestamp; never command extrapolation.

int run_descent(Arm &arm, const Eigen::Matrix<double,1,6> &goal,
                const Joints &last_command, const Joints &previous_command, double descent_speed);
bool completed_xy(int hand, double &x, double &y);
void invalidate_result(int hand);
bool ready_for_gripper(Arm &arm);
// Re-send the joint reference captured when descent finished. No-op until then.
void repeat_final_hold(int hand);
// Keep sending that reference until a later trajectory takes it.
void start_final_hold_repeat();
// Copy that reference without stopping the repeat.
bool peek_final_hold(int hand, Joints &q);
// Stop the repeat, wait for any in-flight seven-joint hold packet to finish,
// then return the joints playback must start from. No old hold send can follow.
bool take_final_hold(int hand, Joints &q);
// Install a stopped retreat endpoint, using the same exclusive ownership as descent.
void hold_joint_reference(int hand, const Joints &q);
void task_event(int hand, const char *event, const char *reason);
void handoff_event(Arm &arm, const Joints &command, const Joints &previous, const char *reason);
void record_gripper_check(int hand, double position_rad, double torque_nm, const char *outcome);

// Implemented by the motion kinematics adapter beside the shared SDK mutex; no motor commands are issued here.
bool solve_local_pose(Arm &arm, const Joints &seed, const Vec3 &position,
                      const Eigen::Matrix3d &rotation, const Config &config,
                      Joints &solution, double &xyz_residual, double &rotation_residual);
bool solve_dynamic_pose(Arm &arm,const Joints &seed,const Vec3 &position,
    const Eigen::Matrix3d &rotation,const Config &config,const Joints &previous_velocity,
    double dt,double floor_z,Joints &next,t170::motion::DifferentialReport &report);

}
