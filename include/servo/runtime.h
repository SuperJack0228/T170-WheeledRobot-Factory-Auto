#pragma once
#include "t170/vision/servo_observer_port.h"
#include "t170/control/visual_servo.h"
class Robot_Arm;

namespace wrist_servo {

// Reset only selected hands: the peer may still be holding/retracting a part.
void prepare_group(bool move_r = true, bool move_l = true);
bool group_is_failed();
std::string last_failure(int hand);
int hand_index(const Robot_Arm &arm);
Eigen::Matrix4d tool_transform(const Robot_Arm &arm);
void send_joint_reference(int hand, const Joints &q);
bool read_joint_feedback(int hand, Joints &q);
bool begin_session(Robot_Arm &arm, const Eigen::Matrix<double,1,6> &goal, int target_class,
                   const char *task_tag = "descent");
void sample_approach_state(Robot_Arm &arm);
void end_session(int hand, bool success);

void stage_tray_target(int hand,const TrayTarget &target);
// Measured state interpolated at the frame timestamp; never command extrapolation.

int run_descent(Robot_Arm &arm, const Eigen::Matrix<double,1,6> &goal,
                const Joints &last_command, const Joints &previous_command, double descent_speed);
bool completed_xy(int hand, double &x, double &y);
void invalidate_result(int hand);
bool ready_for_gripper(Robot_Arm &arm);
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
void handoff_event(Robot_Arm &arm, const Joints &command, const Joints &previous, const char *reason);
void record_gripper_check(int hand, double position_rad, double torque_nm, const char *outcome);

// Implemented beside the SDK mutex in function.cpp; no motor commands are issued here.
bool solve_local_pose(Robot_Arm &arm, const Joints &seed, const Vec3 &position,
                      const Eigen::Matrix3d &rotation, const Config &config,
                      Joints &solution, double &xyz_residual, double &rotation_residual);
}
