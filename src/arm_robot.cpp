#include "arm_robot.h"

#include "Ti5_socketcan.h"
#include "seg_pose_bridge.h"

#include <algorithm>

namespace
{

std::string robot_arm_model_yaml_path()
{
    return project_root_dir() + "/config/Robot_Arm_Model.yaml";
}

} // namespace

ArmRobot::ArmRobot(int mod)
    : solver_(mod == -1 ? Robot_Arm("T7", "T170", "right", robot_arm_model_yaml_path())
                        : Robot_Arm("T7", "T170", "left", robot_arm_model_yaml_path())),
      hand_flag_(mod == -1 ? 1 : 0)
{
    if (mod == -1)
        flange_tool_ << 0, 0.08, -0.11, 0, 0, 0;
    else
        flange_tool_ << 0, -0.08, -0.11, 0, 0, 0;
}

Eigen::Matrix<double, 1, 7> ArmRobot::getJointPos()
{
    Eigen::Matrix<double, 1, 7> motor_q;
    int motor_pos[dof_] = {0};

    while (true)
    {
        get_motor_position(motor_pos, hand_flag_ == 0 ? 0 : 1);
        if (hand_flag_ == 0)
        {
            bool all_zero = true;
            for (int i = 0; i < dof_; ++i)
            {
                if (motor_pos[i] != 0)
                {
                    all_zero = false;
                    break;
                }
            }
            if (!all_zero)
                break;
        }
        else
        {
            break;
        }
    }

    for (int i = 0; i < dof_; ++i)
        motor_q(i) = static_cast<double>(motor_pos[i]) / rad2cnt_;

    return motor_q;
}

Eigen::Matrix<double, 1, 6> ArmRobot::getTcpPos()
{
    const Eigen::Matrix<double, 1, 7> q = getJointPos();
    const Eigen::Matrix4d T = solver_.Forward_Kinematics(q);
    return T2PosEulerAngles(T);
}

Eigen::Matrix<double, 1, 6> ArmRobot::get_base_visual_pos(
    const Eigen::Matrix<double, 1, 6> &falan_visualPos)
{
    const Eigen::Matrix<double, 1, 7> q = getJointPos();
    const Eigen::Matrix4d T_tcp = solver_.Forward_Kinematics(q);
    const Eigen::Matrix4d T_flange = T_tcp * TR(flange_tool_).inverse();
    const Eigen::Matrix4d T_base_visual = T_flange * TR(falan_visualPos);
    return T2PosEulerAngles(T_base_visual);
}

int ArmRobot::servoJ(Eigen::Matrix<double, 1, 7> &qd)
{
    int motor_positions[dof_];
    for (int i = 0; i < dof_; ++i)
        motor_positions[i] = static_cast<int>(qd(i) * rad2cnt_);

    set_motor_position(motor_positions, hand_flag_ == 0 ? 0 : 1);
    return 0;
}

void ArmRobot::play_joint_trajectory(const Eigen::MatrixXd &traj)
{
    if (traj.rows() == 0)
        return;

    for (int i = 0; i < traj.rows(); ++i)
    {
        const auto cycle_start = std::chrono::high_resolution_clock::now();
        Eigen::Matrix<double, 1, 7> q = traj.row(i);
        servoJ(q);

        const double elapsed_ms = std::chrono::duration<double, std::milli>(
                                      std::chrono::high_resolution_clock::now() - cycle_start)
                                      .count();
        const double sleep_ms = std::max(0.0, traj_dt_ * 1000.0 - elapsed_ms);
        if (sleep_ms > 1e-6)
            std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(sleep_ms));
    }
}

int ArmRobot::moveJtoJoint(Eigen::Matrix<double, 1, 7> &qd)
{
    const Eigen::Matrix<double, 1, 7> qc = getJointPos();
    Eigen::MatrixXd q_start = qc;
    Eigen::MatrixXd q_end = qd;
    Eigen::MatrixXd traj;
    const int ret = solver_.Joint_Trajectory(q_start, q_end, traj);
    if (ret != 0)
        return ret;

    play_joint_trajectory(traj);
    return 0;
}

int ArmRobot::moveL(Eigen::Matrix<double, 1, 6> &posd, double vel)
{
    (void)vel;
    solver_.Cart_Linear_Velocity = 0.10;

    const Eigen::Matrix<double, 1, 7> qc = getJointPos();
    Eigen::MatrixXd q_start = qc;
    Eigen::MatrixXd traj;
    const int ret = solver_.Line_Trajectory(q_start, posd, traj, qc(1));
    if (ret != 0)
        return ret;

    play_joint_trajectory(traj);
    return 0;
}
