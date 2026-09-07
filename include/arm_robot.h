#pragma once

#include "Ti5_Arm.h"
#include "function.h"

#include <chrono>
#include <string>
#include <thread>

/// 双臂控制：Robot_Arm 解算 + CAN 电机读写（替代原 kinematics.h 中的 Robot）
class ArmRobot
{
public:
    /// @param mod -1 右臂，1 左臂（与原 Robot 构造一致）
    explicit ArmRobot(int mod);

    Eigen::Matrix<double, 1, 7> getJointPos();
    Eigen::Matrix<double, 1, 6> getTcpPos();
    Eigen::Matrix<double, 1, 6> get_base_visual_pos(const Eigen::Matrix<double, 1, 6> &falan_visualPos);

    int servoJ(Eigen::Matrix<double, 1, 7> &qd);
    int moveJtoJoint(Eigen::Matrix<double, 1, 7> &qd);
    int moveL(Eigen::Matrix<double, 1, 6> &posd, double vel);

    Robot_Arm &solver() { return solver_; }

private:
    Robot_Arm solver_;
    int hand_flag_;
    Eigen::Matrix<double, 1, 6> flange_tool_;

    static constexpr int dof_ = 7;
    static constexpr double rad2cnt_ = 4.0 * 65536.0 / (2.0 * M_PI);
    static constexpr double traj_dt_ = 5e-3;

    void play_joint_trajectory(const Eigen::MatrixXd &traj);
};
