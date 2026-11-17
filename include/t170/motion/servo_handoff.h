#pragma once
#include <Eigen/Dense>
#include <array>
#include <functional>
#include <string>
#include <tuple>
#include <vector>
class Robot_Arm;
#include "t170/motion/types.h"
ArmLineMoveResult arm_dual_line_then_line_nostop(
    Robot_Arm &arm_r,
    const Eigen::Matrix<double, 1, 6> &mid_r,
    const Eigen::Matrix<double, 1, 6> &goal_r,
    bool move_r,
    Robot_Arm &arm_l,
    const Eigen::Matrix<double, 1, 6> &mid_l,
    const Eigen::Matrix<double, 1, 6> &goal_l,
    bool move_l,
    double vel_first_m_s,
    double vel_second_m_s,
    const std::function<void()> &before_move = {},
    bool use_prepared = false,
    const char *log_tag = "收手",
    bool snap_reach = false,
    bool correct_descent = false,
    int hand_target_class = -1);

bool descent_commanded_xy(int hand, double &x_m, double &y_m);
