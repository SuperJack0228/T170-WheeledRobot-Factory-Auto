#pragma once
#include <Eigen/Dense>
#include <array>
#include <functional>
#include <string>
#include <tuple>
#include <vector>
class Arm;
#include "t170/motion/types.h"
int arm_dual_cartesian_track(
    Arm &arm_r,
    Arm &arm_l,
    const Eigen::Matrix<double, 1, 6> &start_r,
    const Eigen::Matrix<double, 1, 6> &start_l,
    bool move_r,
    bool move_l,
    double duration_s,
    const std::function<void(double t_s,
                             Eigen::Matrix<double, 1, 6> &goal_r,
                             Eigen::Matrix<double, 1, 6> &goal_l)> &update_goal);
