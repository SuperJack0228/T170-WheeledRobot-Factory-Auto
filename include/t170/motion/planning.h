#pragma once
#include <Eigen/Dense>
#include <array>
#include <functional>
#include <string>
#include <tuple>
#include <vector>
class Arm;
#include "t170/motion/types.h"
void clamp_cart_goal_z(Eigen::Matrix<double, 1, 6> &pos, const char *where);

void arm_line_move_set_debug_stage(const char *stage);

void append_line_trajectory_fail_debug(
    const char *hand_label,
    int ret,
    const Eigen::Matrix<double, 1, 6> &goal,
    const char *stage,
    double cart_linear_velocity);

int arm_line_move(
    Arm &arm,
    Eigen::Matrix<double, 1, 6> &pos,
    double cart_linear_velocity,
    bool snap_reach = false);

int arm_line_move_settle(Arm &arm, Eigen::Matrix<double, 1, 6> &pos, double cart_linear_velocity, int settle_ms);






void arm_discard_line_then_line_plan(int hand);



int arm_transfer_move(Arm &arm, Eigen::Matrix<double, 1, 6> &pos, double cart_linear_velocity);

ArmLineMoveResult arm_dual_transfer_move(
    Arm &arm_r,
    Eigen::Matrix<double, 1, 6> &pos_r,
    Arm &arm_l,
    Eigen::Matrix<double, 1, 6> &pos_l,
    double cart_linear_velocity);



ArmLineMoveResult arm_dual_line_move(
    Arm &arm_r,
    Eigen::Matrix<double, 1, 6> &pos_r,
    Arm &arm_l,
    Eigen::Matrix<double, 1, 6> &pos_l,
    double cart_linear_velocity,
    bool snap_reach = false);

ArmLineMoveResult arm_dual_line_move_selective(
    Arm &arm_r,
    Eigen::Matrix<double, 1, 6> &pos_r,
    bool move_r,
    Arm &arm_l,
    Eigen::Matrix<double, 1, 6> &pos_l,
    bool move_l,
    double cart_linear_velocity,
    bool snap_reach = false);

int arm_joint_move(Arm &arm, Eigen::Matrix<double, 1, 7> &qd);
