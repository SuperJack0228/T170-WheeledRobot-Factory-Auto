#pragma once
#include <Eigen/Dense>
#include <array>
#include <functional>
#include <string>
#include <tuple>
#include <vector>
class Arm;
#include "t170/motion/types.h"
void arm_arrival_log_clear();

std::vector<ArmArrivalRecord> arm_arrival_log_copy();

bool record_arm_cartesian_arrival(
    Arm &arm,
    const Eigen::Matrix<double, 1, 6> &goal,
    const char *stage = nullptr,
    double settle_sec = 15.0);

void log_arm_encoder_command_vs_actual(Arm &arm, const char *tag);

Eigen::Matrix<double, 1, 7> arm_get_joint_pos(Arm &arm);



bool wait_arms_motors_stopped(
    bool wait_r,
    bool wait_l,
    double timeout_sec = 15.0,
    int min_zero_count = 6,
    const char *log_stage = "手相机前");



constexpr const char kLogPhaseSep[] =
    "================================================================================";

constexpr const char kLogCycleSep[] =
    "********************************************************************************";

void log_phase_banner(const char *title);







