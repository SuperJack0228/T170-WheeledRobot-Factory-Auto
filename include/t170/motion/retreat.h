#pragma once
#include <Eigen/Dense>
#include <array>
#include <functional>
#include <string>
#include <tuple>
#include <vector>
class Arm;
/** Fixed configured MODEL joint Home, feedback-verified, no Cartesian IK/line. */
int arm_joint_home(Arm &arm);
#include "t170/motion/types.h"
int arm_vertical_lift(Arm &arm, double lift_m, double speed_m_s,
                      const std::function<void(double)> &on_measured_lift = {});

int arm_return_after_lift(Arm &arm, const Eigen::Matrix<double,1,7> &ready,
                          const std::function<void()> &on_started = {},
                          double notify_at = -1.0, double limit_scale = 1.0);
