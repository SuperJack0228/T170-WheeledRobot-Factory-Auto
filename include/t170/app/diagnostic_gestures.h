#pragma once
#include <Eigen/Dense>
#include <array>
#include <functional>
#include <string>
#include <tuple>
#include <vector>
class Robot_Arm;
#include "t170/motion/types.h"
int arm_wave_right(Robot_Arm &arm_r, Robot_Arm &arm_l);

int arm_wave_hang(Robot_Arm &arm_r, Robot_Arm &arm_l);

int arm_bow(Robot_Arm &arm_r, Robot_Arm &arm_l);

int arm_welcome(Robot_Arm &arm_r, Robot_Arm &arm_l);

namespace orch {
void wave_grip_ratio(bool is_right, double ratio);
}
