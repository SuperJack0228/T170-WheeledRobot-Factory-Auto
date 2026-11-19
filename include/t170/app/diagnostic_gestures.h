#pragma once
#include <Eigen/Dense>
#include <array>
#include <functional>
#include <string>
#include <tuple>
#include <vector>
class Arm;
#include "t170/motion/types.h"
int arm_wave_right(Arm &arm_r, Arm &arm_l);

int arm_wave_hang(Arm &arm_r, Arm &arm_l);

int arm_bow(Arm &arm_r, Arm &arm_l);

int arm_welcome(Arm &arm_r, Arm &arm_l);

namespace orch {
void wave_grip_ratio(bool is_right, double ratio);
}
