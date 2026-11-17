#pragma once
#include <Eigen/Dense>
#include <array>
#include <functional>
#include <string>
#include <tuple>
#include <vector>
class Arm;
#include "t170/motion/types.h"
void save_arm_last_commanded_position(int hand, const int motor_cmd[7]);
