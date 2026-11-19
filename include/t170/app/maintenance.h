#pragma once
#include "t170/tasks/context.h"
namespace orch {
int go_waist_debug(int pose);
int go_waist_cycle();
int go_waist_jog(int joint, double dq_rad);
int vision_grasp_once();
int vision_servo_dual_once();
int run_charge_test();
int vision_aruco_detect_pipeline();
int vision_tray_holes_pipeline();
}
