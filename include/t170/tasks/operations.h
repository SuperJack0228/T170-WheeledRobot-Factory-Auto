#pragma once
#include "t170/tasks/context.h"
namespace orch {
int go_home(Arm &arm_r, Arm &arm_l);
int go_home_arms_keep_waist_z(Arm &arm_r, Arm &arm_l);
int home_waist_upright();
int go_belt_ready(Arm &arm_r, Arm &arm_l);
int go_belt_place(Arm &arm_r, Arm &arm_l);
int go_belt_grasp_rpy(Arm &arm_r, Arm &arm_l);
int go_belt();
int go_belt_station();
int go_belt_grasp(bool from_place = false);
int go_belt2_ready(Arm &arm_r, Arm &arm_l);
int go_belt2_grasp_rpy(Arm &arm_r, Arm &arm_l);
int go_belt2();

int go_belt2_grasp();
int go_grasp_ready(Arm &arm_r, Arm &arm_l, int row_group);
int go_tray2_ready(Arm &arm_r, Arm &arm_l, int row_group);
int vision_place_tray2(bool allow_right = true, bool allow_left = true, bool return_to_tray = true);
int vision_grasp_existing_pipeline(bool depart_after_grasp = true);
void start_chassis_status_reporter();
void stop_chassis_status_reporter();
bool live_gripper_holding(bool is_right);
void clear_last_grasped(bool right, bool left);
}
