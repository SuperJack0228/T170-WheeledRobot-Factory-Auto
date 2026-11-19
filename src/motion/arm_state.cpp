#include "internal/arm_internal.h"
using namespace Eigen;
using namespace std;
int g_arm_last_cmd_cnt[2][7] = {};
bool g_arm_last_cmd_valid[2] = {false, false};
const char *g_arm_line_move_debug_stage = nullptr;
std::mutex g_arm_line_move_stage_mu;
std::mutex g_arm_kin_mu[2];
int g_planned_descent_row[2] = {-1, -1};
wrist_servo::Joints g_servo_joint_origin[2];
