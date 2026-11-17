#pragma once
#include <Eigen/Dense>
#include <array>
#include <functional>
#include <string>
#include <tuple>
#include <vector>
class Robot_Arm;
void arm_line_move_set_debug_stage(const char *stage);
struct ArmLineMoveResult
{
    int ret_r = 0;
    int ret_l = 0;
};
class ArmLineMoveDebugStage
{
public:
    explicit ArmLineMoveDebugStage(const char *stage) { arm_line_move_set_debug_stage(stage); }
    ~ArmLineMoveDebugStage() { arm_line_move_set_debug_stage(nullptr); }
    ArmLineMoveDebugStage(const ArmLineMoveDebugStage &) = delete;
    ArmLineMoveDebugStage &operator=(const ArmLineMoveDebugStage &) = delete;
};
struct ArmArrivalRecord
{
    std::string stage;
    std::string hand;
    double goal_x = 0, goal_y = 0, goal_z = 0;
    double actual_x = 0, actual_y = 0, actual_z = 0;
    double err_x = 0, err_y = 0, err_z = 0;
    double err_m = 0;
    bool settled = false;
};
struct ArmLinkFk
{
    Eigen::Vector3d shoulder = Eigen::Vector3d::Zero();
    Eigen::Vector3d elbow = Eigen::Vector3d::Zero();
    Eigen::Vector3d wrist = Eigen::Vector3d::Zero();
    Eigen::Vector3d flange = Eigen::Vector3d::Zero();
    Eigen::Vector3d tcp = Eigen::Vector3d::Zero();
    double tcp_mdh_err = 0.0;
};
