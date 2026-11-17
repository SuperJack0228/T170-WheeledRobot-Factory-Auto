#pragma once
#include "t170/skills/legacy/runtime.h"
namespace move_box {
GraspToStandbyResult run_tray_cycle_impl(
    gripper::Gripper &g,
    Robot_Arm &arm_r,
    Robot_Arm &arm_l,
    WaistRobot &waist,
    Eigen::Matrix<double, 1, 6> &posup_down_3_layer,
    RealSenseMultiCam &cameras,
    SegPoseBridge &bridge,
    const std::array<double, 16> &cam2robot,
    std::string &err,
    const std::function<bool()> &should_abort = nullptr,
    SegEngineId engine_id = SegEngineId::Default,
    bool adaptive_approach_rpy = false,
    bool holding_r = false,
    bool holding_l = false,
    const std::function<void(bool did_r, bool did_l)> &on_place_retract = {},
    bool allow_waist_rise = false,
    const std::function<void()> &on_waist_up = {});
}
