#pragma once
#include "t170/skills/types.h"
#include <Eigen/Core>
class Robot_Arm;
class WaistRobot;
class RealSenseMultiCam;
class SegPoseBridge;
namespace gripper { class Gripper; }

namespace t170::skills {
enum class Detector { Default = 0, Head = 1, Hand = 2 };
struct TrayHooks {
    std::function<void(bool did_right, bool did_left)> on_clearance;
    std::function<void()> on_waist_up;
};
struct TrayContext {
    gripper::Gripper &gripper;
    Robot_Arm &right, &left;
    WaistRobot &waist;
    Eigen::Matrix<double, 1, 6> &waist_pose; // Existing mutable, measured waist state.
    RealSenseMultiCam &cameras;
    SegPoseBridge &vision;
    std::array<double, 16> head_camera_to_robot;
    TrayHooks hooks;
};
struct TrayGraspRequest {
    Metadata metadata;
    Hands hands;
    Hands already_holding{false, false}; // Skip these hands without opening them.
    Detector detector = Detector::Default;
    bool adaptive_approach_rpy = false;
    bool allow_waist_rise = false;
};
struct TrayPlaceRequest {
    Metadata metadata;
    Hands hands;
    Hands already_placed{false, false};
    std::array<PartState, 2> initial_parts{PartState::Unknown, PartState::Unknown};
    Detector detector = Detector::Default;
};
// Robot must already be at the tray; no chassis navigation or dispatch protocol here.
Result execute_tray_grasp(TrayContext &, const TrayGraspRequest &, const Control & = {});
Result execute_tray_place(TrayContext &, const TrayPlaceRequest &, const Control & = {});
} // namespace t170::skills
