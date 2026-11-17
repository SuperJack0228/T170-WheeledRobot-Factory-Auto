#pragma once
#include "t170/skills/types.h"
#include <Eigen/Core>
#include <limits>
class Arm;
struct ConveyorStationConfig;
namespace gripper { class Gripper; }

namespace t170::skills {
using Pose = Eigen::Matrix<double, 1, 6>; // x/y/z metres, rx/ry/rz radians; current arm base.
using Joints = Eigen::Matrix<double, 1, 7>; // Motor joint radians.
struct ConveyorHooks {
    std::function<void(const char *)> log_gripper;
    std::function<void()> on_released;
    std::function<void(double)> on_measured_lift;
    std::function<void()> on_return_started;
};
struct ConveyorContext {
    gripper::Gripper &gripper;
    Arm &right, &left;
    const ConveyorStationConfig &station; // Host resolves workcell configuration for this call.
    ConveyorHooks hooks;
};
struct ConveyorGraspRequest {
    Metadata metadata;
    Hand hand = Hand::Right;
    Pose goal = Pose::Constant(std::numeric_limits<double>::quiet_NaN());
    std::string log_tag = "belt_grasp";
};
struct ConveyorPlaceRequest {
    Metadata metadata;
    Hand hand = Hand::Right;
    Pose goal = Pose::Constant(std::numeric_limits<double>::quiet_NaN());
    PartState initial_part = PartState::Unknown;
};
struct ConveyorRetreatRequest {
    Metadata metadata;
    Hand hand = Hand::Right;
    bool after_place = false;
    bool retry_empty = false;
    std::optional<Joints> retry_ready;
    PartState initial_part = PartState::Unknown;
    std::string log_tag = "belt_grasp";
};
// Contact skills end with the gripper action; retreat is explicit so opposite arms can overlap.
Result execute_conveyor_grasp(ConveyorContext &, const ConveyorGraspRequest &, const Control & = {});
Result execute_conveyor_place(ConveyorContext &, const ConveyorPlaceRequest &, const Control & = {});
Result execute_conveyor_retreat(ConveyorContext &, const ConveyorRetreatRequest &, const Control & = {});
} // namespace t170::skills
