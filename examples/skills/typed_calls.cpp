// Compile-only examples. Hardware/context creation is owned by t170_factory.
// This file has no main(), starts no devices and cannot move a robot by itself.
#include "t170/skills/tray.h"
#include "t170/skills/conveyor.h"
#include <utility>

namespace t170::skills::examples {
Result grasp_tray(TrayContext &context, const Metadata &target, const Control &control) {
    TrayGraspRequest request;
    request.metadata=target;request.hands={true,true};request.allow_waist_rise=true;
    return execute_tray_grasp(context,request,control);
}
Result place_tray(TrayContext &context, const Metadata &target,
                  const std::array<PartState,2> &held_parts, const Control &control) {
    TrayPlaceRequest request;
    request.metadata=target;request.initial_parts=held_parts;
    request.hands={held_parts[0]==PartState::Holding,held_parts[1]==PartState::Holding};
    return execute_tray_place(context,request,control);
}
std::pair<Result,std::optional<Result>> grasp_conveyor_and_retreat(
    ConveyorContext &context,const Metadata &target,Hand hand,const Pose &goal,const Control &control) {
    ConveyorGraspRequest grasp;grasp.metadata=target;grasp.hand=hand;grasp.goal=goal;
    Result contact=execute_conveyor_grasp(context,grasp,control);
    // Failure may already have changed the held-part state. Preserve both outputs.
    if(!contact.ok()) return {contact,std::nullopt};
    ConveyorRetreatRequest retreat;retreat.metadata=target;
    retreat.metadata.request_id=contact.metadata.request_id+"/retreat";
    retreat.hand=hand;retreat.initial_part=contact.hands[index(hand)].part;
    return {contact,execute_conveyor_retreat(context,retreat,control)};
}
} // namespace t170::skills::examples
