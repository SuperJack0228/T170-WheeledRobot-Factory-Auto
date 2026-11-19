// These public interfaces must compile without a robot SDK, Python or camera driver.
#include "t170/common/frame.h"
#include "t170/common/servo_types.h"
#include "t170/control/visual_servo.h"
#include "t170/motion/pose_solver.h"
#include "t170/common/pose_math.h"
#include "t170/motion/cartesian_tracking.h"
#include "t170/motion/feedback.h"
#include "t170/motion/kinematics_adapter.h"
#include "t170/motion/planning.h"
#include "t170/motion/retreat.h"
#include "t170/motion/servo.h"
#include "t170/motion/servo_handoff.h"
#include "t170/motion/trajectory_executor.h"
#include "t170/skills/types.h"
#include "t170/skills/tray.h"
#include "t170/skills/conveyor.h"
#include "t170/vision/types.h"
#include "t170/vision/camera.h"
#include "t170/vision/dl.h"
#include "t170/vision/marker.h"
#include "t170/vision/pose.h"
#include "t170/vision/servo_observer_port.h"
#include "t170/app/options.h"
#include "t170/app/commands.h"
#include "t170/app/application.h"
#include "t170/tasks/context.h"
#include "t170/tasks/factory.h"
#include "t170/hardware/chassis.h"
#include "t170/integration/dispatch/client.h"
#include "t170/integration/dispatch/inbox.h"
#include "t170/telemetry/factory_status.h"
#include "t170/telemetry/dispatch_metrics.h"
#include <type_traits>
#include <iostream>
int main() {
    static_assert(t170::skills::interface_version==1);
    static_assert(t170::skills::index(t170::skills::Hand::Left)==0);
    static_assert(t170::skills::index(t170::skills::Hand::Right)==1);
    static_assert(std::is_same_v<vision::Frame, CameraFrameData>);
    static_assert(std::is_abstract_v<vision::IDlEngine>);
    static_assert(std::is_abstract_v<vision::IPoseSolver>);
    static_assert(std::is_abstract_v<t170::hardware::Chassis>);
    static_assert(std::is_abstract_v<t170::integration::dispatch::Client>);
    static_assert(std::is_abstract_v<t170::integration::dispatch::Gateway>);
    static_assert(static_cast<int>(CameraSlot::Head)==0);
    static_assert(static_cast<int>(CameraSlot::RightHand)==1);
    static_assert(static_cast<int>(CameraSlot::LeftHand)==2);
    static_assert(kGraspDetectClassId==0 && kEmptyHoleClassId==1 && kFallenClassId==2);
    wrist_servo::Config cfg;std::string error;
    if(!wrist_servo::validate_config(cfg,error)) return 1;
    std::cout<<"public module headers: SDK-free interfaces and preserved slot/class identities passed\n";
}
