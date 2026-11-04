// These public interfaces must compile without a robot SDK, Python or camera driver.
#include "t170/common/frame.h"
#include "t170/common/servo_types.h"
#include "t170/control/visual_servo.h"
#include "t170/motion/pose_solver.h"
#include "t170/vision/types.h"
#include "t170/vision/camera.h"
#include "t170/vision/dl.h"
#include "t170/vision/marker.h"
#include "t170/vision/pose.h"
#include "t170/vision/servo_observer_port.h"
#include <type_traits>
#include <iostream>
int main() {
    static_assert(std::is_same_v<vision::Frame, CameraFrameData>);
    static_assert(std::is_abstract_v<vision::IDlEngine>);
    static_assert(std::is_abstract_v<vision::IPoseSolver>);
    static_assert(static_cast<int>(CameraSlot::Head)==0);
    static_assert(static_cast<int>(CameraSlot::RightHand)==1);
    static_assert(static_cast<int>(CameraSlot::LeftHand)==2);
    static_assert(kGraspDetectClassId==0 && kEmptyHoleClassId==1 && kFallenClassId==2);
    wrist_servo::Config cfg;std::string error;
    if(!wrist_servo::validate_config(cfg,error)) return 1;
    std::cout<<"public module headers: SDK-free interfaces and preserved slot/class identities passed\n";
}
