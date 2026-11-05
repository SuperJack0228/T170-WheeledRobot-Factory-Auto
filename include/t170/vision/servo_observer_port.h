#pragma once
// Read state / publish observations only. Implemented by the motion owner.
#include "t170/control/visual_servo.h"

namespace wrist_servo {
struct ObservationContext {
    bool active = false, descending = false;
    bool vision_required = true;
    std::uint64_t session = 0;
    int target_class = -1;
    std::string task_tag;
    double final_z = 0;
    Vec2 nominal_xy = Vec2::Zero();
    State latest;
    Vec3 camera_origin = Vec3::Zero();
    Eigen::Matrix3d camera_rotation = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d desired_rotation = Eigen::Matrix3d::Identity();
    ImageProfile profile;
    Config config;
    TrayTarget tray_target;
    int image_width = 0, image_height = 0;
};

bool observation_context(int hand, ObservationContext &context);
bool camera_at_capture(int hand,std::uint64_t session,double capture_s,Eigen::Matrix4d &base_camera,
                       State *capture_state=nullptr);
bool submit_observation(int hand, const PixelObservation &observation);
void observation_event(int hand, std::uint64_t session, const char *reason,
                       const PixelObservation &observation = {});
}
