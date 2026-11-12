#pragma once
#include "t170/common/frame.h"
#include "t170/common/belt_circle_config.h"
#include <Eigen/Dense>
#include <string>
namespace vision {
struct BeltCircleResult {
    bool valid=false;Eigen::Vector2d pixel=Eigen::Vector2d::Zero(),center_xy=Eigen::Vector2d::Zero();
    double radius_m=0,residual_m=0,saturated_fraction=0,elapsed_ms=0;
    int coverage_bins=0,inliers=0;std::string reason;
};
// RGB only. Plane is the fixed workpiece TOP plane, not head-camera depth.
// expected_pixel is the predicted image of this task's target, not an unrestricted image search.
BeltCircleResult locate_belt_circle(const CameraFrameData &frame,const Eigen::Matrix4d &base_camera,
    double plane_z,const Eigen::Vector2d &expected_pixel,const BeltCircleConfig &config);
}
