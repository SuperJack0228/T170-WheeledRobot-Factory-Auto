#pragma once
#include "t170/common/frame.h"
#include <Eigen/Dense>
#include <string>
namespace vision {
struct CalibrationHole {
    bool valid=false;
    Eigen::Vector2d pixel=Eigen::Vector2d::Zero(); // raw color coordinates
    int coverage_bins=0;
    double residual_rectified_px=0;
    std::string reason;
};
// A click selects a neighbourhood, not a measurement. Fit a visible orange rim
// after lens correction and metric plane rectification; never issue motion.
CalibrationHole refine_calibration_hole(const CameraFrameData &frame,const Eigen::Matrix4d &base_camera,
    const Eigen::Matrix4d &base_tray,double radius_m,double u,double v);
}
