#pragma once
#include "t170/vision/tray_localizer.h"
#include <opencv2/core.hpp>

namespace wrist_servo::tray_registration {
struct Rim {
    Eigen::Vector2d point=Eigen::Vector2d::Zero(); // coarse, rectified plane coordinates
    bool partial=false;
};
struct Scene {
    struct Edge {Eigen::Vector2d begin,end,inward;};
    std::vector<Rim> rims;
    std::vector<Edge> edges;
    // Conservative silhouette evidence. No boundary outside the image is inferred.
    std::vector<cv::Point> tray_hull;
    cv::Mat hsv;
    Eigen::Matrix3d seed=Eigen::Matrix3d::Identity();
};
struct Result {
    bool valid=false;
    NeighborGridFit fit;
    int hypotheses=0;
    double margin=0;
    bool partial_bootstrap=false;
    std::string reason,detail;
};
Scene discover(const cv::Mat &undistorted_bgr,const Eigen::Matrix3d &seed,
               const TrayBoardGeometry &board,const TrayPlaceVisionConfig &cfg);
Result acquire(const Scene &scene,const TrayTarget &target,const TrayPlaceVisionConfig &cfg,
               const TrayPixelGate &executable_target={}); // argument is a corrected seed-plane point
// Partial orange arcs are fitted with an outer-edge/radius prior, not by closing a C shape.
bool fit_partial_rim(const std::vector<cv::Point> &contour,const cv::Mat &mask,double scale,
                     double radius,const TrayPlaceVisionConfig &cfg,cv::Point2d &center);
}
