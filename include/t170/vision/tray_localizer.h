#pragma once
#include <Eigen/Dense>
#include <memory>
#include <functional>
#include <string>
#include <vector>
#include "t170/common/frame.h"
#include "t170/common/tray_types.h"

namespace wrist_servo {
using TrayPixelGate=std::function<bool(const Eigen::Vector2d &)>;

bool load_tray_board(const std::string &path,TrayBoardGeometry &board,std::string &error);
bool make_tray_target(const std::vector<TrayAnchor> &anchors,const TrayBoardGeometry &board,
                      int row,int col,TrayTarget &target,std::string &error);
struct TrayImageMark {
    int row=0,col=0;
    float predicted_u=0,predicted_v=0;
    float observed_u=-1,observed_v=-1;
    bool accepted=false,occupied=false;
};
struct TrayLocalization {
    bool valid=false,translation_only=false;
    Eigen::Vector2d pixel=Eigen::Vector2d::Zero();
    double plane_z_m=0,reprojection_px=0;
    int neighbors=0,marker_corners=0;
    int rois=0,contours=0,rejected_size=0,rejected_shape=0;
    int rejected_coverage=0,rejected_residual=0,skipped_occupied=0;
    float target_u=-1,target_v=-1;
    std::vector<TrayImageMark> marks;
    // All discovered candidates, including those not assigned a row/column.
    std::vector<Eigen::Vector2d> candidate_pixels;
    int rejected_distance=0,rejected_target=0,rejected_short=0;
    int global_hypotheses=0,occupied_used=0,partial_used=0;
    double identity_margin=0;
    double undistort_ms=0,acquisition_ms=0,prepare_ms=0,roi_ms=0,partial_ms=0,fit_ms=0;
    int partial_calls=0,pruned_neighbors=0;
    std::string stage;
    std::string source,reason,detail;
};
struct NeighborGridFit {
    bool valid=false;
    Eigen::Matrix3d transform=Eigen::Matrix3d::Identity();
    Eigen::Vector2d target=Eigen::Vector2d::Zero();
    std::vector<int> inliers;
    std::string model,detail;
};
// Pure 2D registration in the coarse rectified plane. Does not estimate robot Z.
NeighborGridFit fit_neighbor_grid(const std::vector<Eigen::Vector2d> &nominal,
    const std::vector<Eigen::Vector2d> &observed,const Eigen::Vector2d &target,
    double pitch,const TrayPlaceVisionConfig &config,bool identity_resolved=false);
// RGB-only observer. No robot commands, depth image, Python or YOLO dependency.
// The target row/column is immutable for the lifetime of one servo session.
class TrayLocalizer {
public:
    TrayLocalizer();
    ~TrayLocalizer();
    void reset(const TrayTarget &target,const TrayPlaceVisionConfig &config);
    TrayLocalization locate(const CameraFrameData &frame,const Eigen::Matrix4d &base_from_camera,
                            const TrayPixelGate &executable_target={});
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}
