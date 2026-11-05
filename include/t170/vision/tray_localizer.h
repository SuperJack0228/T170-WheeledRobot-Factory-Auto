#pragma once
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>
#include "t170/common/frame.h"
#include "t170/common/tray_types.h"

namespace wrist_servo {

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
    double pitch,const TrayPlaceVisionConfig &config);
// RGB-only observer. No robot commands, depth image, Python or YOLO dependency.
// The target row/column is immutable for the lifetime of one servo session.
class TrayLocalizer {
public:
    TrayLocalizer();
    ~TrayLocalizer();
    void reset(const TrayTarget &target,const TrayPlaceVisionConfig &config);
    TrayLocalization locate(const CameraFrameData &frame,const Eigen::Matrix4d &base_from_camera);
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}
