#pragma once
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>
#include "realsense_get.h"

namespace wrist_servo {
struct TrayPlaceVisionConfig {
    bool enabled=false;
    std::string board_config="board_yf100_aruco/config.yaml";
    double association_gate_m=.025;
    double reprojection_gate_px=3.0;
    double circle_residual_m=.0015;
    double max_plane_update_m=.030;
    int min_neighbors=2;
    double rectified_pixels_per_m=2000;
    // Static tray / fixed wrist: retain the head plane Z and normal; estimate XY only.
    bool xy_only=true;
    bool orange_rim_enabled=true;
    bool occupied_orange_rims=true; // use tray rim, never the raised metal edge of an occupied hole
    double rim_min_radius_ratio=.55, rim_max_radius_ratio=1.6;
    int min_coverage_bins=12; // 270 degrees; incomplete half-circles remain invalid
};
struct TrayBoardGeometry {
    int rows=6,cols=6;
    double pitch_m=.075,radius_m=.019,offset_x_m=0,offset_y_m=0;
    double marker_pitch_m=.30,marker_side_m=.0312;
    std::vector<int> marker_ids;
    Eigen::Vector2d hole(int row,int col) const {
        return {((rows+1)*.5-row)*pitch_m+offset_x_m,
                (col-(cols+1)*.5)*pitch_m+offset_y_m};
    }
};
struct TrayAnchor { int row=0,col=0; Eigen::Vector3d base=Eigen::Vector3d::Zero(); int class_id=-1; };
struct TrayTarget {
    bool valid=false;
    int row=0,col=0;
    TrayBoardGeometry board;
    Eigen::Matrix4d base_from_tray=Eigen::Matrix4d::Identity();
    std::vector<int> occupied_holes; // row*100+col; unknown class stays usable
};
bool load_tray_board(const std::string &path,TrayBoardGeometry &board,std::string &error);
bool make_tray_target(const std::vector<TrayAnchor> &anchors,const TrayBoardGeometry &board,
                      int row,int col,TrayTarget &target,std::string &error);
struct TrayLocalization {
    bool valid=false,translation_only=false;
    Eigen::Vector2d pixel=Eigen::Vector2d::Zero();
    double plane_z_m=0,reprojection_px=0;
    int neighbors=0,marker_corners=0;
    int rois=0,contours=0,rejected_size=0,rejected_shape=0;
    int rejected_coverage=0,rejected_residual=0,skipped_occupied=0;
    std::string source,reason;
};
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
