#pragma once
#include <Eigen/Dense>
#include <string>
#include <vector>

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
    // Physical station plane is fixed; image registration can still scale/rotate.
    bool xy_only=true; // only TCP XY is corrected; image registration may scale/rotate
    bool affine_enabled=true;
    bool observe_at_B=true;
    double observe_timeout_s=6.0;
    // B is stationary. A place fix may be older than a grasp frame and still name the same hole.
    double observation_age_s=0.6;
    double target_hold_s=1.2; // bounded motion toward an already measured static XY goal
    double alignment_extension_s=2.0;
    double confirm_stable_s=.08;
    int tracking_max_neighbors=6, tracking_scan_limit=12;
    double affine_min_scale=.65, affine_max_scale=1.5;
    double affine_max_rotation_rad=.35;
    double max_extrapolation_pitches=2.5;
    bool orange_rim_enabled=true;
    bool occupied_orange_rims=true; // use tray rim, never the raised metal edge of an occupied hole
    double rim_min_radius_ratio=.55, rim_max_radius_ratio=1.6;
    int min_coverage_bins=12; // 270 degrees; incomplete half-circles remain invalid
    // Acquisition is independent of the tight, per-hole tracking ROIs.
    bool global_acquisition=true;
    bool boundary_enabled=true; // dark tray / bright background evidence; disable for another fixture
    double global_search_m=.090, global_match_gate_m=.009;
    double global_ambiguity_margin=.75; // one full rim advantage, allowing its residual penalty
    int global_max_hypotheses=4000, reacquire_after_frames=2;
    bool partial_rims=true;
    // First acquisition from arcs needs redundant 2-D support and a repeated fit.
    int partial_acquire_min_neighbors=4, partial_acquire_confirm_frames=2;
    double partial_acquire_stability_m=.004;
    int partial_min_bins=7, single_neighbor_max_frames=4;
    double single_neighbor_step_m=.006, single_neighbor_budget_m=.010;
    double single_neighbor_max_age_s=.6;
    double observe_recovery_after_s=1.2, observe_recovery_lift_m=.020;
    double observe_recovery_speed_m_s=.025;
    bool save_debug_images=false;
    double debug_image_period_s=.25;
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
}
