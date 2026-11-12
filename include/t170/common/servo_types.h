#pragma once
#include <Eigen/Dense>
#include <cstdint>
#include <string>

namespace wrist_servo {
using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Mat2 = Eigen::Matrix2d;
using Joints = Eigen::Matrix<double, 1, 7>;

double monotonic_seconds();
enum class Phase { Acquiring, Aligning, Descending, Hold, Commit, Settling, Done, Failed };
const char *phase_name(Phase phase);

struct State {
    double time_s = 0.0;
    double sampling_uncertainty_s = 0.0;
    Vec3 position = Vec3::Zero();
    Vec3 velocity = Vec3::Zero();
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Joints joints = Joints::Zero();
};

struct PixelObservation {
    std::uint64_t session = 0;
    std::uint64_t frame = 0;
    double capture_s = 0.0;
    double completed_s = 0.0;
    double clock_uncertainty_s = 0.0;
    double device_timestamp_ms = 0.0;
    int timestamp_source = 0;
    double grab_ms = 0.0;
    double infer_ms = 0.0;
    bool capture_timing_accounted=false;
    double submit_age_ms=0;
    Vec2 pixel = Vec2::Zero();
    double confidence = 0.0;
    double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
    int width = 0, height = 0;
    bool virtual_target=false;
    int target_row=0,target_col=0,neighbor_count=0,marker_corner_count=0;
    double plane_z_m=0,plane_reprojection_px=0;
    int tray_rois=0,tray_contours=0,tray_rejected_size=0,tray_rejected_shape=0;
    int tray_rejected_coverage=0,tray_rejected_residual=0,tray_skipped_occupied=0;
    int tray_candidates=0,tray_rejected_distance=0,tray_rejected_target=0,tray_rejected_short=0;
    int tray_global_hypotheses=0,tray_occupied_used=0,tray_partial_used=0;
    double tray_identity_margin=0;
    std::string tray_stage;
    double tray_undistort_ms=0,tray_acquisition_ms=0,tray_prepare_ms=0,tray_roi_ms=0,tray_partial_ms=0,tray_fit_ms=0;
    int tray_partial_calls=0,tray_pruned_neighbors=0;
    double geometry_radius_m=0,geometry_residual_m=0,geometry_ms=0,saturated_fraction=0;
    int geometry_coverage_bins=0,geometry_inliers=0;
    std::string vision_source="yolo";
};

struct Mapping {
    bool valid = false;
    std::string reason;
    Vec2 reference_px = Vec2::Zero();
    Vec2 error_px = Vec2::Zero();
    Mat2 jacobian = Mat2::Zero();
    Vec2 remaining_m = Vec2::Zero();
    double depth_m = 0.0;
    double condition = 0.0;
};

struct Output {
    Phase phase = Phase::Acquiring;
    std::string reason;
    Vec2 remaining_m = Vec2::Zero();
    Vec2 velocity_xy = Vec2::Zero();
    double velocity_z = 0.0;
    double age_s = -1.0;
    double brake_speed = 0.0;
    double descent_scale = 0.0;
    int confirmations = 0;
    Vec3 reference = Vec3::Zero();
    double alignment_time_s = 0, available_time_s = 0;
    double reference_floor_z = 0, finish_xy_tolerance_m = 0;
    bool relaxed_grasp = false;
    bool place_locked = false;
    int stable_goal_frames = 0;
    int close_confirmations=0;
    bool static_goal_motion=false,alignment_extended=false;
    double place_elapsed_s=0,place_deadline_s=0;
    std::string lock_blocker;
};
}
