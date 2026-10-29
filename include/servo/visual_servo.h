#pragma once

#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <deque>
#include <functional>
#include <string>
#include <vector>

namespace YAML { class Node; }

namespace wrist_servo {
using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Mat2 = Eigen::Matrix2d;
using Joints = Eigen::Matrix<double, 1, 7>;

double monotonic_seconds();

struct MapKnot {
    double height_m = 0.0; // TCP Z above the final TCP Z, not sensor depth
    Vec2 reference_px = Vec2::Zero();
    Mat2 image_jacobian = Mat2::Zero(); // d(pixel)/d(base XY), px/m
};

struct ImageProfile {
    std::string mode = "geometry"; // geometry is explicitly an uncalibrated estimate
    Vec2 reference_px = Vec2::Zero(); // at reference_height_m
    double reference_height_m = 0.0;
    double feature_z_offset_m = 0.0; // visible feature plane minus final TCP Z
    double min_depth_m = 0.015;
    double max_depth_m = 0.65;
    std::vector<MapKnot> knots;
};

struct Config {
    bool belt_place_enabled = false; // no tray-hole feature is assumed on a bare conveyor
    bool require_dual = false; // set by the dedicated dual-hand validation command
    double period_s = 0.01;
    double kp = 1.5;
    double kd = 0.2;
    double max_xy_speed_m_s = 0.040;
    double max_xy_accel_m_s2 = 0.20;
    double max_z_accel_m_s2 = 0.25;
    double brake_delay_s = 0.08;
    double model_uncertainty_m = 0.0005;
    double xy_tolerance_m = 0.002;
    double settle_speed_m_s = 0.003;
    double z_tolerance_m = 0.001;
    double commit_height_m = 0.008;
    double freeze_height_m = 0.030;
    double commit_speed_m_s = 0.060;
    double commit_timeout_s = 5.0;
    // Positive base Z points UP: keep the reference above nominal C, not below it.
    double grasp_height_margin_m = 0.002;
    double grasp_z_accept_above_m = 0.006;
    double grasp_xy_accept_m = 0.006;
    double grasp_relax_after_s = 0.35;
    double finish_speed_m_s = 0.008;
    double finish_stable_s = 0.04;
    double alignment_time_margin_s = 0.12;
    double acquire_descent_m = 0.020;
    double handoff_distance_m = 0.015;
    double handoff_speed_m_s = 0.030;
    double max_correction_m = 0.035;
    double innovation_gate_m = 0.012;
    double goal_filter_alpha = 0.65;
    double max_observation_age_s = 0.35;
    double max_clock_uncertainty_s = 0.12;
    double feedback_age_budget_s = 0.02;
    double loss_timeout_s = 1.5;
    double task_timeout_s = 18.0;
    double max_tick_gap_s = 0.15;
    double max_state_gap_s = 0.06;
    double max_reference_lead_m = 0.012; // soft governor, not instantaneous abort
    double hard_reference_lead_m = 0.025;
    double following_timeout_s = 0.50;
    // Existing field branch bypassed SDK limits. Make that choice explicit and
    // still bound every local solution relative to the SDK approach endpoint.
    bool enforce_sdk_joint_limits = false;
    double max_joint_excursion_rad = 0.70;
    double max_joint_lead_rad = 0.08;
    double max_joint_speed_rad_s = 0.45;
    double max_joint_accel_rad_s2 = 2.0;
    double max_orientation_error_rad = 0.05235987756;
    double solver_position_tolerance_m = 0.0006;
    double solver_rotation_tolerance_rad = 0.00872664626;
    int max_solver_failures = 5;
    int stable_frames = 2;
    double min_confidence = 0.45;
    double acquire_radius_px = 180.0;
    double track_radius_px = 100.0;
    double pair_clearance_m = 0.10;
    double log_period_s = 0.05;
    std::array<ImageProfile, 4> profiles; // left grasp, right grasp, left place, right place
};

void load_config(const YAML::Node &node, Config &config);
bool validate_config(const Config &config, std::string &error);
bool write_config_snapshot(const Config &config, const std::string &path, std::string &error);
const ImageProfile &profile_for(const Config &config, int hand, int target_class);

struct State {
    double time_s = 0.0;
    double sampling_uncertainty_s = 0.0;
    Vec3 position = Vec3::Zero();
    Vec3 velocity = Vec3::Zero();
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Joints joints = Joints::Zero();
};

class StateHistory {
public:
    void push(const State &state);
    bool interpolate(double time_s, double max_gap_s, State &state) const;
    void clear() { samples_.clear(); }
private:
    std::deque<State> samples_;
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
    Vec2 pixel = Vec2::Zero();
    double confidence = 0.0;
    double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
    int width = 0, height = 0;
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

Mapping map_observation(const ImageProfile &profile, const PixelObservation &obs,
                        const State &capture, double final_z,
                        const Vec3 &camera_origin, const Eigen::Matrix3d &camera_rotation);

enum class Phase { Acquiring, Aligning, Descending, Hold, Commit, Settling, Done, Failed };
const char *phase_name(Phase phase);

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
};

class Controller {
public:
    void reset(const Config &config, const State &start, double final_z, double descent_speed,
               const Vec2 &nominal_xy, bool grasp);
    void reset(const Config &config, const State &start, double final_z, double descent_speed) {
        reset(config,start,final_z,descent_speed,start.position.head<2>(),true);
    }
    bool observe(const PixelObservation &obs, const State &capture, const Mapping &map,
                 double now, std::string &reason);
    Output tick(const State &state, double dt);
    void fail(const std::string &reason);
    void feedback_reference(const Vec3 &reference, const Vec3 &velocity) {
        reference_=reference; velocity_=velocity.head<2>(); vz_=velocity.z();
    }
    Vec2 goal_xy() const { return goal_xy_; }
    bool has_goal() const { return have_goal_; }
    Phase phase() const { return phase_; }
private:
    Config cfg_;
    Vec2 initial_xy_ = Vec2::Zero(), goal_xy_ = Vec2::Zero();
    Vec2 velocity_ = Vec2::Zero();
    Vec3 reference_ = Vec3::Zero();
    double acquire_floor_z_ = 0, precision_floor_since_ = -1, finish_since_ = -1;
    bool grasp_ = true, relaxed_grasp_ = false;
    double vz_ = 0.0, start_time_ = 0.0, final_z_ = 0.0, descent_speed_ = 0.0;
    double last_capture_ = -1.0, last_clock_uncertainty_ = 0.0;
    double commit_start_ = -1.0;
    std::uint64_t last_frame_ = 0;
    bool have_goal_ = false;
    int confirmations_ = 0;
    int close_confirmations_ = 0;
    Phase phase_ = Phase::Acquiring;
    std::string failure_;
};

using ForwardModel = std::function<Eigen::Matrix4d(const Joints &)>;
using JointLimitCheck = std::function<bool(const Joints &)>;
bool solve_pose_with_fk(const Joints &seed, const Vec3 &target, const Eigen::Matrix3d &rotation,
                        const Config &config, const ForwardModel &forward, const JointLimitCheck &out_of_limits,
                        Joints &solution, double &xyz_residual, double &rotation_residual);

// Conservative one-dimensional envelope; used with actual motion state and acceleration limits.
double braking_speed(double distance_m, double deceleration_m_s2, double delay_s);
} // namespace wrist_servo
