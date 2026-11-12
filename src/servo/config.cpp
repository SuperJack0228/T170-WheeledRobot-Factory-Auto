#include "t170/control/visual_servo.h"
#include <yaml-cpp/yaml.h>
#include <stdexcept>
#include <fstream>
#if __has_include("servo_build_identity.h")
#include "servo_build_identity.h"
#endif

namespace wrist_servo {
namespace {
template<class T> void read(const YAML::Node &n, const char *key, T &value) {
    if (n && n[key]) value = n[key].as<T>();
}
Vec2 vec2(const YAML::Node &n) {
    if (!n.IsSequence() || n.size() != 2) throw std::runtime_error("servo pixel/reference must contain 2 values");
    return Vec2(n[0].as<double>(), n[1].as<double>());
}
void profile(const YAML::Node &n, ImageProfile &p) {
    if (!n) return;
    read(n,"mode",p.mode); read(n,"reference_height_m",p.reference_height_m);
    read(n,"feature_z_offset_m",p.feature_z_offset_m);
    read(n,"plane_relative_reference",p.plane_relative_reference);
    read(n,"constrain_reference_tilt",p.constrain_reference_tilt);
    read(n,"reference_roll_rad",p.reference_roll_rad); read(n,"reference_pitch_rad",p.reference_pitch_rad);
    read(n,"reference_tcp_above_plane_m",p.reference_tcp_above_plane_m);
    read(n,"min_depth_m",p.min_depth_m); read(n,"max_depth_m",p.max_depth_m);
    if (n["reference_px"]) p.reference_px = vec2(n["reference_px"]);
    if (n["knots"]) {
        if (!n["knots"].IsSequence()) throw std::runtime_error("servo knots must be a sequence");
        p.knots.clear();
        for (const auto &node : n["knots"]) {
            MapKnot k;
            k.height_m = node["height_m"].as<double>();
            k.reference_px = vec2(node["reference_px"]);
            const auto a = node["jacobian_px_per_m"];
            if (!a.IsSequence() || a.size()!=2) throw std::runtime_error("servo Jacobian must be 2x2 px/m");
            k.image_jacobian.row(0) = vec2(a[0]).transpose();
            k.image_jacobian.row(1) = vec2(a[1]).transpose();
            p.knots.push_back(k);
        }
    }
}
}
void load_config(const YAML::Node &n, Config &c) {
    read(n,"belt_place_enabled",c.belt_place_enabled);
    read(n,"belt_grasp_enabled",c.belt_grasp_enabled);
    read(n,"require_dual",c.require_dual);
    const auto circle=n?n["belt2_circle"]:YAML::Node();
#define CIRCLE_READ(field) read(circle,#field,c.belt2_circle.field)
    CIRCLE_READ(enabled);CIRCLE_READ(radius_m);CIRCLE_READ(min_radius_m);CIRCLE_READ(max_radius_m);
    CIRCLE_READ(radius_tolerance);CIRCLE_READ(feature_above_final_m);CIRCLE_READ(search_radius_m);
    CIRCLE_READ(residual_m);CIRCLE_READ(pixels_per_m);CIRCLE_READ(max_time_ms);
    CIRCLE_READ(min_coverage_bins);CIRCLE_READ(min_inliers);CIRCLE_READ(ransac_trials);CIRCLE_READ(saturation_threshold);
#undef CIRCLE_READ

    const auto placement=n ? n["tray_place"] : YAML::Node();
#define PLACE_READ(field) read(placement,#field,c.tray_place.field)
    PLACE_READ(enabled); PLACE_READ(board_config); PLACE_READ(association_gate_m);
    PLACE_READ(reprojection_gate_px); PLACE_READ(circle_residual_m);
    PLACE_READ(max_plane_update_m); PLACE_READ(min_neighbors); PLACE_READ(rectified_pixels_per_m);
    PLACE_READ(affine_enabled); PLACE_READ(observe_at_B); PLACE_READ(observe_timeout_s);
    PLACE_READ(observation_age_s);
    PLACE_READ(target_hold_s); PLACE_READ(alignment_extension_s); PLACE_READ(confirm_stable_s);
    PLACE_READ(tracking_max_neighbors); PLACE_READ(tracking_scan_limit);
    PLACE_READ(affine_min_scale); PLACE_READ(affine_max_scale); PLACE_READ(affine_max_rotation_rad);
    PLACE_READ(max_extrapolation_pitches);
    PLACE_READ(xy_only); PLACE_READ(orange_rim_enabled); PLACE_READ(rim_min_radius_ratio);
    PLACE_READ(occupied_orange_rims);
    PLACE_READ(rim_max_radius_ratio); PLACE_READ(min_coverage_bins);
    PLACE_READ(global_acquisition);
    PLACE_READ(boundary_enabled);
    PLACE_READ(global_search_m);
    PLACE_READ(global_match_gate_m);
    PLACE_READ(global_ambiguity_margin);
    PLACE_READ(global_max_hypotheses);
    PLACE_READ(reacquire_after_frames);
    PLACE_READ(partial_rims);
    PLACE_READ(partial_min_bins);
    PLACE_READ(single_neighbor_max_frames);
    PLACE_READ(single_neighbor_step_m);
    PLACE_READ(single_neighbor_budget_m);
    PLACE_READ(single_neighbor_max_age_s);
    PLACE_READ(observe_recovery_after_s);
    PLACE_READ(observe_recovery_lift_m);
    PLACE_READ(observe_recovery_speed_m_s);
    PLACE_READ(save_debug_images);
    PLACE_READ(debug_image_period_s);
#undef PLACE_READ
#define SERVO_READ(field) read(n, #field, c.field)
    SERVO_READ(period_s); SERVO_READ(kp); SERVO_READ(kd);
    SERVO_READ(max_xy_speed_m_s); SERVO_READ(max_xy_accel_m_s2); SERVO_READ(max_z_accel_m_s2);
    SERVO_READ(brake_delay_s); SERVO_READ(model_uncertainty_m); SERVO_READ(xy_tolerance_m);
    SERVO_READ(settle_speed_m_s); SERVO_READ(z_tolerance_m);
    SERVO_READ(commit_height_m); SERVO_READ(commit_speed_m_s);
    SERVO_READ(commit_timeout_s);
    SERVO_READ(freeze_height_m); SERVO_READ(grasp_height_margin_m); SERVO_READ(grasp_z_accept_above_m);
    SERVO_READ(grasp_xy_accept_m); SERVO_READ(grasp_relax_after_s); SERVO_READ(finish_speed_m_s);
    SERVO_READ(grasp_z_feedback_tolerance_m);
    SERVO_READ(place_xy_accept_m); SERVO_READ(place_lock_height_m); SERVO_READ(place_visibility_floor_m);
    SERVO_READ(place_goal_stability_m); SERVO_READ(place_lock_frames); SERVO_READ(place_finish_speed_m_s);
    SERVO_READ(place_height_margin_m); SERVO_READ(place_z_accept_above_m);
    SERVO_READ(finish_stable_s); SERVO_READ(alignment_time_margin_s); SERVO_READ(acquire_descent_m);
    SERVO_READ(handoff_distance_m); SERVO_READ(handoff_speed_m_s);
    SERVO_READ(hard_reference_lead_m); SERVO_READ(following_timeout_s);
    SERVO_READ(max_correction_m); SERVO_READ(innovation_gate_m); SERVO_READ(goal_filter_alpha);
    SERVO_READ(max_observation_age_s); SERVO_READ(max_clock_uncertainty_s); SERVO_READ(loss_timeout_s);
    SERVO_READ(feedback_age_budget_s);
    SERVO_READ(task_timeout_s); SERVO_READ(max_tick_gap_s); SERVO_READ(max_state_gap_s);
    SERVO_READ(max_reference_lead_m); SERVO_READ(max_joint_lead_rad); SERVO_READ(max_joint_speed_rad_s);
    SERVO_READ(max_joint_accel_rad_s2); SERVO_READ(max_orientation_error_rad);
    SERVO_READ(solver_position_tolerance_m); SERVO_READ(solver_rotation_tolerance_rad);
    SERVO_READ(max_solver_failures); SERVO_READ(stable_frames); SERVO_READ(min_confidence);
    SERVO_READ(acquire_radius_px); SERVO_READ(track_radius_px); SERVO_READ(pair_clearance_m);
    SERVO_READ(log_period_s);
    SERVO_READ(enforce_sdk_joint_limits); SERVO_READ(max_joint_excursion_rad);
#undef SERVO_READ
    const char *names[] = {"left_grasp","right_grasp","left_place","right_place"};
    for (int i=0;i<4;++i) profile(n ? n[names[i]] : YAML::Node(), c.profiles[i]);
    std::string error;
    if (!validate_config(c,error)) throw std::runtime_error("visual_servo: " + error);
}

bool write_config_snapshot(const Config &c,const std::string &path,std::string &error) {
    try {
        YAML::Emitter out;
        out.SetDoublePrecision(17);
        out << YAML::BeginMap << YAML::Key << "schema_version" << YAML::Value << 2;
#ifdef T170_SERVO_FULL_REVISION
        out << YAML::Key << "build_revision" << YAML::Value << T170_SERVO_FULL_REVISION;
        out << YAML::Key << "build_dirty" << YAML::Value << T170_SERVO_BUILD_DIRTY;
        out << YAML::Key << "build_diff_sha256" << YAML::Value << T170_SERVO_DIFF_SHA256;
        out << YAML::Key << "build_source_config_sha256" << YAML::Value << T170_SERVO_SOURCE_SHA256;
#else
        out << YAML::Key << "build_revision" << YAML::Value << "unknown";
#endif
#ifdef T170_BUILD_CONFIGURATION
        out << YAML::Key << "build_configuration" << YAML::Value << T170_BUILD_CONFIGURATION;
#endif
        out << YAML::Key << "build_time" << YAML::Value << __DATE__ " " __TIME__;
        out << YAML::Key << "visual_servo" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "belt_place_enabled" << YAML::Value << c.belt_place_enabled;
        out << YAML::Key << "belt_grasp_enabled" << YAML::Value << c.belt_grasp_enabled;
        out << YAML::Key << "require_dual" << YAML::Value << c.require_dual;
        out << YAML::Key << "tray_place" << YAML::Value << YAML::BeginMap;
#define PLACE_WRITE(field) out << YAML::Key << #field << YAML::Value << c.tray_place.field
        PLACE_WRITE(enabled); PLACE_WRITE(board_config); PLACE_WRITE(association_gate_m);
        PLACE_WRITE(reprojection_gate_px); PLACE_WRITE(circle_residual_m);
        PLACE_WRITE(max_plane_update_m); PLACE_WRITE(min_neighbors); PLACE_WRITE(rectified_pixels_per_m);
        PLACE_WRITE(affine_enabled); PLACE_WRITE(observe_at_B); PLACE_WRITE(observe_timeout_s);
        PLACE_WRITE(observation_age_s);
        PLACE_WRITE(target_hold_s); PLACE_WRITE(alignment_extension_s); PLACE_WRITE(confirm_stable_s);
        PLACE_WRITE(tracking_max_neighbors); PLACE_WRITE(tracking_scan_limit);
        PLACE_WRITE(affine_min_scale); PLACE_WRITE(affine_max_scale); PLACE_WRITE(affine_max_rotation_rad);
        PLACE_WRITE(max_extrapolation_pitches);
        PLACE_WRITE(xy_only); PLACE_WRITE(orange_rim_enabled); PLACE_WRITE(rim_min_radius_ratio);
        PLACE_WRITE(occupied_orange_rims);
        PLACE_WRITE(rim_max_radius_ratio); PLACE_WRITE(min_coverage_bins);
        PLACE_WRITE(global_acquisition);
        PLACE_WRITE(boundary_enabled);
        PLACE_WRITE(global_search_m);
        PLACE_WRITE(global_match_gate_m);
        PLACE_WRITE(global_ambiguity_margin);
        PLACE_WRITE(global_max_hypotheses);
        PLACE_WRITE(reacquire_after_frames);
        PLACE_WRITE(partial_rims);
        PLACE_WRITE(partial_min_bins);
        PLACE_WRITE(single_neighbor_max_frames);
        PLACE_WRITE(single_neighbor_step_m);
        PLACE_WRITE(single_neighbor_budget_m);
        PLACE_WRITE(single_neighbor_max_age_s);
        PLACE_WRITE(observe_recovery_after_s);
        PLACE_WRITE(observe_recovery_lift_m);
        PLACE_WRITE(observe_recovery_speed_m_s);
        PLACE_WRITE(save_debug_images);
        PLACE_WRITE(debug_image_period_s);
#undef PLACE_WRITE
        out << YAML::EndMap;
        out << YAML::Key << "period_s" << YAML::Value << c.period_s;
        out << YAML::Key << "kp" << YAML::Value << c.kp;
        out << YAML::Key << "kd" << YAML::Value << c.kd;
        out << YAML::Key << "max_xy_speed_m_s" << YAML::Value << c.max_xy_speed_m_s;
        out << YAML::Key << "max_xy_accel_m_s2" << YAML::Value << c.max_xy_accel_m_s2;
        out << YAML::Key << "max_z_accel_m_s2" << YAML::Value << c.max_z_accel_m_s2;
        out << YAML::Key << "brake_delay_s" << YAML::Value << c.brake_delay_s;
        out << YAML::Key << "model_uncertainty_m" << YAML::Value << c.model_uncertainty_m;
        out << YAML::Key << "xy_tolerance_m" << YAML::Value << c.xy_tolerance_m;
        out << YAML::Key << "settle_speed_m_s" << YAML::Value << c.settle_speed_m_s;
        out << YAML::Key << "z_tolerance_m" << YAML::Value << c.z_tolerance_m;
        out << YAML::Key << "commit_height_m" << YAML::Value << c.commit_height_m;
        out << YAML::Key << "commit_speed_m_s" << YAML::Value << c.commit_speed_m_s;
        out << YAML::Key << "commit_timeout_s" << YAML::Value << c.commit_timeout_s;
        out << YAML::Key << "max_correction_m" << YAML::Value << c.max_correction_m;
        out << YAML::Key << "innovation_gate_m" << YAML::Value << c.innovation_gate_m;
        out << YAML::Key << "goal_filter_alpha" << YAML::Value << c.goal_filter_alpha;
        out << YAML::Key << "max_observation_age_s" << YAML::Value << c.max_observation_age_s;
        out << YAML::Key << "max_clock_uncertainty_s" << YAML::Value << c.max_clock_uncertainty_s;
        out << YAML::Key << "feedback_age_budget_s" << YAML::Value << c.feedback_age_budget_s;
        out << YAML::Key << "loss_timeout_s" << YAML::Value << c.loss_timeout_s;
        out << YAML::Key << "task_timeout_s" << YAML::Value << c.task_timeout_s;
        out << YAML::Key << "max_tick_gap_s" << YAML::Value << c.max_tick_gap_s;
        out << YAML::Key << "max_state_gap_s" << YAML::Value << c.max_state_gap_s;
        out << YAML::Key << "max_reference_lead_m" << YAML::Value << c.max_reference_lead_m;
        out << YAML::Key << "max_joint_lead_rad" << YAML::Value << c.max_joint_lead_rad;
        out << YAML::Key << "max_joint_speed_rad_s" << YAML::Value << c.max_joint_speed_rad_s;
        out << YAML::Key << "max_joint_accel_rad_s2" << YAML::Value << c.max_joint_accel_rad_s2;
        out << YAML::Key << "max_orientation_error_rad" << YAML::Value << c.max_orientation_error_rad;
        out << YAML::Key << "solver_position_tolerance_m" << YAML::Value << c.solver_position_tolerance_m;
        out << YAML::Key << "solver_rotation_tolerance_rad" << YAML::Value << c.solver_rotation_tolerance_rad;
        out << YAML::Key << "max_solver_failures" << YAML::Value << c.max_solver_failures;
        out << YAML::Key << "stable_frames" << YAML::Value << c.stable_frames;
        out << YAML::Key << "min_confidence" << YAML::Value << c.min_confidence;
        out << YAML::Key << "acquire_radius_px" << YAML::Value << c.acquire_radius_px;
        out << YAML::Key << "belt2_circle" << YAML::Value << YAML::BeginMap;
#define CIRCLE_WRITE(field) out << YAML::Key << #field << YAML::Value << c.belt2_circle.field
        CIRCLE_WRITE(enabled);CIRCLE_WRITE(radius_m);CIRCLE_WRITE(min_radius_m);CIRCLE_WRITE(max_radius_m);
        CIRCLE_WRITE(radius_tolerance);CIRCLE_WRITE(feature_above_final_m);CIRCLE_WRITE(search_radius_m);
        CIRCLE_WRITE(residual_m);CIRCLE_WRITE(pixels_per_m);CIRCLE_WRITE(max_time_ms);
        CIRCLE_WRITE(min_coverage_bins);CIRCLE_WRITE(min_inliers);CIRCLE_WRITE(ransac_trials);CIRCLE_WRITE(saturation_threshold);
#undef CIRCLE_WRITE
        out << YAML::EndMap;
        out << YAML::Key << "track_radius_px" << YAML::Value << c.track_radius_px;
        out << YAML::Key << "pair_clearance_m" << YAML::Value << c.pair_clearance_m;
        out << YAML::Key << "log_period_s" << YAML::Value << c.log_period_s;
        out << YAML::Key << "freeze_height_m" << YAML::Value << c.freeze_height_m;
        out << YAML::Key << "grasp_height_margin_m" << YAML::Value << c.grasp_height_margin_m;
        out << YAML::Key << "grasp_z_accept_above_m" << YAML::Value << c.grasp_z_accept_above_m;
#define FINISH_WRITE(field) out << YAML::Key << #field << YAML::Value << c.field
        FINISH_WRITE(grasp_z_feedback_tolerance_m); FINISH_WRITE(place_xy_accept_m);
        FINISH_WRITE(place_lock_height_m); FINISH_WRITE(place_visibility_floor_m);
        FINISH_WRITE(place_goal_stability_m); FINISH_WRITE(place_lock_frames); FINISH_WRITE(place_finish_speed_m_s);
        FINISH_WRITE(place_height_margin_m); FINISH_WRITE(place_z_accept_above_m);
#undef FINISH_WRITE
        out << YAML::Key << "grasp_xy_accept_m" << YAML::Value << c.grasp_xy_accept_m;
        out << YAML::Key << "grasp_relax_after_s" << YAML::Value << c.grasp_relax_after_s;
        out << YAML::Key << "finish_speed_m_s" << YAML::Value << c.finish_speed_m_s;
        out << YAML::Key << "finish_stable_s" << YAML::Value << c.finish_stable_s;
        out << YAML::Key << "alignment_time_margin_s" << YAML::Value << c.alignment_time_margin_s;
        out << YAML::Key << "acquire_descent_m" << YAML::Value << c.acquire_descent_m;
        out << YAML::Key << "handoff_distance_m" << YAML::Value << c.handoff_distance_m;
        out << YAML::Key << "handoff_speed_m_s" << YAML::Value << c.handoff_speed_m_s;
        out << YAML::Key << "hard_reference_lead_m" << YAML::Value << c.hard_reference_lead_m;
        out << YAML::Key << "following_timeout_s" << YAML::Value << c.following_timeout_s;
        out << YAML::Key << "enforce_sdk_joint_limits" << YAML::Value << c.enforce_sdk_joint_limits;
        out << YAML::Key << "max_joint_excursion_rad" << YAML::Value << c.max_joint_excursion_rad;
        const char *names[]={"left_grasp","right_grasp","left_place","right_place"};
        for(int i=0;i<4;++i) {
            const auto &p=c.profiles[i];
            out << YAML::Key << names[i] << YAML::Value << YAML::BeginMap;
            out << YAML::Key << "mode" << YAML::Value << p.mode;
            out << YAML::Key << "reference_px" << YAML::Value << YAML::Flow << YAML::BeginSeq
                << p.reference_px.x() << p.reference_px.y() << YAML::EndSeq;
            out << YAML::Key << "constrain_reference_tilt" << YAML::Value << p.constrain_reference_tilt;
            out << YAML::Key << "reference_roll_rad" << YAML::Value << p.reference_roll_rad;
            out << YAML::Key << "reference_pitch_rad" << YAML::Value << p.reference_pitch_rad;
            out << YAML::Key << "plane_relative_reference" << YAML::Value << p.plane_relative_reference;
            out << YAML::Key << "reference_tcp_above_plane_m" << YAML::Value << p.reference_tcp_above_plane_m;
            out << YAML::Key << "reference_height_m" << YAML::Value << p.reference_height_m;
            out << YAML::Key << "feature_z_offset_m" << YAML::Value << p.feature_z_offset_m;
            out << YAML::Key << "min_depth_m" << YAML::Value << p.min_depth_m;
            out << YAML::Key << "max_depth_m" << YAML::Value << p.max_depth_m;
            out << YAML::Key << "knots" << YAML::Value << YAML::BeginSeq;
            for(const auto &k:p.knots) {
                out << YAML::BeginMap << YAML::Key << "height_m" << YAML::Value << k.height_m;
                out << YAML::Key << "reference_px" << YAML::Value << YAML::Flow << YAML::BeginSeq
                    << k.reference_px.x() << k.reference_px.y() << YAML::EndSeq;
                out << YAML::Key << "jacobian_px_per_m" << YAML::Value << YAML::BeginSeq;
                for(int r=0;r<2;++r) out << YAML::Flow << YAML::BeginSeq
                    << k.image_jacobian(r,0) << k.image_jacobian(r,1) << YAML::EndSeq;
                out << YAML::EndSeq << YAML::EndMap;
            }
            out << YAML::EndSeq << YAML::EndMap;
        }
        out << YAML::EndMap << YAML::EndMap;
        if(!out.good()) throw std::runtime_error(out.GetLastError());
        std::ofstream file(path); file << out.c_str() << '\n'; file.flush();
        if(!file) throw std::runtime_error("cannot write " + path);
        return true;
    } catch(const std::exception &ex) { error=ex.what(); return false; }
}
} // namespace wrist_servo
