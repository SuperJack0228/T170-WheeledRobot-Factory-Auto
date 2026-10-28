#include "servo/visual_servo.h"
#include <yaml-cpp/yaml.h>
#include <stdexcept>
#include <fstream>

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
    read(n,"require_dual",c.require_dual);
#define SERVO_READ(field) read(n, #field, c.field)
    SERVO_READ(period_s); SERVO_READ(kp); SERVO_READ(kd);
    SERVO_READ(max_xy_speed_m_s); SERVO_READ(max_xy_accel_m_s2); SERVO_READ(max_z_accel_m_s2);
    SERVO_READ(brake_delay_s); SERVO_READ(model_uncertainty_m); SERVO_READ(xy_tolerance_m);
    SERVO_READ(settle_speed_m_s); SERVO_READ(z_tolerance_m); SERVO_READ(slow_xy_error_m);
    SERVO_READ(stop_xy_error_m); SERVO_READ(commit_height_m); SERVO_READ(commit_speed_m_s);
    SERVO_READ(commit_timeout_s);
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
        out << YAML::BeginMap << YAML::Key << "schema_version" << YAML::Value << 1;
#ifdef T170_SERVO_REVISION
        out << YAML::Key << "build_revision" << YAML::Value << T170_SERVO_REVISION;
#else
        out << YAML::Key << "build_revision" << YAML::Value << "unknown";
#endif
        out << YAML::Key << "build_time" << YAML::Value << __DATE__ " " __TIME__;
        out << YAML::Key << "visual_servo" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "belt_place_enabled" << YAML::Value << c.belt_place_enabled;
        out << YAML::Key << "require_dual" << YAML::Value << c.require_dual;
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
        out << YAML::Key << "slow_xy_error_m" << YAML::Value << c.slow_xy_error_m;
        out << YAML::Key << "stop_xy_error_m" << YAML::Value << c.stop_xy_error_m;
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
        out << YAML::Key << "track_radius_px" << YAML::Value << c.track_radius_px;
        out << YAML::Key << "pair_clearance_m" << YAML::Value << c.pair_clearance_m;
        out << YAML::Key << "log_period_s" << YAML::Value << c.log_period_s;
        const char *names[]={"left_grasp","right_grasp","left_place","right_place"};
        for(int i=0;i<4;++i) {
            const auto &p=c.profiles[i];
            out << YAML::Key << names[i] << YAML::Value << YAML::BeginMap;
            out << YAML::Key << "mode" << YAML::Value << p.mode;
            out << YAML::Key << "reference_px" << YAML::Value << YAML::Flow << YAML::BeginSeq
                << p.reference_px.x() << p.reference_px.y() << YAML::EndSeq;
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
