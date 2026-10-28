#include "servo/visual_servo.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace wrist_servo {
double monotonic_seconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

const ImageProfile &profile_for(const Config &c, int hand, int cls) {
    return c.profiles.at((cls == 1 ? 2 : 0) + (hand == 1 ? 1 : 0));
}

bool validate_config(const Config &c, std::string &error) {
    auto positive = [&](double x, const char *name) {
        if (!std::isfinite(x) || x <= 0) { error = std::string(name) + " must be finite and positive"; return false; }
        return true;
    };
    for (const auto &[x, name] : std::vector<std::pair<double, const char *>>{
        {c.period_s,"period_s"},{c.kp,"kp"},{c.max_xy_speed_m_s,"max_xy_speed_m_s"},
        {c.max_xy_accel_m_s2,"max_xy_accel_m_s2"},{c.max_z_accel_m_s2,"max_z_accel_m_s2"},
        {c.xy_tolerance_m,"xy_tolerance_m"},{c.settle_speed_m_s,"settle_speed_m_s"},
        {c.z_tolerance_m,"z_tolerance_m"},{c.slow_xy_error_m,"slow_xy_error_m"},
        {c.stop_xy_error_m,"stop_xy_error_m"},{c.commit_speed_m_s,"commit_speed_m_s"},
        {c.commit_timeout_s,"commit_timeout_s"},
        {c.max_correction_m,"max_correction_m"},{c.innovation_gate_m,"innovation_gate_m"},
        {c.max_observation_age_s,"max_observation_age_s"},{c.max_clock_uncertainty_s,"max_clock_uncertainty_s"},
        {c.feedback_age_budget_s,"feedback_age_budget_s"},
        {c.loss_timeout_s,"loss_timeout_s"},{c.task_timeout_s,"task_timeout_s"},
        {c.max_tick_gap_s,"max_tick_gap_s"},{c.max_state_gap_s,"max_state_gap_s"},
        {c.max_reference_lead_m,"max_reference_lead_m"},{c.max_joint_lead_rad,"max_joint_lead_rad"},
        {c.max_joint_speed_rad_s,"max_joint_speed_rad_s"},{c.max_joint_accel_rad_s2,"max_joint_accel_rad_s2"},
        {c.max_orientation_error_rad,"max_orientation_error_rad"},
        {c.solver_position_tolerance_m,"solver_position_tolerance_m"},
        {c.solver_rotation_tolerance_rad,"solver_rotation_tolerance_rad"},
        {c.acquire_radius_px,"acquire_radius_px"},{c.track_radius_px,"track_radius_px"},
        {c.pair_clearance_m,"pair_clearance_m"},{c.log_period_s,"log_period_s"}}) {
        if (!positive(x, name)) return false;
    }
    if (!std::isfinite(c.kd) || c.kd < 0 || !std::isfinite(c.brake_delay_s) || c.brake_delay_s < 0 ||
        !std::isfinite(c.model_uncertainty_m) || c.model_uncertainty_m < 0 ||
        !std::isfinite(c.commit_height_m) || c.commit_height_m < 0 ||
        !(c.goal_filter_alpha > 0 && c.goal_filter_alpha <= 1) ||
        !(c.min_confidence > 0 && c.min_confidence <= 1) ||
        c.period_s > c.max_tick_gap_s || c.max_observation_age_s >= c.loss_timeout_s ||
        c.task_timeout_s <= c.loss_timeout_s || c.stable_frames < 2 || c.max_solver_failures < 1 ||
        c.slow_xy_error_m >= c.stop_xy_error_m || c.xy_tolerance_m >= c.slow_xy_error_m ||
        c.model_uncertainty_m >= 0.5*c.xy_tolerance_m) {
        error = "invalid controller limits, gains or time budgets"; return false;
    }
    for (const auto &p : c.profiles) {
        if ((p.mode != "geometry" && p.mode != "table") || !p.reference_px.allFinite() ||
            !std::isfinite(p.reference_height_m) || !std::isfinite(p.feature_z_offset_m) ||
            !positive(p.min_depth_m,"min_depth_m") || !positive(p.max_depth_m,"max_depth_m") ||
            p.min_depth_m >= p.max_depth_m) { error = "invalid image profile"; return false; }
        if (p.mode == "table" && p.knots.empty()) { error = "table profile requires measured knots"; return false; }
        double previous = -std::numeric_limits<double>::infinity();
        for (const auto &k : p.knots) {
            if (!std::isfinite(k.height_m) || k.height_m <= previous ||
                !k.reference_px.allFinite() || !k.image_jacobian.allFinite()) {
                error = "profile knots must be finite and strictly increasing"; return false;
            }
            Eigen::JacobiSVD<Mat2> svd(k.image_jacobian);
            if (svd.singularValues()(1) < 1 || svd.singularValues()(0) / svd.singularValues()(1) > 30) {
                error = "singular or ill-conditioned profile knot"; return false;
            }
            previous = k.height_m;
        }
    }
    error.clear(); return true;
}

void StateHistory::push(const State &s) {
    if (!std::isfinite(s.time_s) || !s.position.allFinite() || !s.joints.allFinite()) return;
    if (!samples_.empty() && s.time_s <= samples_.back().time_s) return;
    samples_.push_back(s);
    while (samples_.size() > 600 || (!samples_.empty() && s.time_s - samples_.front().time_s > 3.0))
        samples_.pop_front();
}

bool StateHistory::interpolate(double t, double max_gap, State &s) const {
    if (samples_.empty() || !std::isfinite(t) || t < samples_.front().time_s || t > samples_.back().time_s)
        return false; // never silently extrapolate a commanded state
    auto hi = std::lower_bound(samples_.begin(), samples_.end(), t,
                              [](const State &a, double time) { return a.time_s < time; });
    if (hi == samples_.end()) return false;
    if (hi->time_s == t) { s = *hi; return true; }
    if (hi == samples_.begin()) return false;
    const auto lo = std::prev(hi);
    const double gap = hi->time_s - lo->time_s;
    if (!(gap > 0) || gap > max_gap) return false;
    const double u = (t - lo->time_s) / gap;
    s = *lo; s.time_s = t;
    s.sampling_uncertainty_s=std::max(lo->sampling_uncertainty_s,hi->sampling_uncertainty_s);
    s.position = lo->position + u * (hi->position - lo->position);
    s.velocity = lo->velocity + u * (hi->velocity - lo->velocity);
    s.joints = lo->joints + u * (hi->joints - lo->joints);
    s.rotation = Eigen::Quaterniond(lo->rotation).slerp(u, Eigen::Quaterniond(hi->rotation)).toRotationMatrix();
    return true;
}

Mapping map_observation(const ImageProfile &p, const PixelObservation &o, const State &s,
                        double final_z, const Vec3 &origin, const Eigen::Matrix3d &R) {
    Mapping m;
    const double h = s.position.z() - final_z;
    if (!o.pixel.allFinite() || !std::isfinite(h)) { m.reason = "nonfinite_pixel_or_height"; return m; }
    if (p.mode == "table") {
        if (p.knots.empty() || h < p.knots.front().height_m || h > p.knots.back().height_m) {
            m.reason = "height_outside_profile"; return m;
        }
        auto hi = std::lower_bound(p.knots.begin(), p.knots.end(), h,
                                  [](const MapKnot &k, double height) { return k.height_m < height; });
        if (hi == p.knots.begin() || hi->height_m == h) {
            m.reference_px = hi->reference_px; m.jacobian = hi->image_jacobian;
        } else {
            const auto lo = std::prev(hi);
            const double u = (h - lo->height_m) / (hi->height_m - lo->height_m);
            m.reference_px = lo->reference_px + u * (hi->reference_px - lo->reference_px);
            m.jacobian = lo->image_jacobian + u * (hi->image_jacobian - lo->image_jacobian);
        }
    } else {
        if (!(o.fx > 1 && o.fy > 1) || !origin.allFinite() || !R.allFinite()) {
            m.reason = "invalid_camera_geometry"; return m;
        }
        const double xn0 = (p.reference_px.x() - o.cx) / o.fx;
        const double yn0 = (p.reference_px.y() - o.cy) / o.fy;
        const Vec3 ray0(xn0, yn0, 1);
        const double origin_at_reference_z = origin.z() - (h - p.reference_height_m);
        const double denominator0 = (R * ray0).z();
        const double feature_z = final_z + p.feature_z_offset_m;
        if (std::abs(denominator0) < 0.15) { m.reason = "reference_ray_parallel_to_plane"; return m; }
        const double reference_depth = (feature_z - origin_at_reference_z) / denominator0;
        const Vec3 desired = reference_depth * ray0 - R.transpose() * Vec3(0, 0, h - p.reference_height_m);
        if (!(reference_depth >= p.min_depth_m && reference_depth <= p.max_depth_m) ||
            !(desired.z() >= p.min_depth_m && desired.z() <= p.max_depth_m)) {
            m.reason = "reference_depth_outside_range"; return m;
        }
        m.reference_px << o.cx + o.fx * desired.x() / desired.z(),
                          o.cy + o.fy * desired.y() / desired.z();
        const double xn = (o.pixel.x() - o.cx) / o.fx;
        const double yn = (o.pixel.y() - o.cy) / o.fy;
        const double denominator = (R * Vec3(xn, yn, 1)).z();
        if (std::abs(denominator) < 0.15) { m.reason = "observed_ray_parallel_to_plane"; return m; }
        m.depth_m = (feature_z - origin.z()) / denominator;
        if (!(m.depth_m >= p.min_depth_m && m.depth_m <= p.max_depth_m)) {
            m.reason = "feature_depth_outside_range"; return m; // no invented 4cm floor
        }
        for (int j = 0; j < 2; ++j) {
            m.jacobian(0,j) = -o.fx / m.depth_m * (R(j,0) - xn * R(j,2));
            m.jacobian(1,j) = -o.fy / m.depth_m * (R(j,1) - yn * R(j,2));
        }
    }
    if (!m.jacobian.allFinite() || !m.reference_px.allFinite()) { m.reason = "nonfinite_map"; return m; }
    if(o.width>0 && o.height>0 && (m.reference_px.x()<0 || m.reference_px.y()<0 ||
       m.reference_px.x()>=o.width || m.reference_px.y()>=o.height)) {
        m.reason="desired_feature_outside_image"; return m;
    }
    Eigen::JacobiSVD<Mat2> svd(m.jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto singular = svd.singularValues();
    m.condition = singular(1) > 0 ? singular(0) / singular(1) : INFINITY;
    if (singular(1) < 1 || m.condition > 30) { m.reason = "ill_conditioned_map"; return m; }
    m.error_px = o.pixel - m.reference_px;
    m.remaining_m = -svd.solve(m.error_px);
    m.valid = m.remaining_m.allFinite();
    m.reason = m.valid ? "ok" : "nonfinite_correction";
    return m;
}

const char *phase_name(Phase p) {
    switch (p) {
        case Phase::Acquiring: return "acquiring"; case Phase::Aligning: return "aligning";
        case Phase::Descending: return "descending"; case Phase::Hold: return "hold";
        case Phase::Commit: return "commit"; case Phase::Done: return "done";
        case Phase::Failed: return "failed";
    }
    return "invalid";
}

double braking_speed(double d, double a, double delay) {
    if (!(d > 0 && a > 0 && delay >= 0)) return 0;
    // Algebraically equivalent to sqrt((a*t)^2+2*a*d)-a*t; avoids cancellation.
    return 2 * a * d / (std::sqrt(a*a*delay*delay + 2*a*d) + a*delay);
}

void Controller::reset(const Config &c, const State &s, double z, double speed) {
    cfg_ = c; initial_xy_ = s.position.head<2>(); goal_xy_ = initial_xy_;
    velocity_ = s.velocity.head<2>(); vz_ = s.velocity.z();
    start_time_ = s.time_s; final_z_ = z; descent_speed_ = speed;
    have_goal_ = false; last_capture_ = -1; last_frame_ = 0; confirmations_ = 0;
    commit_start_=-1;
    phase_ = Phase::Acquiring; failure_.clear();
}

bool Controller::observe(const PixelObservation &o, const State &s, const Mapping &m,
                         double now, std::string &reason) {
    if (phase_ == Phase::Commit || phase_ == Phase::Done || phase_ == Phase::Failed) { reason = "session_not_accepting"; return false; }
    if (!m.valid) { reason = m.reason; return false; }
    if (o.frame <= last_frame_ || o.capture_s <= last_capture_) { reason = "duplicate_or_out_of_order"; return false; }
    if (!std::isfinite(o.capture_s) || !std::isfinite(o.clock_uncertainty_s) ||
        o.clock_uncertainty_s < 0 || o.clock_uncertainty_s > cfg_.max_clock_uncertainty_s ||
        now < o.capture_s || now - o.capture_s + o.clock_uncertainty_s > cfg_.max_observation_age_s) {
        reason = "stale_or_uncertain_timestamp"; return false;
    }
    if (!std::isfinite(o.confidence) || o.confidence < cfg_.min_confidence) { reason = "low_confidence"; return false; }
    const Vec2 candidate = s.position.head<2>() + m.remaining_m;
    if (!candidate.allFinite() || (candidate - initial_xy_).norm() > cfg_.max_correction_m) {
        reason = "outside_correction_envelope"; return false;
    }
    if (have_goal_ && (candidate - goal_xy_).norm() > cfg_.innovation_gate_m) {
        reason = "goal_innovation_rejected"; return false;
    }
    if(have_goal_ && o.capture_s-last_capture_>cfg_.max_observation_age_s) confirmations_=0;
    goal_xy_ = have_goal_ ? Vec2(goal_xy_ + cfg_.goal_filter_alpha * (candidate - goal_xy_)) : candidate;
    have_goal_ = true; last_capture_ = o.capture_s; last_frame_ = o.frame;
    last_clock_uncertainty_ = o.clock_uncertainty_s;
    const bool aligned = m.remaining_m.norm() <= cfg_.xy_tolerance_m &&
                         s.velocity.head<2>().norm() <= cfg_.settle_speed_m_s;
    confirmations_ = aligned ? confirmations_ + 1 : 0;
    reason = "accepted"; return true;
}

void Controller::fail(const std::string &reason) {
    if(phase_==Phase::Failed) return; // retain the initiating fault, not subsequent peer/timeout symptoms
    phase_ = Phase::Failed; failure_ = reason;
}

Output Controller::tick(const State &s, double dt) {
    Output out; out.age_s = have_goal_ ? s.time_s - last_capture_ : -1;
    out.confirmations = confirmations_;
    if (!std::isfinite(dt) || dt <= 0 || dt > cfg_.max_tick_gap_s || !s.position.allFinite() || !s.velocity.allFinite())
        fail("invalid_state_or_tick_gap");
    if (s.time_s - start_time_ > cfg_.task_timeout_s) fail("task_timeout");
    if(phase_==Phase::Commit && s.time_s-commit_start_>cfg_.commit_timeout_s) fail("commit_timeout");
    if (phase_ != Phase::Commit && phase_ != Phase::Done && phase_ != Phase::Failed &&
        s.time_s - (have_goal_ ? last_capture_ : start_time_) > cfg_.loss_timeout_s)
        fail(have_goal_ ? "target_lost_timeout" : "no_valid_observation_timeout");
    Vec2 target_v = Vec2::Zero(); double target_z = 0;
    out.remaining_m = goal_xy_ - s.position.head<2>();
    const bool fresh = have_goal_ && out.age_s >= 0 &&
                       out.age_s + last_clock_uncertainty_ <= cfg_.max_observation_age_s;
    if (phase_ != Phase::Failed && phase_ != Phase::Done) {
        const bool committing = phase_ == Phase::Commit;
        if (!committing && !fresh) { phase_ = have_goal_ ? Phase::Hold : Phase::Acquiring; out.reason = "waiting_fresh_vision"; }
        else {
            const double distance = out.remaining_m.norm();
            target_v = cfg_.kp * out.remaining_m - cfg_.kd * s.velocity.head<2>();
            const double budget = std::max(0.0, distance - 0.25*cfg_.xy_tolerance_m - cfg_.model_uncertainty_m);
            out.brake_speed = braking_speed(budget, cfg_.max_xy_accel_m_s2,
                                            cfg_.brake_delay_s + last_clock_uncertainty_);
            const double cap = std::min(cfg_.max_xy_speed_m_s, out.brake_speed);
            if (target_v.norm() > cap && target_v.norm() > 0) target_v *= cap / target_v.norm();
            if (distance <= cfg_.xy_tolerance_m) target_v.setZero();
            const double h = s.position.z() - final_z_;
            const bool settled = distance <= cfg_.xy_tolerance_m &&
                                 s.velocity.head<2>().norm() <= cfg_.settle_speed_m_s &&
                                 confirmations_ >= cfg_.stable_frames;
            if (!committing && h <= cfg_.commit_height_m + cfg_.z_tolerance_m && settled) {
                phase_ = Phase::Commit; commit_start_=s.time_s;
            }
            if (phase_ != Phase::Commit) {
                out.descent_scale = std::clamp((cfg_.stop_xy_error_m - distance) /
                                              (cfg_.stop_xy_error_m - cfg_.slow_xy_error_m), 0.0, 1.0);
                // Stop before the last visible region, including actual Z braking travel.
                const double z_budget = std::max(0.0, h - cfg_.commit_height_m);
                target_z = -std::min(descent_speed_ * out.descent_scale,
                                    braking_speed(z_budget, cfg_.max_z_accel_m_s2, cfg_.brake_delay_s));
                phase_ = out.descent_scale > 0 ? Phase::Descending : Phase::Aligning;
            } else {
                out.descent_scale = 1;
                target_z = -std::min({descent_speed_, cfg_.commit_speed_m_s,
                                    braking_speed(std::max(0.0,h-0.5*cfg_.z_tolerance_m),
                                                  cfg_.max_z_accel_m_s2, cfg_.brake_delay_s)});
                if (h < -cfg_.z_tolerance_m) fail("z_overshoot");
                if (h <= cfg_.z_tolerance_m && h >= -cfg_.z_tolerance_m &&
                    std::abs(s.velocity.z()) <= cfg_.settle_speed_m_s && settled) {
                    phase_ = Phase::Done; target_v.setZero(); target_z = 0;
                }
            }
        }
    }
    if (phase_ == Phase::Failed || phase_ == Phase::Done) { target_v.setZero(); target_z = 0; }
    const double usable_dt = (std::isfinite(dt) && dt > 0) ? std::min(dt,cfg_.max_tick_gap_s) : cfg_.period_s;
    Vec2 change = target_v - velocity_;
    if (change.norm() > cfg_.max_xy_accel_m_s2*usable_dt)
        change *= cfg_.max_xy_accel_m_s2*usable_dt/change.norm();
    velocity_ += change;
    vz_ += std::clamp(target_z-vz_, -cfg_.max_z_accel_m_s2*usable_dt, cfg_.max_z_accel_m_s2*usable_dt);
    out.velocity_xy = velocity_; out.velocity_z = vz_; out.phase = phase_;
    if (phase_ == Phase::Failed) out.reason = failure_;
    return out;
}
} // namespace wrist_servo
