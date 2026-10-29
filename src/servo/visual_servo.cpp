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
    const auto &p=c.tray_place;
    if(p.board_config.empty() || !std::isfinite(p.association_gate_m) || p.association_gate_m<=0 ||
       !std::isfinite(p.reprojection_gate_px) || p.reprojection_gate_px<=0 ||
       !std::isfinite(p.circle_residual_m) || p.circle_residual_m<=0 ||
       !std::isfinite(p.max_plane_update_m) || p.max_plane_update_m<=0 || p.max_plane_update_m>.05 ||
       p.min_neighbors<2 || p.min_neighbors>12 || !std::isfinite(p.rectified_pixels_per_m) ||
       p.rectified_pixels_per_m<500 || p.rectified_pixels_per_m>4000) {
        error="invalid tray placement vision configuration";return false;
    }
    auto positive = [&](double x, const char *name) {
        if (!std::isfinite(x) || x <= 0) { error = std::string(name) + " must be finite and positive"; return false; }
        return true;
    };
    for (const auto &[x, name] : std::vector<std::pair<double, const char *>>{
        {c.period_s,"period_s"},{c.kp,"kp"},{c.max_xy_speed_m_s,"max_xy_speed_m_s"},
        {c.max_xy_accel_m_s2,"max_xy_accel_m_s2"},{c.max_z_accel_m_s2,"max_z_accel_m_s2"},
        {c.xy_tolerance_m,"xy_tolerance_m"},{c.settle_speed_m_s,"settle_speed_m_s"},
        {c.z_tolerance_m,"z_tolerance_m"},{c.commit_speed_m_s,"commit_speed_m_s"},
        {c.commit_timeout_s,"commit_timeout_s"},
        {c.freeze_height_m,"freeze_height_m"},{c.grasp_height_margin_m,"grasp_height_margin_m"},
        {c.grasp_z_accept_above_m,"grasp_z_accept_above_m"},{c.grasp_xy_accept_m,"grasp_xy_accept_m"},
        {c.grasp_relax_after_s,"grasp_relax_after_s"},{c.finish_speed_m_s,"finish_speed_m_s"},
        {c.finish_stable_s,"finish_stable_s"},{c.alignment_time_margin_s,"alignment_time_margin_s"},
        {c.acquire_descent_m,"acquire_descent_m"},{c.handoff_distance_m,"handoff_distance_m"},
        {c.handoff_speed_m_s,"handoff_speed_m_s"},{c.hard_reference_lead_m,"hard_reference_lead_m"},
        {c.following_timeout_s,"following_timeout_s"},
        {c.max_correction_m,"max_correction_m"},{c.innovation_gate_m,"innovation_gate_m"},
        {c.max_observation_age_s,"max_observation_age_s"},{c.max_clock_uncertainty_s,"max_clock_uncertainty_s"},
        {c.feedback_age_budget_s,"feedback_age_budget_s"},
        {c.loss_timeout_s,"loss_timeout_s"},{c.task_timeout_s,"task_timeout_s"},
        {c.max_tick_gap_s,"max_tick_gap_s"},{c.max_state_gap_s,"max_state_gap_s"},
        {c.max_reference_lead_m,"max_reference_lead_m"},{c.max_joint_lead_rad,"max_joint_lead_rad"},
        {c.max_joint_excursion_rad,"max_joint_excursion_rad"},{c.max_joint_speed_rad_s,"max_joint_speed_rad_s"},{c.max_joint_accel_rad_s2,"max_joint_accel_rad_s2"},
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
        c.grasp_xy_accept_m < c.xy_tolerance_m || c.freeze_height_m <= c.commit_height_m ||
        c.commit_height_m <= c.grasp_height_margin_m + c.z_tolerance_m ||
        c.grasp_height_margin_m <= c.solver_position_tolerance_m ||
        c.grasp_z_accept_above_m < c.grasp_height_margin_m + c.z_tolerance_m ||
        c.hard_reference_lead_m <= c.max_reference_lead_m ||
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
        case Phase::Commit: return "commit"; case Phase::Settling: return "settling"; case Phase::Done: return "done";
        case Phase::Failed: return "failed";
    }
    return "invalid";
}

double braking_speed(double d, double a, double delay) {
    if (!(d > 0 && a > 0 && delay >= 0)) return 0;
    // Algebraically equivalent to sqrt((a*t)^2+2*a*d)-a*t; avoids cancellation.
    return 2 * a * d / (std::sqrt(a*a*delay*delay + 2*a*d) + a*delay);
}

void Controller::reset(const Config &c, const State &s, double z, double speed,
                       const Vec2 &nominal_xy, bool grasp) {
    cfg_ = c; initial_xy_ = nominal_xy; goal_xy_ = nominal_xy;
    reference_=s.position; grasp_=grasp; relaxed_grasp_=false;
    acquire_floor_z_=std::max(z+c.commit_height_m,s.position.z()-c.acquire_descent_m);
    precision_floor_since_=finish_since_=-1; close_confirmations_=0;
    velocity_ = s.velocity.head<2>(); vz_ = s.velocity.z();
    start_time_ = s.time_s; final_z_ = z; descent_speed_ = speed;
    have_goal_ = false; last_capture_ = -1; last_frame_ = 0; confirmations_ = 0;
    commit_start_=-1;
    phase_ = Phase::Acquiring; failure_.clear();
}

bool Controller::observe(const PixelObservation &o, const State &s, const Mapping &m,
                         double now, std::string &reason) {
    if (phase_ == Phase::Commit || phase_ == Phase::Settling || phase_ == Phase::Done || phase_ == Phase::Failed) { reason = "session_not_accepting"; return false; }
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
    if(have_goal_ && o.capture_s-last_capture_>cfg_.max_observation_age_s) { confirmations_=0; close_confirmations_=0; }
    goal_xy_ = have_goal_ ? Vec2(goal_xy_ + cfg_.goal_filter_alpha * (candidate - goal_xy_)) : candidate;
    have_goal_ = true; last_capture_ = o.capture_s; last_frame_ = o.frame;
    last_clock_uncertainty_ = o.clock_uncertainty_s;
    // Visual evidence is independent of noisy finite-difference motor speed.
    const double hysteresis=1.5*cfg_.xy_tolerance_m;
    if(m.remaining_m.norm()<=cfg_.xy_tolerance_m) ++confirmations_;
    else if(m.remaining_m.norm()>hysteresis) confirmations_=0;
    const double accept=grasp_?cfg_.grasp_xy_accept_m:cfg_.xy_tolerance_m;
    close_confirmations_=m.remaining_m.norm()<=accept?close_confirmations_+1:0;
    reason = "accepted"; return true;
}

void Controller::fail(const std::string &reason) {
    if(phase_==Phase::Failed) return; // retain the initiating fault, not subsequent peer/timeout symptoms
    phase_ = Phase::Failed; failure_ = reason;
}

Output Controller::tick(const State &s, double dt) {
    Output out; out.age_s=have_goal_?s.time_s-last_capture_:-1;
    out.confirmations=confirmations_;
    if(!std::isfinite(dt) || dt<=0 || dt>cfg_.max_tick_gap_s ||
       !s.position.allFinite() || !s.velocity.allFinite()) fail("invalid_state_or_tick_gap");
    if(s.time_s-start_time_>cfg_.task_timeout_s) fail("task_timeout");
    const bool frozen=phase_==Phase::Commit || phase_==Phase::Settling;
    if(frozen && s.time_s-commit_start_>cfg_.commit_timeout_s) fail("commit_timeout");
    if(!frozen && phase_!=Phase::Done && phase_!=Phase::Failed &&
       s.time_s-(have_goal_?last_capture_:start_time_)>cfg_.loss_timeout_s)
        fail(have_goal_?"target_lost_timeout":"no_valid_observation_timeout");
    // The nominal visual feature plane remains C. This is only a motion clearance.
    const double stop_z=final_z_+(grasp_?cfg_.grasp_height_margin_m:0.0);
    const double finish_speed=grasp_?cfg_.finish_speed_m_s:cfg_.settle_speed_m_s;
    const double accept=grasp_?cfg_.grasp_xy_accept_m:cfg_.xy_tolerance_m;
    out.finish_xy_tolerance_m=accept;
    out.remaining_m=goal_xy_-s.position.head<2>();
    const double distance=out.remaining_m.norm();
    const bool fresh=have_goal_ && out.age_s>=0 &&
        out.age_s+last_clock_uncertainty_<=cfg_.max_observation_age_s;
    const double usable_dt=(std::isfinite(dt) && dt>0)?std::min(dt,cfg_.max_tick_gap_s):cfg_.period_s;
    Vec2 target_v=Vec2::Zero(); double target_z=0;
    double floor_z=frozen?stop_z:(have_goal_?final_z_+cfg_.commit_height_m:acquire_floor_z_);
    if(phase_!=Phase::Failed && phase_!=Phase::Done) {
        if(s.position.z()<final_z_-(grasp_?0.0:cfg_.z_tolerance_m)) fail("z_below_nominal_C");
        if(phase_!=Phase::Failed) {
            const bool near_guard=reference_.z()<=final_z_+cfg_.commit_height_m+cfg_.z_tolerance_m;
            if(near_guard && precision_floor_since_<0) precision_floor_since_=s.time_s;
            const bool precise=fresh && confirmations_>=cfg_.stable_frames &&
                distance<=(grasp_?1.5:1.0)*cfg_.xy_tolerance_m && s.velocity.head<2>().norm()<=finish_speed;
            const bool usable_grasp=grasp_ && fresh && close_confirmations_>=cfg_.stable_frames &&
                distance<=accept && near_guard && precision_floor_since_>=0 &&
                s.time_s-precision_floor_since_>=cfg_.grasp_relax_after_s &&
                s.velocity.head<2>().norm()<=cfg_.finish_speed_m_s;
            if(!frozen && s.position.z()-final_z_<=cfg_.freeze_height_m && (precise || usable_grasp)) {
                phase_=Phase::Commit; commit_start_=s.time_s; relaxed_grasp_=!precise;
                floor_z=stop_z;
            }
            const bool committed=phase_==Phase::Commit || phase_==Phase::Settling;
            const bool can_advance=committed || fresh || !have_goal_;
            if(can_advance) {
                const Vec2 reference_remaining=goal_xy_-reference_.head<2>();
                const Vec2 pd=cfg_.kp*out.remaining_m-cfg_.kd*s.velocity.head<2>();
                const double budget=std::max(0.0,distance-.25*cfg_.xy_tolerance_m-cfg_.model_uncertainty_m);
                out.brake_speed=braking_speed(budget,cfg_.max_xy_accel_m_s2,
                                              cfg_.brake_delay_s+last_clock_uncertainty_);
                const double cap=std::min({cfg_.max_xy_speed_m_s,out.brake_speed,
                    braking_speed(reference_remaining.norm(),cfg_.max_xy_accel_m_s2,cfg_.brake_delay_s)});
                if(reference_remaining.norm()>1e-9)
                    target_v=reference_remaining.normalized()*std::clamp(
                        pd.dot(reference_remaining.normalized()),0.0,cap);
                if(distance<=cfg_.xy_tolerance_m) target_v.setZero();
                const double travel=std::max(0.0,reference_.z()-floor_z);
                double speed=descent_speed_;
                // Freezing XY early must not force the whole remaining stroke to creep.
                // Start the final speed transition early enough for bounded deceleration.
                const double transition=cfg_.commit_height_m+descent_speed_*cfg_.brake_delay_s+
                    std::max(0.0,descent_speed_*descent_speed_-cfg_.commit_speed_m_s*cfg_.commit_speed_m_s)/
                    (2*cfg_.max_z_accel_m_s2);
                if(committed && reference_.z()-final_z_<=transition)
                    speed=std::min(speed,cfg_.commit_speed_m_s);
                if(!committed && have_goal_) {
                    // Time to remove the observed XY residual, including saturation and observation age.
                    const double ratio=std::max(1.0,distance/cfg_.xy_tolerance_m);
                    out.alignment_time_s=std::max(std::max(0.0,distance-cfg_.xy_tolerance_m)/cfg_.max_xy_speed_m_s,
                        std::log(ratio)*(1+cfg_.kd)/cfg_.kp)+s.velocity.head<2>().norm()/cfg_.max_xy_accel_m_s2+
                        std::max(0.0,out.age_s)+cfg_.alignment_time_margin_s;
                    out.available_time_s=travel/std::max(descent_speed_,1e-6);
                    if(!precise) speed=std::min(speed,travel/std::max(out.alignment_time_s,cfg_.period_s));
                }
                const double lead=(reference_-s.position).norm();
                const double governor=std::clamp((cfg_.hard_reference_lead_m-lead)/
                    (cfg_.hard_reference_lead_m-cfg_.max_reference_lead_m),0.0,1.0);
                target_v*=governor;
                target_z=-std::min(speed,braking_speed(travel,cfg_.max_z_accel_m_s2,cfg_.brake_delay_s))*governor;
                out.descent_scale=descent_speed_>0?-target_z/descent_speed_:0;
                if(!committed) phase_=have_goal_?Phase::Descending:Phase::Acquiring;
            } else { phase_=Phase::Hold; out.reason="waiting_fresh_vision"; }
        }
    }
    if(phase_==Phase::Failed || phase_==Phase::Done) { target_v.setZero(); target_z=0; }
    Vec2 change=target_v-velocity_;
    if(change.norm()>cfg_.max_xy_accel_m_s2*usable_dt) change*=cfg_.max_xy_accel_m_s2*usable_dt/change.norm();
    velocity_+=change;
    vz_+=std::clamp(target_z-vz_,-cfg_.max_z_accel_m_s2*usable_dt,cfg_.max_z_accel_m_s2*usable_dt);
    Vec3 increment(velocity_.x()*usable_dt,velocity_.y()*usable_dt,vz_*usable_dt);
    const Vec2 reference_remaining=goal_xy_-reference_.head<2>();
    if(reference_remaining.norm()<1e-9) increment.head<2>().setZero();
    else if(increment.head<2>().dot(reference_remaining)>reference_remaining.squaredNorm() &&
       increment.head<2>().norm()>0)
        increment.head<2>()*=reference_remaining.squaredNorm()/increment.head<2>().dot(reference_remaining);
    // Constrain reference progression, never snap the reference to the measured state.
    // This also prevents a nominally stopped motor from chasing a point below C.
    if(reference_.z()+increment.z()<floor_z) increment.z()=std::min(0.0,floor_z-reference_.z());
    const Vec3 lead=reference_-s.position;
    if((lead+increment).norm()>cfg_.hard_reference_lead_m) {
        double scale=0;
        if(lead.norm()<=cfg_.hard_reference_lead_m && increment.squaredNorm()>1e-18) {
            const double b=lead.dot(increment);
            scale=std::clamp((-b+std::sqrt(std::max(0.0,b*b-increment.squaredNorm()*
                (lead.squaredNorm()-cfg_.hard_reference_lead_m*cfg_.hard_reference_lead_m))))/increment.squaredNorm(),0.0,1.0);
        } else if((lead+increment).norm()<lead.norm()) scale=1;
        increment*=scale;
    }
    reference_+=increment;
    velocity_=increment.head<2>()/usable_dt; vz_=increment.z()/usable_dt;
    if(phase_==Phase::Commit || phase_==Phase::Settling) {
        const bool reference_stopped=reference_.z()<=stop_z+1e-5 &&
            std::abs(vz_)<1e-4 && velocity_.norm()<1e-4;
        if(reference_stopped) phase_=Phase::Settling;
        const bool ready=reference_stopped && distance<=accept &&
            s.position.z()>=final_z_-(grasp_?0.0:cfg_.z_tolerance_m) &&
            s.position.z()<=final_z_+(grasp_?cfg_.grasp_z_accept_above_m:cfg_.z_tolerance_m) &&
            std::abs(s.velocity.z())<=finish_speed && s.velocity.head<2>().norm()<=finish_speed;
        if(ready) { if(finish_since_<0) finish_since_=s.time_s; }
        else finish_since_=-1;
        if(finish_since_>=0 && s.time_s-finish_since_>=cfg_.finish_stable_s) {
            phase_=Phase::Done; velocity_.setZero(); vz_=0;
        }
    }
    out.reference=reference_; out.reference_floor_z=floor_z; out.relaxed_grasp=relaxed_grasp_;
    out.velocity_xy=velocity_; out.velocity_z=vz_; out.phase=phase_;
    if(phase_==Phase::Failed) out.reason=failure_;
    else if(phase_==Phase::Commit) out.reason=relaxed_grasp_?"target_frozen_grasp_tolerance":"target_frozen_precise";
    else if(phase_==Phase::Settling) out.reason="holding_legal_reference";
    else if(phase_==Phase::Done) out.reason=relaxed_grasp_?"ready_grasp_tolerance":"ready_precise";
    return out;
}
} // namespace wrist_servo
