#include "internal/arm_internal.h"
#include "t170/motion/pose_solver.h"
#include "t170/motion/joint_motion.h"
#include "t170/common/paths.h"
#include "t170/telemetry/dispatch_metrics.h"
#include "t170/common/servo_trace.h"
#include "t170/skills/legacy/profiles.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <filesystem>
#include <algorithm>
#include <chrono>
#include <thread>
#include <cctype>
#include <limits>
#include <stdexcept>
using namespace Eigen;
using namespace std;
using namespace t170::motion::detail;
void save_arm_last_commanded_position(int hand, const int motor_cmd[7])
{
    if (hand < 0 || hand > 1)
        return;
    for (int i = 0; i < 7; ++i)
        g_arm_last_cmd_cnt[hand][i] = motor_cmd[i];
    g_arm_last_cmd_valid[hand] = true;
}

namespace t170::motion::detail {
wrist_servo::ConnectorLimits retract_connector_limits()
{
    const auto &c = g_move_cfg.head_grasp;
    return {c.retract_connector_speed_rad_s, c.retract_connector_accel_rad_s2,
            c.retract_connector_jerk_rad_s3, c.retract_connector_max_s};
}
}


namespace t170::motion::detail {
std::function<bool(const Eigen::RowVectorXd &)> lift_connector_validity(
    Robot_Arm &arm, const Eigen::RowVectorXd &held)
{
    const Eigen::Vector3d start=arm_tcp_from_joints(arm,held).head<3>().transpose();
    return [&arm,start](const Eigen::RowVectorXd &q) {
        const Eigen::Vector3d p=arm_tcp_from_joints(arm,q).head<3>().transpose();
        // A connector near a loaded tray must neither dip below the held TCP
        // nor turn a small IK discrepancy into a large sideways motion.
        return p.allFinite() && p.z()>=start.z()-.002 &&
               (p.head<2>()-start.head<2>()).norm()<=.008;
    };
}
}


namespace {
Eigen::MatrixXd bridge_from_hold(const Eigen::MatrixXd &src, const wrist_servo::Joints &held,
                                 double dt, int hand, Robot_Arm *arm)
{
    const auto valid=arm ? lift_connector_validity(*arm,held) :
                          std::function<bool(const Eigen::RowVectorXd &)>{};
    const auto out = wrist_servo::connect_held_start(src, held, dt, retract_connector_limits(),valid);
    const double gap = src.rows() && src.cols() == 7
                           ? (src.row(0) - held).cwiseAbs().maxCoeff() : INFINITY;
    if (out.rows() == 0 || out.rows() != src.rows())
        std::cout << "[retract] hand=" << hand << " start_gap_deg=" << gap * 180 / M_PI
                  << " connector_ms=" << std::max<Eigen::Index>(0, out.rows()-src.rows())*dt*1000
                  << " result=" << (out.rows() ? "quintic" : "invalid_start") << std::endl;
    return out;
}
}


namespace t170::motion::detail {
int execute_arm_trajectory(const MatrixXd &traj_in, int hand, double dt,
                           Robot_Arm *retract_arm,
                           const std::function<void(const wrist_servo::Joints &)> &observe_progress)
{
    if (hand == 1 && right_arm_motors_locked()) return 0;
    if (traj_in.rows()<2 || traj_in.cols()!=7 || !traj_in.allFinite() || !std::isfinite(dt) || dt<=0)
        return -16;
    MatrixXd traj = traj_in;
    wrist_servo::Joints held;
    const bool had_hold = wrist_servo::peek_final_hold(hand, held);
    const bool diagnostic = had_hold || retract_arm != nullptr;
    // Plan the small start connector while the old reference is still being held.
    // A large/stale discrepancy is not sent as a one-tick jump.
    if (had_hold) {
        traj = bridge_from_hold(traj_in, held, dt, hand, retract_arm);
        if (traj.rows()<2) return -16;
    }
    wrist_servo::ObservationContext context;
    if (diagnostic) wrist_servo::observation_context(hand, context);
    wrist_servo::Joints initial_actual;
    const bool initial_feedback = diagnostic && wrist_servo::read_joint_feedback(hand, initial_actual);
    const double release_start = wrist_servo::monotonic_seconds();
    wrist_servo::Joints transferred;
    const bool took_hold = wrist_servo::take_final_hold(hand, transferred);
    const double release_ms = (wrist_servo::monotonic_seconds()-release_start)*1000;
    if (took_hold && (!had_hold || (transferred-held).cwiseAbs().maxCoeff()>1e-9)) {
        traj = bridge_from_hold(traj_in, transferred, dt, hand, retract_arm);
        if (traj.rows()<2) return -16;
    }
    auto trace = [&](const char *event, const char *reason, int row, double cycle_ms,
                     const Eigen::RowVectorXd &v, const Eigen::RowVectorXd &a,
                     const Eigen::RowVectorXd &jerk, const wrist_servo::Joints *actual) {
        if (!diagnostic) return;
        wrist_servo::TraceRow r;
        r.time_s=wrist_servo::monotonic_seconds(); r.session=context.session;
        r.hand=hand; r.task_tag=context.task_tag; r.target_class=context.target_class;
        r.final_z=context.final_z; r.event=event; r.reason=reason;
        r.trajectory_row=row; r.dt_ms=cycle_ms; r.hold_release_wait_ms=release_ms;
        r.commanded=traj.row(row);
        r.command_step_rad=row>0 ? (traj.row(row)-traj.row(row-1)).cwiseAbs().maxCoeff() :
            (took_hold ? (traj.row(0)-transferred).cwiseAbs().maxCoeff() : 0);
        r.command_speed_rad_s=v.cwiseAbs().maxCoeff();
        r.command_accel_rad_s2=a.cwiseAbs().maxCoeff(); r.command_jerk_rad_s3=jerk.cwiseAbs().maxCoeff();
        const double nan=std::numeric_limits<double>::quiet_NaN();
        r.measured.joints.setConstant(nan); r.measured.position.setConstant(nan);
        r.measured.velocity.setConstant(nan); r.reference.setConstant(nan);
        r.joint_feedback_valid=actual!=nullptr;
        if(actual) {
            r.measured.joints=*actual;
            if(retract_arm) r.measured.position=arm_tcp_from_joints(*retract_arm,*actual).head<3>().transpose();
        }
        wrist_servo::trace_writer().push(std::move(r));
    };
    Eigen::RowVectorXd previous_v=Eigen::RowVectorXd::Zero(7), previous_a=previous_v;
    trace("retract_start",took_hold ? "exclusive_hold_to_trajectory" : "trajectory_without_hold",
          0,0,previous_v,previous_a,previous_a,initial_feedback ? &initial_actual : nullptr);
    double last_tick=0;
    double peak_step=0,peak_v=0,peak_a=0,peak_j=0;
    for (int i=0; i<traj.rows(); ++i) {
        if (hardware_abort_requested() || can_io_faulted()) {
            trace("retract_abort",hardware_abort_requested() ? "STOP" : "CAN_fault",
                  i,0,previous_v,previous_a,previous_a,nullptr);
            return hardware_abort_requested() ? -4 : -2;
        }
        const auto cycle_start=std::chrono::steady_clock::now();
        const double tick=wrist_servo::monotonic_seconds();
        const double cycle_ms=last_tick>0 ? (tick-last_tick)*1000 : 0;
        last_tick=tick;
        int motor_cmd[7] = {0};
        for (int j=0;j<7;++j) motor_cmd[j]=rad_to_encoder_cnt(traj(i,j));
        set_motor_position(motor_cmd,hand);
        // Only staged retreats request feedback, at 20 Hz. Planning/photography never
        // runs in this callback; it only publishes measured clearance to a waiting peer.
        if(observe_progress && i%10==0) {
            wrist_servo::Joints actual;
            if(wrist_servo::read_joint_feedback(hand,actual)) observe_progress(actual);
        }
        const Eigen::RowVectorXd v=i>0 ? Eigen::RowVectorXd((traj.row(i)-traj.row(i-1))/dt)
                                                     : Eigen::RowVectorXd::Zero(7);
        const Eigen::RowVectorXd a=(v-previous_v)/dt, jerk=(a-previous_a)/dt;
        peak_v=std::max(peak_v,v.cwiseAbs().maxCoeff()); peak_a=std::max(peak_a,a.cwiseAbs().maxCoeff());
        peak_j=std::max(peak_j,jerk.cwiseAbs().maxCoeff());
        if(i>0) peak_step=std::max(peak_step,(traj.row(i)-traj.row(i-1)).cwiseAbs().maxCoeff());
        // Capture every command in the first 250 ms, then 10 Hz and the end.
        // No synchronous file writes; only staged retreats opt into 20Hz feedback above.
        if(i<=50 || i%20==0 || i+1==traj.rows())
            trace("retract_sample","command_stream",i,cycle_ms,v,a,jerk,nullptr);
        previous_v=v; previous_a=a;
        std::this_thread::sleep_until(cycle_start+std::chrono::duration<double>(dt));
    }
    int last_cmd[7] = {0};
    for(int j=0;j<7;++j) last_cmd[j]=rad_to_encoder_cnt(traj(traj.rows()-1,j));
    save_arm_last_commanded_position(hand,last_cmd);
    wrist_servo::Joints final_actual;
    const bool final_feedback=diagnostic && wrist_servo::read_joint_feedback(hand,final_actual);
    trace("retract_end","playback_complete",static_cast<int>(traj.rows()-1),0,
          previous_v,previous_a,Eigen::RowVectorXd::Zero(7),final_feedback ? &final_actual : nullptr);
    if(diagnostic)
        std::cout << "[retract] hand=" << hand << " rows=" << traj.rows()
                  << " nominal_s=" << (traj.rows()-1)*dt << " hold_release_ms=" << release_ms
                  << " peak_step_deg=" << peak_step*180/M_PI << " peak_v_rad_s=" << peak_v
                  << " peak_a_rad_s2=" << peak_a << " peak_j_rad_s3=" << peak_j << std::endl;
    return 0;
}
}


namespace wrist_servo {
void send_joint_reference(int hand,const Joints &q) {
    if((hand!=0 && hand!=1) || !q.allFinite()) throw std::runtime_error("invalid_joint_reference");
    int counts[7];
    for(int j=0;j<7;++j) counts[j]=rad_to_encoder_cnt(q(j));
    uint32_t ids[7];
    const auto *source=hand==1?kRightArmCanIds:kLeftArmCanIds;
    std::copy(source,source+7,ids);
    if(socketcan_sendcommand(0,7,ids,30,counts)!=1) throw std::runtime_error("joint_command_write_failed");
    save_arm_last_commanded_position(hand,counts);
}
}

