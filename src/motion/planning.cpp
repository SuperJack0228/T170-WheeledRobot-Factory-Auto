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
namespace {
Eigen::MatrixXd make_linear_joint_traj(
    const Eigen::MatrixXd &q_start,
    const Eigen::MatrixXd &q_end,
    double dt)
{
    const int dof = static_cast<int>(std::min({q_start.cols(), q_end.cols(), static_cast<Eigen::Index>(7)}));
    double max_dq = 0.0;
    for (int j = 0; j < dof; ++j)
        max_dq = std::max(max_dq, std::abs(q_end(0, j) - q_start(0, j)));
    constexpr double kDegPerSec = 30.0;
    const double t_s = std::max(0.4, (max_dq * 180.0 / M_PI) / kDegPerSec);
    const int n = std::max(2, static_cast<int>(std::ceil(t_s / dt)) + 1);
    Eigen::MatrixXd traj(n, dof);
    for (int i = 0; i < n; ++i)
    {
        const double s = static_cast<double>(i) / static_cast<double>(n - 1);
        const double e = 0.5 * (1.0 - std::cos(M_PI * s));
        for (int j = 0; j < dof; ++j)
            traj(i, j) = q_start(0, j) + e * (q_end(0, j) - q_start(0, j));
    }
    return traj;
}
}


void clamp_cart_goal_z(Matrix<double, 1, 6> &pos, const char *where)
{
    // 与 grasp_valid.z_min 一致。旧硬编码 -0.33/-0.30 会把下压目标抬回去，空中合爪。
    const double z_min = g_move_cfg.grasp_valid.z_min;
    if (pos(2) >= z_min)
        return;

    const char *loc = (where != nullptr && where[0] != '\0') ? where : "unknown";
    std::cerr << "\033[31m[arm][WARN] goal z=" << std::fixed << std::setprecision(4) << pos(2)
              << " 低于 grasp_valid.z_min=" << z_min << "，位置: " << loc << "，钳位为 " << z_min
              << "\033[0m" << std::endl;
    pos(2) = z_min;
}

void arm_line_move_set_debug_stage(const char *stage)
{
    std::lock_guard<std::mutex> lock(g_arm_line_move_stage_mu);
    g_arm_line_move_debug_stage = stage;
}

namespace {
constexpr const char *kLineTrajFailDebugSep =
    "================================================================================\n";
}


namespace {
std::mutex g_line_traj_fail_log_mu;
}


namespace {
const char *arm_hand_label(int hand)
{
    return hand == 1 ? "right" : "left";
}
}


namespace {
std::string pose6_to_csv(const Matrix<double, 1, 6> &p)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    for (int i = 0; i < 6; ++i)
    {
        if (i > 0)
            oss << ',';
        oss << p(i);
    }
    return oss.str();
}
}


void append_line_trajectory_fail_debug(
    const char *hand_label,
    int ret,
    const Matrix<double, 1, 6> &goal,
    const char *stage,
    double cart_linear_velocity)
{
    const std::filesystem::path file_path =
        project_root_dir() + "/picture_debug/line_trajectory_fail.txt";
    std::lock_guard<std::mutex> lock(g_line_traj_fail_log_mu);

    std::error_code ec;
    std::filesystem::create_directories(file_path.parent_path(), ec);
    if (ec)
    {
        std::cerr << "[line_trajectory_fail] 创建目录失败: " << ec.message() << std::endl;
        return;
    }

    std::ofstream out(file_path, std::ios::app);
    if (!out)
    {
        std::cerr << "[line_trajectory_fail] 无法写入 " << file_path.string() << std::endl;
        return;
    }

    const auto now = std::chrono::system_clock::now();
    const std::time_t sec = std::chrono::system_clock::to_time_t(now);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) %
                    1000;
    std::tm tm_local{};
    localtime_r(&sec, &tm_local);

    out << kLineTrajFailDebugSep;
    out << "[line_trajectory_fail] hand=" << (hand_label ? hand_label : "?")
        << " ret=" << ret << "\n";
    out << "stage=" << (stage && stage[0] != '\0' ? stage : "(未标注)") << "\n";
    out << "time=" << std::put_time(&tm_local, "%Y-%m-%d %H:%M:%S") << '.'
        << std::setw(3) << std::setfill('0') << ms.count() << "\n";
    out << "format: x,y,z(m) rx,ry,rz(rad)\n";
    out << "goal=" << pose6_to_csv(goal) << "\n";
    out << "cart_vel=" << std::fixed << std::setprecision(3) << cart_linear_velocity << "\n";
    out << kLineTrajFailDebugSep << "\n";
    out.flush();

    std::cerr << "[line_trajectory_fail] 已写入 " << file_path.string()
              << " hand=" << (hand_label ? hand_label : "?") << " ret=" << ret << std::endl;
}

int arm_line_move(
    Robot_Arm &arm, Matrix<double, 1, 6> &pos, double cart_linear_velocity, bool snap_reach)
{
    if (arm_hand_side(arm) == 1 && right_arm_motors_locked())
        return 0;
    clamp_cart_goal_z(pos, "arm_line_move(直线运动下发前)");
    const int hand = arm_hand_side(arm);
    Eigen::MatrixXd q_start = arm_read_motor_joints(hand);
    if (hardware_abort_requested())
        return -4;
    if (can_io_faulted())
        return -2;

    arm.Cart_Linear_Velocity = cart_linear_velocity;

    Eigen::MatrixXd traj;
    const int ret = arm.Line_Trajectory(q_start, pos, traj);
    if (ret != 0)
    {
        const char *stage = nullptr;
        {
            std::lock_guard<std::mutex> lock(g_arm_line_move_stage_mu);
            stage = g_arm_line_move_debug_stage;
        }
        append_line_trajectory_fail_debug(arm_hand_label(hand), ret, pos, stage, cart_linear_velocity);
        return ret;
    }
    const int exec = execute_arm_trajectory(traj, hand, k_arm_traj_dt);
    if (exec == 0)
        record_arm_cartesian_arrival(arm, pos, nullptr);
    return exec;
}

int arm_line_move_settle(Robot_Arm &arm, Matrix<double, 1, 6> &pos, double cart_linear_velocity, int settle_ms)
{
    if (arm_hand_side(arm) == 1 && right_arm_motors_locked())
        return 0;
    clamp_cart_goal_z(pos, "arm_line_move_settle(直线运动下发前)");
    const int hand = arm_hand_side(arm);
    Eigen::MatrixXd q_start = arm_read_motor_joints(hand);
    if (hardware_abort_requested())
        return -4;
    if (can_io_faulted())
        return -2;

    arm.Cart_Linear_Velocity = cart_linear_velocity;

    Eigen::MatrixXd traj;
    const int ret = arm.Line_Trajectory(q_start, pos, traj);
    if (ret != 0)
    {
        const char *stage = nullptr;
        {
            std::lock_guard<std::mutex> lock(g_arm_line_move_stage_mu);
            stage = g_arm_line_move_debug_stage;
        }
        append_line_trajectory_fail_debug(arm_hand_label(hand), ret, pos, stage, cart_linear_velocity);
        return ret;
    }
    const int exec = execute_arm_trajectory(traj, hand, k_arm_traj_dt);
    if (exec != 0)
        return exec;
    if (settle_ms > 0)
    {
        std::cout << "[arm] 轨迹已下发完，再等 " << settle_ms << " ms 后回读关节角\n" << std::flush;
        hardware_abort_sleep_ms(settle_ms);
    }
    if (hardware_abort_requested())
        return -4;

    const int n = static_cast<int>(std::min(traj.cols(), static_cast<Eigen::Index>(7)));
    const Eigen::Matrix<double, 1, 7> q_act = arm_get_joint_pos(arm);
    std::cout << std::fixed << std::setprecision(3)
              << "[arm] " << (hand == 1 ? "右手" : "左手")
              << " 关节角(度)  理论  实际  差\n";
    for (int j = 0; j < n; ++j)
    {
        const double theory_deg = traj(traj.rows() - 1, j) * 180.0 / M_PI;
        const double actual_deg = q_act(j) * 180.0 / M_PI;
        std::cout << "  J" << (j + 1)
                  << "  " << theory_deg
                  << "  " << actual_deg
                  << "  " << (actual_deg - theory_deg) << "\n";
    }
    const Eigen::Matrix<double, 1, 6> tcp = arm_get_tcp_pos(arm);
    std::cout << std::setprecision(4)
              << "[arm] 等稳后 TCP 目标=(" << pos(0) << "," << pos(1) << "," << pos(2)
              << ") 实际=(" << tcp(0) << "," << tcp(1) << "," << tcp(2)
              << ") 差=(" << (tcp(0) - pos(0)) << "," << (tcp(1) - pos(1)) << ","
              << (tcp(2) - pos(2)) << ")\n";
    record_arm_cartesian_arrival(arm, pos, nullptr);
    return 0;
}

int arm_line_move_hold_redundant(Robot_Arm &arm, Matrix<double, 1, 6> &pos, double cart_linear_velocity)
{
    if (arm_hand_side(arm) == 1 && right_arm_motors_locked())
        return 0;
    clamp_cart_goal_z(pos, "arm_line_move_hold_redundant(直线运动下发前)");
    const int hand = arm_hand_side(arm);
    Eigen::MatrixXd q_start = arm_read_motor_joints(hand);
    if (hardware_abort_requested())
        return -4;
    if (can_io_faulted())
        return -2;

    arm.Cart_Linear_Velocity = cart_linear_velocity;

    Eigen::MatrixXd traj;
    const double j2 = (q_start.cols() >= 2) ? q_start(0, 1) : 0.0;
    const int ret = arm.Line_Trajectory(q_start, pos, traj, j2);
    if (ret != 0)
    {
        const char *stage = nullptr;
        {
            std::lock_guard<std::mutex> lock(g_arm_line_move_stage_mu);
            stage = g_arm_line_move_debug_stage;
        }
        append_line_trajectory_fail_debug(arm_hand_label(hand), ret, pos, stage, cart_linear_velocity);
        return ret;
    }
    const int exec = execute_arm_trajectory(traj, hand, k_arm_traj_dt);
    if (exec == 0)
        record_arm_cartesian_arrival(arm, pos, nullptr);
    return exec;
}



ArmLineMoveResult arm_dual_line_move(
    Robot_Arm &arm_r,
    Matrix<double, 1, 6> &pos_r,
    Robot_Arm &arm_l,
    Matrix<double, 1, 6> &pos_l,
    double cart_linear_velocity,
    bool snap_reach)
{
    ArmLineMoveResult out;
    std::thread th_r([&]() { out.ret_r = arm_line_move(arm_r, pos_r, cart_linear_velocity, snap_reach); });
    std::thread th_l([&]() { out.ret_l = arm_line_move(arm_l, pos_l, cart_linear_velocity, snap_reach); });
    th_r.join();
    th_l.join();
    return out;
}

ArmLineMoveResult arm_dual_line_move_selective(
    Robot_Arm &arm_r,
    Matrix<double, 1, 6> &pos_r,
    const bool move_r,
    Robot_Arm &arm_l,
    Matrix<double, 1, 6> &pos_l,
    const bool move_l,
    const double cart_linear_velocity,
    const bool snap_reach)
{
    if (move_r && move_l)
        return arm_dual_line_move(arm_r, pos_r, arm_l, pos_l, cart_linear_velocity, snap_reach);

    ArmLineMoveResult out;
    if (move_r)
        out.ret_r = arm_line_move(arm_r, pos_r, cart_linear_velocity, snap_reach);
    if (move_l)
        out.ret_l = arm_line_move(arm_l, pos_l, cart_linear_velocity, snap_reach);
    return out;
}

namespace {
double traj_joint_step(const Eigen::MatrixXd &traj, int i)
{
    return (traj.row(i) - traj.row(i - 1)).norm();
}
}


namespace {
double traj_peak_step(const Eigen::MatrixXd &traj)
{
    double peak = 0.0;
    for (int i = 1; i < traj.rows(); ++i)
        peak = std::max(peak, traj_joint_step(traj, i));
    return peak;
}
}


namespace {
int last_cruise_row(const Eigen::MatrixXd &traj)
{
    const int n = static_cast<int>(traj.rows());
    if (n < 3)
        return n - 1;
    const double th = std::max(1e-6, 0.55 * traj_peak_step(traj));
    int last = 1;
    for (int i = 1; i < n; ++i)
    {
        if (traj_joint_step(traj, i) >= th)
            last = i;
    }
    return last;
}
}


namespace {
int first_cruise_row(const Eigen::MatrixXd &traj)
{
    const int n = static_cast<int>(traj.rows());
    if (n < 3)
        return 0;
    const double th = std::max(1e-6, 0.55 * traj_peak_step(traj));
    for (int i = 1; i < n; ++i)
    {
        if (traj_joint_step(traj, i) >= th)
            return i;
    }
    return n - 1;
}
}


namespace t170::motion::detail {
int plan_arm_line_from(
    Robot_Arm &arm,
    Eigen::Matrix<double, 1, 6> goal,
    double vel_m_s,
    const Eigen::MatrixXd &q_start,
    Eigen::MatrixXd &traj,
    bool lock_redundant)
{
    if (q_start.rows() != 1 || q_start.cols() < 6 || !q_start.allFinite() || !goal.allFinite())
        return -10;
    if (!(vel_m_s > 0.0))
        return -10;
    clamp_cart_goal_z(goal, "两段直线");
    arm.Cart_Linear_Velocity = vel_m_s;
    int ret = -16;
    if (lock_redundant && q_start.cols() >= 2)
        ret = arm.Line_Trajectory(q_start, goal, traj, q_start(0, 1));
    if (!lock_redundant || ret != 0 || traj.rows() < 2)
    {
        if (lock_redundant && ret != 0)
            std::cerr << "[retract] 竖直抬升锁冗余关节失败，改为自由冗余\n";
        ret = arm.Line_Trajectory(q_start, goal, traj);
    }
    if (ret != 0 || traj.rows() < 2 || traj.cols()!=q_start.cols()) {
        const std::string reason="SDK_code="+std::to_string(ret)+" rows="+std::to_string(traj.rows())+
            " columns="+std::to_string(traj.cols());
        wrist_servo::task_event(arm_hand_side(arm),"retract_sdk_failed",reason.c_str());
        append_line_trajectory_fail_debug(arm_hand_label(arm_hand_side(arm)),ret!=0?ret:-16,
            goal,"独立直线SDK规划",vel_m_s);
        return ret != 0 ? ret : -16;
    }
    if (lock_redundant) {
        // Never disguise an SDK branch change by overwriting row zero alone.
        // Keep the real first rows and connect from the held reference at rest.
        Eigen::MatrixXd connected = wrist_servo::connect_held_start(
            traj, q_start.row(0), k_arm_traj_dt, retract_connector_limits(),
            lift_connector_validity(arm,q_start.row(0)));
        if (connected.rows()<2) {
            std::cerr << "[retract] locked SDK start not continuous, retry free redundancy\n";
            ret=arm.Line_Trajectory(q_start,goal,traj);
            if(ret!=0 || traj.rows()<2) return ret!=0 ? ret : -16;
            connected=wrist_servo::connect_held_start(
                traj,q_start.row(0),k_arm_traj_dt,retract_connector_limits(),
                lift_connector_validity(arm,q_start.row(0)));
        }
        if (connected.rows()<2) {
            wrist_servo::task_event(arm_hand_side(arm),"lift_connector_failed","SDK_start_does_not_join_held_reference");
            return -16;
        }
        traj=std::move(connected);
    }
    return 0;
}
}


namespace {
Eigen::MatrixXd concat_full_lines(const Eigen::MatrixXd &first, const Eigen::MatrixXd &second)
{
    if (first.rows() < 2 || second.rows() < 2 || first.cols() != second.cols())
        return Eigen::MatrixXd();
    const int n = static_cast<int>(first.rows() + second.rows() - 1);
    if (n <= 0 || n > 20000)
        return Eigen::MatrixXd();
    Eigen::MatrixXd out(n, first.cols());
    out.topRows(first.rows()) = first;
    out.bottomRows(second.rows() - 1) = second.bottomRows(second.rows() - 1);
    return out;
}
}


namespace {
Eigen::MatrixXd stitch_lift_then_return(Robot_Arm &arm, const Eigen::MatrixXd &first,
                                            const Eigen::MatrixXd &second)
{
    const int n1=static_cast<int>(first.rows()), n2=static_cast<int>(second.rows());
    if(n1<3 || n2<3 || first.cols()!=second.cols()) return {};
    // Round only the end of the lift, after at least 90% of the planned lift
    // samples. The validity check also constrains the actual TCP path below.
    const int ia=std::max(1,std::max((n1-1)*90/100,last_cruise_row(first)));
    const int ib=std::min(n2-2,std::min(std::max(1,(n2-1)*15/100),first_cruise_row(second)));
    const Eigen::Vector3d a=arm_tcp_from_joints(arm,first.row(ia)).head<3>().transpose();
    const Eigen::Vector3d corner=arm_tcp_from_joints(arm,first.row(n1-1)).head<3>().transpose();
    const Eigen::Vector3d b=arm_tcp_from_joints(arm,second.row(ib)).head<3>().transpose();
    const double start_z=arm_tcp_from_joints(arm,first.row(0))(2);
    const bool nearly_lifted=a.z()>=corner.z()-std::min(.01,std::max(0.0,corner.z()-start_z)*.1);
    auto distance_to_segment=[](const Eigen::Vector3d &p,const Eigen::Vector3d &u,const Eigen::Vector3d &v) {
        const Eigen::Vector3d d=v-u;
        const double t=d.squaredNorm()>1e-12 ? std::clamp((p-u).dot(d)/d.squaredNorm(),0.0,1.0) : 0.0;
        return (p-u-t*d).norm();
    };
    const auto valid=[&](const Eigen::RowVectorXd &q) {
        const Eigen::Vector3d p=arm_tcp_from_joints(arm,q).head<3>().transpose();
        return nearly_lifted && p.allFinite() && p.z()>=std::min(a.z(),b.z())-.002 &&
            std::min(distance_to_segment(p,a,corner),distance_to_segment(p,corner,b))<=.008;
    };
    const auto bridge=wrist_servo::connect_joint_boundaries(
        wrist_servo::trajectory_boundary(first,ia,k_arm_traj_dt,false),
        wrist_servo::trajectory_boundary(second,ib,k_arm_traj_dt,true),
        k_arm_traj_dt,retract_connector_limits(),valid);
    if(bridge.rows()>0) {
        const int tail=n2-ib-1;
        Eigen::MatrixXd out(ia+bridge.rows()+tail,first.cols());
        out.topRows(ia)=first.topRows(ia);
        out.middleRows(ia,bridge.rows())=bridge;
        out.bottomRows(tail)=second.bottomRows(tail);
        std::cout << "[retract] hand=" << arm_hand_side(arm) << " corner=quintic"
                  << " lift_join=" << ia << " return_join=" << ib
                  << " connector_ms=" << (bridge.rows()-1)*k_arm_traj_dt*1000 << std::endl;
        return out;
    }
    // No arbitrary straight interpolation and no sleep. Retain both SDK
    // acceleration/deceleration profiles if a short safe blend cannot be found.
    // Their shared endpoint is included once.
    const Eigen::MatrixXd aligned=wrist_servo::connect_held_start(
        second,first.row(n1-1),k_arm_traj_dt,retract_connector_limits(),
        lift_connector_validity(arm,first.row(n1-1)));
    if(aligned.rows()<2) return {};
    Eigen::MatrixXd out(n1+aligned.rows()-1,first.cols());
    out.topRows(n1)=first; out.bottomRows(aligned.rows()-1)=aligned.bottomRows(aligned.rows()-1);
    std::cout << "[retract] hand=" << arm_hand_side(arm)
              << " corner=sdk_profiles (no valid short quintic)" << std::endl;
    return out;
}
}


namespace t170::motion::detail {
int plan_line_then_line(
    Robot_Arm &arm,
    Eigen::Matrix<double, 1, 6> mid,
    Eigen::Matrix<double, 1, 6> goal,
    double vel_first_m_s,
    double vel_second_m_s,
    Eigen::MatrixXd &traj_out,
    const char *tag,
    const Eigen::MatrixXd *q_override,
    bool lift_then_retract)
{
    const int hand = arm_hand_side(arm);
    if (hand == 1 && right_arm_motors_locked())
        return 0;
    if (hand >= 0 && hand <= 1)
        g_planned_descent_row[hand] = -1;
    Eigen::MatrixXd peeked;
    const Eigen::MatrixXd *start = q_override;
    if (lift_then_retract && (start == nullptr || start->rows() != 1 || start->cols() < 6))
    {
        wrist_servo::Joints held;
        if (wrist_servo::peek_final_hold(hand, held))
        {
            peeked.resize(1, 7);
            peeked.row(0) = held;
            start = &peeked;
        }
    }
    Eigen::MatrixXd q_start = (start != nullptr && start->rows() == 1 && start->cols() >= 6)
                                  ? *start
                                  : arm_read_motor_joints(hand);
    if (lift_then_retract && start != nullptr)
    {
        const Eigen::Matrix<double, 1, 6> held_pose = arm_tcp_from_joints(arm, q_start.row(0));
        const double raised_z = mid(2);
        mid = held_pose;
        if (raised_z > held_pose(2) + 1e-4)
            mid(2) = raised_z;
    }
    Eigen::MatrixXd traj_mid;
    const int rc_mid = plan_arm_line_from(
        arm, mid, vel_first_m_s, q_start, traj_mid, lift_then_retract);
    if (rc_mid != 0)
        return rc_mid;
    Eigen::MatrixXd q_mid(1, traj_mid.cols());
    q_mid.row(0) = traj_mid.row(traj_mid.rows() - 1);
    Eigen::MatrixXd traj_goal;
    const int rc_goal = plan_arm_line_from(arm, goal, vel_second_m_s, q_mid, traj_goal);
    if (rc_goal != 0)
        return rc_goal;
    const int ib = lift_then_retract ? 0 : 1;
    const int n_tail = static_cast<int>(traj_goal.rows()) - ib;
    traj_out = lift_then_retract ? stitch_lift_then_return(arm, traj_mid, traj_goal)
                                 : concat_full_lines(traj_mid, traj_goal);
    if (traj_out.rows() < 2)
        return -16;
    int descent = static_cast<int>(traj_out.rows()) - n_tail;
    if (descent < 1)
        descent = 1;
    if (descent >= traj_out.rows())
        descent = static_cast<int>(traj_out.rows()) - 1;
    if (hand >= 0 && hand <= 1)
        g_planned_descent_row[hand] = descent;
    std::cout << "[" << (tag != nullptr ? tag : "line") << "] " << arm_hand_label(hand)
              << (lift_then_retract ? " 竖直抬升后衔接收手" : " 两段完整直线")
              << " 点=" << traj_out.rows()
              << " 第一段=" << traj_mid.rows()
              << " 第二段=" << traj_goal.rows()
              << " 抬升保留到=" << descent << "\n";
    return 0;
}
}


namespace {
struct PreparedLinePlan
{
    std::mutex mu;
    std::thread worker;
    bool have = false;
    int ret = -1;
    Eigen::MatrixXd traj;
    ~PreparedLinePlan() { if(worker.joinable()) worker.join(); }
};
}


namespace {
PreparedLinePlan g_prepared_line[2];
}


namespace t170::motion::detail {
bool take_prepared_line(int hand, Eigen::MatrixXd &traj, int &ret)
{
    if (hand < 0 || hand > 1)
        return false;
    PreparedLinePlan &slot = g_prepared_line[hand];
    if (slot.worker.joinable())
        slot.worker.join();
    std::lock_guard<std::mutex> lock(slot.mu);
    if (!slot.have || slot.traj.rows() < 2)
    {
        slot.have = false;
        return false;
    }
    const Eigen::MatrixXd q_now = arm_read_motor_joints(hand);
    double mismatch =
        (q_now.cols() == slot.traj.cols())
            ? (q_now.row(0) - slot.traj.row(0)).cwiseAbs().maxCoeff()
            : 1.0;
    wrist_servo::Joints held;
    if (wrist_servo::peek_final_hold(hand, held) && slot.traj.cols() >= 7)
    {
        const double hold_mismatch = (held - slot.traj.row(0).head(7)).cwiseAbs().maxCoeff();
        mismatch = std::min(mismatch, hold_mismatch);
    }
    if (!(mismatch <= 0.05))
    {
        std::cout << "[retract] " << arm_hand_label(hand)
                  << " 预规划起点已偏离，重新规划\n";
        slot.have = false;
        slot.traj.resize(0, 0);
        return false;
    }
    traj = slot.traj;
    ret = slot.ret;
    slot.have = false;
    slot.traj.resize(0, 0);
    std::cout << "[retract] " << arm_hand_label(hand)
              << " 合爪期间已规划好，直接下发\n";
    return true;
}
}


void arm_discard_line_then_line_plan(int hand)
{
    if(hand<0 || hand>1) return;
    auto &slot=g_prepared_line[hand];
    if(slot.worker.joinable()) slot.worker.join();
    std::lock_guard<std::mutex> lock(slot.mu);
    slot.have=false;slot.traj.resize(0,0);
}





int arm_transfer_move(Robot_Arm &arm, Matrix<double, 1, 6> &pos, double cart_linear_velocity)
{
    return arm_line_move(arm, pos, cart_linear_velocity);
}

ArmLineMoveResult arm_dual_transfer_move(
    Robot_Arm &arm_r,
    Matrix<double, 1, 6> &pos_r,
    Robot_Arm &arm_l,
    Matrix<double, 1, 6> &pos_l,
    double cart_linear_velocity)
{
    ArmLineMoveResult out;
    std::thread th_r([&]() { out.ret_r = arm_transfer_move(arm_r, pos_r, cart_linear_velocity); });
    std::thread th_l([&]() { out.ret_l = arm_transfer_move(arm_l, pos_l, cart_linear_velocity); });
    th_r.join();
    th_l.join();
    return out;
}



int arm_joint_move(Robot_Arm &arm, Matrix<double, 1, 7> &qd)
{
    const int hand = arm_hand_side(arm);
    if (hand == 1 && right_arm_motors_locked())
        return 0;
    const Eigen::MatrixXd q_start = arm_read_motor_joints(hand);
    Eigen::MatrixXd q_end = qd;
    if (q_end.rows() != 1)
        q_end = q_end.transpose();
    Eigen::MatrixXd traj;
    const int ret = arm.Joint_Trajectory(q_start, q_end, traj);
    if (ret != 0)
    {
        std::cerr << "[arm] Joint_Trajectory ret=" << ret
                  << "（起点可在模型限位外），改用不限位关节插补回目标\n";
        traj = make_linear_joint_traj(q_start, q_end, k_arm_traj_dt);
    }
    return execute_arm_trajectory(traj, hand, k_arm_traj_dt);
}
