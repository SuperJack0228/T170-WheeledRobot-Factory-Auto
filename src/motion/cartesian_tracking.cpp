#include "t170/vision/wrist_observer.h"
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
constexpr double kServoDtS = 5e-3;
}


namespace {
constexpr double kServoDeadM = 0.002;
}


namespace {
constexpr double kServoVmaxMps = 0.05;
}


namespace {
constexpr double kServoKp = 2.0;
}


namespace {
constexpr double kServoAmax = 0.40;
}


namespace {
constexpr double kServoMaxJointStepRad = 0.02;
}


namespace {
constexpr double kServoBranchRad = 0.35;
}


namespace {
double servo_apply_pose(Arm &arm, int hand, const Eigen::Matrix<double, 1, 6> &pose,
                        Eigen::RowVectorXd &q, int &ik_code, double &max_dq)
{
    ik_code = -1;
    max_dq = 0.0;
    if (q.size() < 7)
        return 0.0;
    Eigen::RowVectorXd q_try = q;
    auto joint_jump = [&](const Eigen::RowVectorXd &cand) {
        double jump = 0.0;
        for (int j = 0; j < 7; ++j)
            jump = std::max(jump, std::abs(cand(j) - q(j)));
        return jump;
    };
    auto usable = [&](const Eigen::RowVectorXd &cand, double &jump) {
        if (cand.size() < 7 || !cand.allFinite() || arm.Check_Joint_Limit(cand))
            return false;
        jump = joint_jump(cand);
        return jump <= kServoBranchRad;
    };
    {
        std::lock_guard<std::mutex> kin(g_arm_kin_mu[hand == 1 ? 1 : 0]);
        ik_code = arm.Inverse_Kinematics_Analytic(pose, q_try, q(1));
        double jump = 0.0;
        if (ik_code != 0 || !usable(q_try, jump))
        {
            q_try = q;
            Eigen::Matrix<double, 1, 6> pose_num = pose;
            const int numeric_rc = arm.Inverse_Kinematics_Numeric(pose_num, q_try);
            if (numeric_rc != 0 || !usable(q_try, jump))
            {
                max_dq = jump;
                if (ik_code == 0)
                    ik_code = numeric_rc != 0 ? numeric_rc : -12;
                return 0.0;
            }
            ik_code = 0;
        }
        max_dq = jump;
    }
    double scale = 1.0;
    if (max_dq > kServoMaxJointStepRad)
    {
        scale = kServoMaxJointStepRad / max_dq;
        q_try = q + scale * (q_try - q);
    }
    wrist_servo::send_joint_reference(hand, q_try);
    q = std::move(q_try);
    return scale;
}
}


namespace {
struct ServoSide
{
    bool on = false;
    int hand = 0;
    Eigen::Matrix<double, 1, 6> cmd = Eigen::Matrix<double, 1, 6>::Zero();
    Eigen::Matrix<double, 1, 6> goal = Eigen::Matrix<double, 1, 6>::Zero();
    Eigen::Vector2d vel = Eigen::Vector2d::Zero();
    Eigen::RowVectorXd q;
    int ik_fail = 0;
    int clamped = 0;
    bool logged_fail = false;
};
}


namespace {
void servo_init_side(ServoSide &side, Arm &arm, bool on,
                     const Eigen::Matrix<double, 1, 6> &start_pose)
{
    side.on = on && !(arm_hand_side(arm) == 1 && right_arm_motors_locked());
    if (!side.on)
        return;
    side.hand = arm_hand_side(arm);
    side.q.resize(7);
    if (side.hand >= 0 && side.hand <= 1 && g_arm_last_cmd_valid[side.hand])
    {
        for (int j = 0; j < 7; ++j)
            side.q(j) = encoder_cnt_to_rad(g_arm_last_cmd_cnt[side.hand][j]);
        side.q = arm.to_model(side.q);
    }
    else
    {
        side.q = arm_read_model_joints(side.hand).row(0);
    }
    side.cmd = start_pose;
    if (!side.cmd.allFinite() || side.q.size() < 7)
    {
        side.on = false;
        std::cerr << "[伺服] " << (side.hand == 1 ? "右手" : "左手")
                  << " 当前位姿无效，这一侧不跟踪\n";
        return;
    }
    side.goal = side.cmd;
}
}


namespace {
void servo_step_side(ServoSide &side, Arm &arm)
{
    if (!side.on)
        return;
    const Eigen::Vector2d err(side.goal(0) - side.cmd(0), side.goal(1) - side.cmd(1));
    const double dist = err.norm();
    Eigen::Vector2d v_des = Eigen::Vector2d::Zero();
    if (dist > kServoDeadM)
    {
        const double vm = std::min(kServoVmaxMps, dist * kServoKp);
        v_des = err / dist * vm;
    }
    Eigen::Vector2d dv = v_des - side.vel;
    const double dvn = dv.norm();
    const double cap = kServoAmax * kServoDtS;
    if (dvn > cap && dvn > 1e-12)
        dv *= cap / dvn;
    const Eigen::Vector2d vel_next = side.vel + dv;
    Eigen::Matrix<double, 1, 6> next = side.cmd;
    next(0) += vel_next(0) * kServoDtS;
    next(1) += vel_next(1) * kServoDtS;
    int ik_code = 0;
    double max_dq = 0.0;
    const double scale = servo_apply_pose(arm, side.hand, next, side.q, ik_code, max_dq);
    if (scale > 0.0)
    {
        side.cmd(0) += (next(0) - side.cmd(0)) * scale;
        side.cmd(1) += (next(1) - side.cmd(1)) * scale;
        side.vel = vel_next * scale;
        if (scale < 0.999)
            ++side.clamped;
    }
    else
    {
        side.vel *= 0.5;
        ++side.ik_fail;
        if (!side.logged_fail)
        {
            side.logged_fail = true;
            std::cerr << std::fixed << std::setprecision(3)
                      << "[伺服] " << (side.hand == 1 ? "右手" : "左手")
                      << " 这一拍没发出去，逆解 code=" << ik_code
                      << " 关节差 " << (max_dq * 180.0 / M_PI) << "°\n";
        }
    }
}
}


int arm_dual_cartesian_track(
    Arm &arm_r,
    Arm &arm_l,
    const Eigen::Matrix<double, 1, 6> &start_r,
    const Eigen::Matrix<double, 1, 6> &start_l,
    bool move_r,
    bool move_l,
    double duration_s,
    const std::function<void(double t_s,
                             Eigen::Matrix<double, 1, 6> &goal_r,
                             Eigen::Matrix<double, 1, 6> &goal_l)> &update_goal)
{
    const bool until_abort = duration_s == 0.0;
    if ((!until_abort && !(duration_s > 0.0)) || !update_goal)
        return -1;
    ServoSide right;
    ServoSide left;
    servo_init_side(right, arm_r, move_r, start_r);
    servo_init_side(left, arm_l, move_l, start_l);
    if ((move_r && !right.on) || (move_l && !left.on))
        return -1;
    if (!right.on && !left.on)
        return 0;

    std::cout << std::fixed << std::setprecision(1)
              << "[伺服] 连续跟踪 ";
    if (until_abort)
        std::cout << "直到 abort";
    else
        std::cout << duration_s << " s";
    std::cout << "，每 5 ms 一步，"
              << "速度上限 " << (kServoVmaxMps * 1000.0)
              << " mm/s，加速度上限 " << kServoAmax
              << " m/s²，死区 " << (kServoDeadM * 1000.0) << " mm\n";

    const auto t0 = std::chrono::steady_clock::now();
    int ticks = 0;
    while (true)
    {
        if (hardware_abort_requested())
            return -4;
        if (can_io_faulted())
            return -2;
        const auto cycle = std::chrono::steady_clock::now();
        const double t = std::chrono::duration<double>(cycle - t0).count();
        if (!until_abort && t >= duration_s)
            break;
        update_goal(t, right.goal, left.goal);
        servo_step_side(right, arm_r);
        servo_step_side(left, arm_l);
        if (ticks % 20 == 0)
        {
            auto report = [](const char *name, const ServoSide &side) {
                if (!side.on)
                    return;
                const double ex = (side.goal(0) - side.cmd(0)) * 1000.0;
                const double ey = (side.goal(1) - side.cmd(1)) * 1000.0;
                const double sp = side.vel.norm() * 1000.0;
                std::cout << std::fixed << std::setprecision(1)
                          << "  " << name << " 还差 " << std::hypot(ex, ey)
                          << " mm  速度 " << sp << " mm/s";
            };
            std::cout << "[伺服] t=" << std::setprecision(1) << t << "s";
            report("右", right);
            report("左", left);
            std::cout << std::endl;
        }
        if (ticks % 200 == 0)
        {
            auto one = [](const char *name, const ServoSide &side) {
                if (!side.on)
                    return std::string();
                const double left_mm = std::hypot((side.goal(0) - side.cmd(0)) * 1000.0,
                                                  (side.goal(1) - side.cmd(1)) * 1000.0);
                std::ostringstream text;
                text << std::fixed << std::setprecision(1) << name << " 还差 " << left_mm
                     << " mm 失败 " << side.ik_fail << " 拍";
                return text.str();
            };
            const std::string right_text = one("右", right);
            const std::string left_text = one("左", left);
            std::string detail;
            if (!right_text.empty())
                detail += right_text;
            if (!left_text.empty())
            {
                if (!detail.empty())
                    detail += "；";
                detail += left_text;
            }
            hand_track_log("伺服", "跟踪", detail);
        }
        ++ticks;
        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - cycle).count();
        const double sleep_ms = std::max(0.0, kServoDtS * 1000.0 - elapsed_ms);
        if (sleep_ms > 1e-6)
            std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(sleep_ms));
    }
    std::cout << "[伺服] 跟踪结束"
              << " 右逆解失败 " << right.ik_fail << " 拍、限幅 " << right.clamped
              << " 拍；左逆解失败 " << left.ik_fail << " 拍、限幅 " << left.clamped
              << " 拍\n";
    return 0;
}
