#include "t170/app/diagnostic_gestures.h"
#include "../motion/internal/arm_internal.h"
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
using WaveQ = Eigen::Matrix<double, 1, 7>;
}


namespace {
WaveQ wave_q(double j1, double j2, double j3, double j4, double j5, double j6, double j7)
{
    WaveQ q;
    q << j1, j2, j3, j4, j5, j6, j7;
    return q;
}
}


namespace {
const WaveQ k_wave_hang = wave_q(-0.0140, 1.3142, -0.5882, 0.2129, -0.5777, 0.3019, 0.0925);
}


namespace {
const WaveQ k_wave_front = wave_q(0.0541, 1.0385, -0.6562, 1.0437, -0.3019, 0.6109, -0.2601);
}


namespace {
const WaveQ k_wave_raise = wave_q(0.2286, 1.0071, 0.2548, 1.7924, 1.1519, 0.6109, 0.9215);
}


namespace {
const WaveQ k_wave_out = wave_q(0.2286, 1.0071, 0.3752, 1.6825, 1.3980, 0.6109, 0.8866);
}


namespace {
const WaveQ k_wave_in = wave_q(0.2304, 1.0071, 0.1344, 1.9007, 0.9058, 0.6109, 0.9564);
}


namespace {
constexpr double k_wave_amp = 1.4;
}


namespace {
constexpr double k_wave_head_pitch_deg = 10.0;
}


namespace {
constexpr double k_wave_grip_ready = 0.3;
}


namespace {
constexpr double k_wave_period_s = 1.1;
}


namespace {
constexpr double k_wave_cycles = 3.0;
}


namespace {
constexpr double k_wave_skip_rad = 2.0 * M_PI / 180.0;
}


namespace {
const double k_wave_right_min_deg[7] = {-90, -90, -90, -10, -180, -40, -20};
}


namespace {
const double k_wave_right_max_deg[7] = {180, 5, 90, 140, 180, 40, 90};
}


namespace {
const double k_wave_left_min_deg[7] = {-90, 5, -90, -10, -90, -40, -90};
}


namespace {
const double k_wave_left_max_deg[7] = {180, 90, 90, 140, 90, 40, 20};
}


namespace {
double wave_model_rad(int hand, int joint, double q_motor)
{
    double dir = 1.0;
    double off = 0.0;
    if (hand == 1)
    {
        if (joint == 1)
            off = M_PI / 2.0;
    }
    else
    {
        if (joint == 0 || joint == 3 || joint == 5)
            dir = -1.0;
        if (joint == 1)
            off = -M_PI / 2.0;
    }
    return (q_motor - off) / dir;
}
}


namespace {
int wave_log_over_limit(Robot_Arm &arm, int hand, const WaveQ &q, const char *tag)
{
    if (!arm.Check_Joint_Limit(q))
        return 0;
    const double *mn = hand == 1 ? k_wave_right_min_deg : k_wave_left_min_deg;
    const double *mx = hand == 1 ? k_wave_right_max_deg : k_wave_left_max_deg;
    bool named = false;
    for (int j = 0; j < 7; ++j)
    {
        const double model_deg = wave_model_rad(hand, j, q(j)) * 180.0 / M_PI;
        if (model_deg >= mn[j] - 1e-3 && model_deg <= mx[j] + 1e-3)
            continue;
        named = true;
        std::cerr << std::fixed << std::setprecision(2)
                  << "[wave] " << (hand == 1 ? "右臂" : "左臂")
                  << (tag ? tag : "") << " J" << (j + 1)
                  << " 超限 电机角 " << (q(j) * 180.0 / M_PI)
                  << "° 模型角(conv0) " << model_deg
                  << "° 限位 [" << mn[j] << ", " << mx[j] << "]°\n";
    }
    if (!named)
    {
        std::cerr << "[wave] " << (hand == 1 ? "右臂" : "左臂")
                  << (tag ? tag : "") << " Check_Joint_Limit 超限\n";
    }
    return -1;
}
}


namespace {
int wave_check_traj(Robot_Arm &arm, int hand, const Eigen::MatrixXd &traj, const char *tag,
                    bool skip_start = false)
{
    const int i0 = (skip_start && traj.rows() > 1) ? 1 : 0;
    for (int i = i0; i < traj.rows(); ++i)
    {
        WaveQ q;
        for (int j = 0; j < 7; ++j)
            q(j) = traj(i, j);
        if (wave_log_over_limit(arm, hand, q, tag) != 0)
            return -1;
    }
    return 0;
}
}


namespace {
Eigen::MatrixXd wave_quintic(const WaveQ &qa, const WaveQ &qb, double duration_s)
{
    const int n = std::max(2, static_cast<int>(std::lround(duration_s / k_arm_traj_dt)) + 1);
    Eigen::MatrixXd traj(n, 7);
    for (int i = 0; i < n; ++i)
    {
        const double u = static_cast<double>(i) / static_cast<double>(n - 1);
        const double u2 = u * u;
        const double u3 = u2 * u;
        const double s = 10.0 * u3 - 15.0 * u2 * u2 + 6.0 * u2 * u3;
        for (int j = 0; j < 7; ++j)
            traj(i, j) = qa(j) + s * (qb(j) - qa(j));
    }
    return traj;
}
}


namespace {
Eigen::MatrixXd wave_sine_traj()
{
    const double total = k_wave_period_s * k_wave_cycles;
    const double half = k_wave_period_s * 0.5;
    const WaveQ amp = k_wave_amp * (k_wave_out - k_wave_raise);
    const int n = std::max(2, static_cast<int>(std::lround(total / k_arm_traj_dt)) + 1);
    Eigen::MatrixXd traj(n, 7);
    for (int i = 0; i < n; ++i)
    {
        const double t = std::min(total, static_cast<double>(i) * k_arm_traj_dt);
        const double phase = 2.0 * M_PI * t / k_wave_period_s;
        double env = 1.0;
        if (t < half)
            env = 0.5 * (1.0 - std::cos(M_PI * t / half));
        else if (t > total - half)
            env = 0.5 * (1.0 - std::cos(M_PI * (total - t) / half));
        const double s = env * std::sin(phase);
        for (int j = 0; j < 7; ++j)
            traj(i, j) = k_wave_raise(j) + s * amp(j);
    }
    return traj;
}
}


namespace {
int wave_pause_ms(int ms)
{
    hardware_abort_sleep_ms(ms);
    if (hardware_abort_requested())
        return -4;
    return 0;
}
}


namespace {
Eigen::MatrixXd wave_via(const WaveQ &q0, const WaveQ &q1, const WaveQ &q2, double t1, double t2)
{
    auto eval = [](double t, double T, double q_a, double q_b, double v_a, double v_b) {
        const double T2 = T * T;
        const double T3 = T2 * T;
        const double T4 = T3 * T;
        const double T5 = T4 * T;
        const double a0 = q_a;
        const double a1 = v_a;
        const double a2 = 0.0;
        const double a3 = (20.0 * (q_b - q_a) - (8.0 * v_b + 12.0 * v_a) * T) / (2.0 * T3);
        const double a4 = (30.0 * (q_a - q_b) + (14.0 * v_b + 16.0 * v_a) * T) / (2.0 * T4);
        const double a5 = (12.0 * (q_b - q_a) - 6.0 * (v_b + v_a) * T) / (2.0 * T5);
        const double t2p = t * t;
        const double t3p = t2p * t;
        return a0 + a1 * t + a2 * t2p + a3 * t3p + a4 * t2p * t2p + a5 * t2p * t3p;
    };
    const int n1 = std::max(2, static_cast<int>(std::lround(t1 / k_arm_traj_dt)) + 1);
    const int n2 = std::max(2, static_cast<int>(std::lround(t2 / k_arm_traj_dt)) + 1);
    Eigen::MatrixXd traj(n1 + n2 - 1, 7);
    WaveQ v_mid;
    for (int j = 0; j < 7; ++j)
    {
        const double v_in = (q1(j) - q0(j)) / t1;
        const double v_out = (q2(j) - q1(j)) / t2;
        v_mid(j) = 0.5 * (v_in + v_out);
    }
    for (int i = 0; i < n1; ++i)
    {
        const double t = t1 * static_cast<double>(i) / static_cast<double>(n1 - 1);
        for (int j = 0; j < 7; ++j)
            traj(i, j) = eval(t, t1, q0(j), q1(j), 0.0, v_mid(j));
    }
    for (int i = 1; i < n2; ++i)
    {
        const double t = t2 * static_cast<double>(i) / static_cast<double>(n2 - 1);
        for (int j = 0; j < 7; ++j)
            traj(n1 + i - 1, j) = eval(t, t2, q1(j), q2(j), v_mid(j), 0.0);
    }
    return traj;
}
}


namespace {
int wave_head_zero()
{
    if (head_id < 0)
    {
        std::cerr << "[wave] 头电机未绑定\n";
        return -2;
    }
    for (uint32_t id : kHeadCanIds)
    {
        if (!motor_enable_id(id))
            std::cerr << "[wave] 头使能失败 id=" << id << "，仍尝试回零\n";
    }
    uint32_t ids[3] = {kHeadYawCanId, kHeadPitchCanId, kHeadRollCanId};
    const double pose[3] = {0.0, k_wave_head_pitch_deg, 0.0};
    std::cout << "[wave] 头偏航和横滚回零，俯仰低到 " << k_wave_head_pitch_deg << "°\n";
    if (smooth_motor_move_deg(head_id, 3, ids, pose, 25.0, 20) != 0)
        return -2;
    if (hardware_abort_requested())
        return -4;
    return 0;
}
}


namespace {
void wave_waist_yaw_deg(double yaw_deg)
{
    if (waist_id < 0)
        return;
    uint32_t id = 1;
    int cnt = motor_angle_deg_to_cnt(yaw_deg);
    socketcan_sendcommand(waist_id, 1, &id, 30, &cnt);
}
}


namespace {

}


namespace {
double wave_quintic_s(double u)
{
    const double u2 = u * u;
    const double u3 = u2 * u;
    return 10.0 * u3 - 15.0 * u2 * u2 + 6.0 * u2 * u3;
}
}


namespace {
int wave_play_lift(int hand, const Eigen::MatrixXd &traj, const std::vector<double> &yaw_deg)
{
    if (hardware_abort_requested())
        return -4;
    if (can_io_faulted())
        return -2;
    if (traj.rows() < 2)
        return 0;
    if (hand == 1 && right_arm_motors_locked())
        return 0;
    const int dof = static_cast<int>(std::min(traj.cols(), static_cast<Eigen::Index>(7)));
    const int n = static_cast<int>(traj.rows());
    for (int i = 0; i < n; ++i)
    {
        if (hardware_abort_requested())
        {
            std::cout << "[arm] STOP，停止轨迹\n";
            return -4;
        }
        if (can_io_faulted())
        {
            std::cout << "[arm] CAN 收发失败，停止轨迹: " << can_io_fault_message() << std::endl;
            return -2;
        }
        const auto cycle_start = std::chrono::high_resolution_clock::now();
        int motor_cmd[7] = {0};
        for (int j = 0; j < dof; ++j)
            motor_cmd[j] = rad_to_encoder_cnt(traj(i, j));
        set_motor_position(motor_cmd, hand);
        if (i < static_cast<int>(yaw_deg.size()))
            wave_waist_yaw_deg(yaw_deg[static_cast<size_t>(i)]);
        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::high_resolution_clock::now() - cycle_start)
                .count();
        const double sleep_ms = std::max(0.0, k_arm_traj_dt * 1000.0 - elapsed_ms);
        if (sleep_ms > 1e-6)
            std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(sleep_ms));
    }
    int last_cmd[7] = {0};
    for (int j = 0; j < dof; ++j)
        last_cmd[j] = rad_to_encoder_cnt(traj(traj.rows() - 1, j));
    save_arm_last_commanded_position(hand, last_cmd);
    return 0;
}
}


namespace {
void wave_head_pitch_deg(double pitch_deg)
{
    if (head_id < 0)
        return;
    uint32_t ids[3] = {kHeadYawCanId, kHeadPitchCanId, kHeadRollCanId};
    int cnt[3] = {
        motor_angle_deg_to_cnt(0.0),
        motor_angle_deg_to_cnt(pitch_deg),
        motor_angle_deg_to_cnt(0.0),
    };
    socketcan_sendcommand(head_id, 3, ids, 30, cnt);
}
}


namespace {
int wave_play(int hand, const Eigen::MatrixXd &traj, bool nod,
              const std::vector<double> *yaw_deg = nullptr)
{
    if (hardware_abort_requested())
        return -4;
    if (can_io_faulted())
        return -2;
    if (traj.rows() < 2)
        return 0;
    if (!nod && yaw_deg == nullptr)
        return execute_arm_trajectory(traj, hand, k_arm_traj_dt);

    if (hand == 1 && right_arm_motors_locked())
        return 0;
    const int dof = static_cast<int>(std::min(traj.cols(), static_cast<Eigen::Index>(7)));
    const double total = k_wave_period_s * k_wave_cycles;
    const double half = k_wave_period_s * 0.5;
    constexpr double k_nod_deg = 8.0;
    for (int i = 0; i < traj.rows(); ++i)
    {
        if (hardware_abort_requested())
        {
            std::cout << "[arm] STOP，停止轨迹\n";
            return -4;
        }
        if (can_io_faulted())
        {
            std::cout << "[arm] CAN 收发失败，停止轨迹: " << can_io_fault_message() << std::endl;
            return -2;
        }
        const auto cycle_start = std::chrono::high_resolution_clock::now();
        int motor_cmd[7] = {0};
        for (int j = 0; j < dof; ++j)
            motor_cmd[j] = rad_to_encoder_cnt(traj(i, j));
        set_motor_position(motor_cmd, hand);
        if (yaw_deg != nullptr && i < static_cast<int>(yaw_deg->size()))
            wave_waist_yaw_deg((*yaw_deg)[static_cast<size_t>(i)]);

        if (nod)
        {
            const double t = std::min(total, static_cast<double>(i) * k_arm_traj_dt);
            const double phase = 2.0 * M_PI * t / k_wave_period_s;
            double env = 1.0;
            if (t < half)
                env = 0.5 * (1.0 - std::cos(M_PI * t / half));
            else if (t > total - half)
                env = 0.5 * (1.0 - std::cos(M_PI * (total - t) / half));
            wave_head_pitch_deg(k_wave_head_pitch_deg +
                                k_nod_deg * env * 0.5 * (1.0 - std::cos(phase)));
        }

        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::high_resolution_clock::now() - cycle_start)
                .count();
        const double sleep_ms = std::max(0.0, k_arm_traj_dt * 1000.0 - elapsed_ms);
        if (sleep_ms > 1e-6)
            std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(sleep_ms));
    }
    int last_cmd[7] = {0};
    for (int j = 0; j < dof; ++j)
        last_cmd[j] = rad_to_encoder_cnt(traj(traj.rows() - 1, j));
    save_arm_last_commanded_position(hand, last_cmd);
    if (nod)
        wave_head_pitch_deg(k_wave_head_pitch_deg);
    return 0;
}
}


namespace {
int wave_goto(Robot_Arm &arm, const WaveQ &goal, double fixed_s)
{
    const int hand = arm_hand_side(arm);
    if (hardware_abort_requested())
        return -4;
    if (can_io_faulted())
        return -2;
    const Eigen::MatrixXd q_now = arm_read_motor_joints(hand);
    if (can_io_faulted() || !q_now.allFinite())
        return -2;
    WaveQ qa;
    for (int j = 0; j < 7; ++j)
        qa(j) = q_now(0, j);
    const double max_dq = (qa - goal).cwiseAbs().maxCoeff();
    if (max_dq <= k_wave_skip_rad)
        return 0;
    const double duration = fixed_s > 0.0 ? fixed_s : std::max(1.0, max_dq / 0.45);
    const Eigen::MatrixXd traj = wave_quintic(qa, goal, duration);
    if (wave_check_traj(arm, hand, traj, "", true) != 0)
        return -1;
    return wave_play(hand, traj, false);
}
}


namespace {
Eigen::MatrixXd wave_sine_for(const WaveQ &raise, const WaveQ &outward)
{
    const double total = k_wave_period_s * k_wave_cycles;
    const double half = k_wave_period_s * 0.5;
    const WaveQ amp = k_wave_amp * (outward - raise);
    const int n = std::max(2, static_cast<int>(std::lround(total / k_arm_traj_dt)) + 1);
    Eigen::MatrixXd traj(n, 7);
    for (int i = 0; i < n; ++i)
    {
        const double t = std::min(total, static_cast<double>(i) * k_arm_traj_dt);
        const double phase = 2.0 * M_PI * t / k_wave_period_s;
        double env = 1.0;
        if (t < half)
            env = 0.5 * (1.0 - std::cos(M_PI * t / half));
        else if (t > total - half)
            env = 0.5 * (1.0 - std::cos(M_PI * (total - t) / half));
        const double s = env * std::sin(phase);
        for (int j = 0; j < 7; ++j)
            traj(i, j) = raise(j) + s * amp(j);
    }
    return traj;
}
}


namespace {
int wave_one(Robot_Arm &arm, bool is_right)
{
    const WaveQ hang = is_right ? k_wave_hang : WaveQ(-k_wave_hang);
    const WaveQ front = is_right ? k_wave_front : WaveQ(-k_wave_front);
    const WaveQ raise = is_right ? k_wave_raise : WaveQ(-k_wave_raise);
    const WaveQ outward = is_right ? k_wave_out : WaveQ(-k_wave_out);
    const int hand = is_right ? 1 : 0;
    const char *name = is_right ? "右手" : "左手";

    WaveQ q_from = hang;
    const Eigen::MatrixXd q_now = arm_read_motor_joints(hand);
    if (can_io_faulted() || !q_now.allFinite())
        return -2;
    for (int j = 0; j < 7; ++j)
        q_from(j) = q_now(0, j);

    constexpr double k_lift_front_s = 1.0;
    constexpr double k_lift_raise_s = 0.8;
    constexpr double k_pause_s = 0.25;
    constexpr double k_lower_front_s = 0.9;
    constexpr double k_lower_hang_s = 1.1;
    constexpr double k_welcome_waist_deg = 20.0;
    const double wave_s = k_wave_period_s * k_wave_cycles;
    const double yaw_here = is_right ? -k_welcome_waist_deg : k_welcome_waist_deg;
    const double yaw_across = -yaw_here;
    const double t_side = wave_s;
    const double t_across = t_side + wave_s;
    const double t_end = t_across + wave_s;
    auto yaw_at = [&](double t) {
        if (t <= 0.0)
            return 0.0;
        if (t < t_side)
            return wave_quintic_s(t / t_side) * yaw_here;
        if (t < t_across)
        {
            const double u = (t - t_side) / (t_across - t_side);
            return yaw_here + wave_quintic_s(u) * (yaw_across - yaw_here);
        }
        if (t < t_end)
        {
            const double u = (t - t_across) / (t_end - t_across);
            return yaw_across + wave_quintic_s(u) * (0.0 - yaw_across);
        }
        return 0.0;
    };
    auto yaw_for = [&](double t0, int n) {
        std::vector<double> y(static_cast<size_t>(std::max(0, n)), 0.0);
        for (int i = 0; i < n; ++i)
            y[static_cast<size_t>(i)] = yaw_at(t0 + static_cast<double>(i) * k_arm_traj_dt);
        return y;
    };
    std::cout << "[welcome] " << name
              << " 腰 ±" << k_welcome_waist_deg
              << "°，开始转到挥手侧、再转到另一侧、最后回 0，每段 "
              << wave_s << "s，和挥三下一样长\n";
    enable_waist_motors();
    const Eigen::MatrixXd lift = wave_via(q_from, front, raise, k_lift_front_s, k_lift_raise_s);
    if (wave_check_traj(arm, hand, lift, " 抬手") != 0)
        return -1;
    double t_cursor = 0.0;
    const std::vector<double> yaw_lift = yaw_for(t_cursor, static_cast<int>(lift.rows()));
    int rc = wave_play_lift(hand, lift, yaw_lift);
    t_cursor += static_cast<double>(lift.rows()) * k_arm_traj_dt;
    if (rc != 0)
        return rc;
    const int pause_n = std::max(1, static_cast<int>(std::lround(k_pause_s / k_arm_traj_dt)));
    for (int i = 0; i < pause_n; ++i)
    {
        if (hardware_abort_requested())
            return -4;
        wave_waist_yaw_deg(yaw_at(t_cursor));
        t_cursor += k_arm_traj_dt;
        std::this_thread::sleep_for(std::chrono::duration<double>(k_arm_traj_dt));
    }
    orch::wave_grip_ratio(is_right, k_wave_grip_ready);
    if (hardware_abort_requested())
        return -4;
    std::cout << "[welcome] " << name << " 挥 3 下\n";
    const Eigen::MatrixXd sine = wave_sine_for(raise, outward);
    if (wave_check_traj(arm, hand, sine, " 挥动") != 0)
    {
        orch::wave_grip_ratio(is_right, 1.0);
        return -1;
    }
    const std::vector<double> yaw_sine = yaw_for(t_cursor, static_cast<int>(sine.rows()));
    rc = wave_play(hand, sine, true, &yaw_sine);
    t_cursor += static_cast<double>(sine.rows()) * k_arm_traj_dt;
    if (rc != 0)
    {
        orch::wave_grip_ratio(is_right, 1.0);
        return rc;
    }
    std::cout << "[welcome] " << name << " 放下\n";
    const Eigen::MatrixXd lower = wave_via(raise, front, hang, k_lower_front_s, k_lower_hang_s);
    if (wave_check_traj(arm, hand, lower, " 放下") != 0)
    {
        orch::wave_grip_ratio(is_right, 1.0);
        return -1;
    }
    const std::vector<double> yaw_lower = yaw_for(t_cursor, static_cast<int>(lower.rows()));
    rc = wave_play(hand, lower, false, &yaw_lower);
    t_cursor += static_cast<double>(lower.rows()) * k_arm_traj_dt;
    orch::wave_grip_ratio(is_right, 1.0);
    if (rc != 0)
        return rc;
    while (t_cursor < t_end)
    {
        if (hardware_abort_requested())
            return -4;
        wave_waist_yaw_deg(yaw_at(t_cursor));
        t_cursor += k_arm_traj_dt;
        std::this_thread::sleep_for(std::chrono::duration<double>(k_arm_traj_dt));
    }
    wave_waist_yaw_deg(0.0);
    return 0;
}
}


namespace {
struct BowWaist
{
    double ankle = 0.0;
    double knee = 0.0;
    double hip = 0.0;
    double roll = 0.0;
    double yaw = 0.0;
};
}


namespace {
BowWaist bow_read_waist()
{
    int cnt[5] = {0, 0, 0, 0, 0};
    get_motor_waist_position(cnt);
    auto geo = [](int c, double motor_to_geo) {
        return motor_to_geo * motor_cnt_to_angle_deg(c) * M_PI / 180.0;
    };
    BowWaist q;
    q.ankle = geo(cnt[0], -1.0);
    q.knee = geo(cnt[1], -1.0);
    q.hip = geo(cnt[2], 1.0);
    q.roll = geo(cnt[3], 1.0);
    q.yaw = geo(cnt[4], 1.0);
    return q;
}
}


namespace {
void bow_send_waist(const BowWaist &q)
{
    const double motor_deg[5] = {
        -q.ankle * 180.0 / M_PI,
        -q.knee * 180.0 / M_PI,
        q.hip * 180.0 / M_PI,
        q.roll * 180.0 / M_PI,
        q.yaw * 180.0 / M_PI,
    };
    int cnt[5] = {0, 0, 0, 0, 0};
    for (int i = 0; i < 5; ++i)
        cnt[i] = motor_angle_deg_to_cnt(motor_deg[i]);
    set_waist_motor_position(cnt);
}
}


namespace {
BowWaist bow_blend_waist(const BowWaist &a, const BowWaist &b, double s)
{
    BowWaist q;
    q.ankle = a.ankle + s * (b.ankle - a.ankle);
    q.knee = a.knee + s * (b.knee - a.knee);
    q.hip = a.hip + s * (b.hip - a.hip);
    q.roll = a.roll + s * (b.roll - a.roll);
    q.yaw = a.yaw + s * (b.yaw - a.yaw);
    return q;
}
}


namespace {
WaveQ bow_blend_arm(const WaveQ &a, const WaveQ &b, double s)
{
    return a + s * (b - a);
}
}


namespace {
int bow_play(const WaveQ &r0, const WaveQ &r1, const WaveQ &l0, const WaveQ &l1,
             const BowWaist &w0, const BowWaist &w1, double head0_deg, double head1_deg,
             double duration_s)
{
    enable_waist_motors();
    const int n = std::max(2, static_cast<int>(std::lround(duration_s / k_arm_traj_dt)) + 1);
    for (int i = 0; i < n; ++i)
    {
        if (hardware_abort_requested())
            return -4;
        if (can_io_faulted())
            return -2;
        const auto cycle_start = std::chrono::high_resolution_clock::now();
        const double u = static_cast<double>(i) / static_cast<double>(n - 1);
        const double s = wave_quintic_s(u);
        const WaveQ right = bow_blend_arm(r0, r1, s);
        const WaveQ left = bow_blend_arm(l0, l1, s);
        int cmd_r[7] = {0};
        int cmd_l[7] = {0};
        for (int j = 0; j < 7; ++j)
        {
            cmd_r[j] = rad_to_encoder_cnt(right(j));
            cmd_l[j] = rad_to_encoder_cnt(left(j));
        }
        set_motor_position(cmd_r, 1);
        set_motor_position(cmd_l, 0);
        bow_send_waist(bow_blend_waist(w0, w1, s));
        wave_head_pitch_deg(head0_deg + s * (head1_deg - head0_deg));
        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::high_resolution_clock::now() - cycle_start)
                .count();
        const double sleep_ms = std::max(0.0, k_arm_traj_dt * 1000.0 - elapsed_ms);
        if (sleep_ms > 1e-6)
            std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(sleep_ms));
    }
    int last_r[7] = {0};
    int last_l[7] = {0};
    for (int j = 0; j < 7; ++j)
    {
        last_r[j] = rad_to_encoder_cnt(r1(j));
        last_l[j] = rad_to_encoder_cnt(l1(j));
    }
    save_arm_last_commanded_position(1, last_r);
    save_arm_last_commanded_position(0, last_l);
    return 0;
}
}


namespace {
Eigen::MatrixXd bow_rest_via(const WaveQ &q0, const WaveQ &q1, const WaveQ &q2, double t1, double t2)
{
    const Eigen::MatrixXd a = wave_quintic(q0, q1, t1);
    const Eigen::MatrixXd b = wave_quintic(q1, q2, t2);
    Eigen::MatrixXd traj(a.rows() + b.rows() - 1, 7);
    traj.topRows(a.rows()) = a;
    traj.bottomRows(b.rows() - 1) = b.bottomRows(b.rows() - 1);
    return traj;
}
}


int arm_wave_right(Robot_Arm &arm_r, Robot_Arm &arm_l)
{
    if (arm_hand_side(arm_r) != 1 || arm_hand_side(arm_l) != 0)
    {
        std::cerr << "[wave] 需要右臂和左臂\n";
        return -1;
    }
    if (right_arm_motors_locked())
    {
        std::cerr << "[wave] 右臂电机锁定，不挥手\n";
        return -6;
    }
    if (hardware_abort_requested())
        return -4;

    const WaveQ q_hang_l = -k_wave_hang;
    const Eigen::MatrixXd sine = wave_sine_traj();
    if (wave_log_over_limit(arm_r, 1, k_wave_hang, " 下垂") != 0 ||
        wave_log_over_limit(arm_r, 1, k_wave_front, " 身前") != 0 ||
        wave_log_over_limit(arm_r, 1, k_wave_raise, " 举起") != 0 ||
        wave_log_over_limit(arm_r, 1, k_wave_out, " 靠右") != 0 ||
        wave_log_over_limit(arm_r, 1, k_wave_in, " 收回") != 0 ||
        wave_log_over_limit(arm_l, 0, q_hang_l, " 下垂") != 0 ||
        wave_check_traj(arm_r, 1, sine, " 挥动") != 0)
        return -1;

    int rc = wave_head_zero();
    if (rc != 0)
        return rc;
    std::cout << "[wave] 左手收到下垂\n";
    rc = wave_goto(arm_l, q_hang_l, -1.0);
    if (rc != 0)
        return rc;
    std::cout << "[wave] 右手收到下垂\n";
    rc = wave_goto(arm_r, k_wave_hang, -1.0);
    if (rc != 0)
        return rc;
    orch::wave_grip_ratio(false, 1.0);
    orch::wave_grip_ratio(true, 1.0);
    if (hardware_abort_requested())
        return -4;
    std::cout << "[wave] 右手经身前抬到肩高\n";
    WaveQ q_lift = k_wave_hang;
    const Eigen::MatrixXd q_at_hang = arm_read_motor_joints(1);
    if (can_io_faulted() || !q_at_hang.allFinite())
        return -2;
    for (int j = 0; j < 7; ++j)
        q_lift(j) = q_at_hang(0, j);
    const Eigen::MatrixXd lift = wave_via(q_lift, k_wave_front, k_wave_raise, 1.0, 0.8);
    if (wave_check_traj(arm_r, 1, lift, " 抬手") != 0)
        return -1;
    rc = wave_play(1, lift, false);
    if (rc != 0)
        return rc;
    rc = wave_pause_ms(250);
    if (rc != 0)
        return rc;
    orch::wave_grip_ratio(true, k_wave_grip_ready);
    if (hardware_abort_requested())
        return -4;
    std::cout << "[wave] 右手挥 3 下，头轻微点头\n";
    rc = wave_play(1, sine, true);
    if (rc != 0)
    {
        orch::wave_grip_ratio(true, 1.0);
        return rc;
    }
    std::cout << "[wave] 右手连续放下\n";
    const Eigen::MatrixXd lower = wave_via(k_wave_raise, k_wave_front, k_wave_hang, 0.9, 1.1);
    if (wave_check_traj(arm_r, 1, lower, " 放下") != 0)
    {
        orch::wave_grip_ratio(true, 1.0);
        return -1;
    }
    rc = wave_play(1, lower, false);
    orch::wave_grip_ratio(true, 1.0);
    if (rc != 0)
        return rc;
    std::cout << "[wave] 完成，双手停在下垂\n";
    return 0;
}

int arm_wave_hang(Robot_Arm &arm_r, Robot_Arm &arm_l)
{
    if (arm_hand_side(arm_r) != 1 || arm_hand_side(arm_l) != 0)
    {
        std::cerr << "[wave] 需要右臂和左臂\n";
        return -1;
    }
    if (right_arm_motors_locked())
    {
        std::cerr << "[wave] 右臂电机锁定，不进下垂准备\n";
        return -6;
    }
    if (hardware_abort_requested())
        return -4;

    const WaveQ q_hang_l = -k_wave_hang;
    if (wave_log_over_limit(arm_r, 1, k_wave_hang, " 下垂") != 0 ||
        wave_log_over_limit(arm_l, 0, q_hang_l, " 下垂") != 0)
        return -1;

    std::cout << "[wave] 左手收到下垂\n";
    int rc = wave_goto(arm_l, q_hang_l, -1.0);
    if (rc != 0)
        return rc;
    std::cout << "[wave] 右手收到下垂\n";
    rc = wave_goto(arm_r, k_wave_hang, -1.0);
    if (rc != 0)
        return rc;
    orch::wave_grip_ratio(false, 1.0);
    orch::wave_grip_ratio(true, 1.0);
    std::cout << "[wave] 双手已在下垂\n";
    return hardware_abort_requested() ? -4 : 0;
}

int arm_bow(Robot_Arm &arm_r, Robot_Arm &arm_l)
{
    if (arm_hand_side(arm_r) != 1 || arm_hand_side(arm_l) != 0)
    {
        std::cerr << "[bow] 需要右臂和左臂\n";
        return -1;
    }
    if (right_arm_motors_locked())
    {
        std::cerr << "[bow] 右臂电机锁定，不鞠躬\n";
        return -6;
    }
    if (hardware_abort_requested())
        return -4;

    constexpr double k_bow_slow = 2.0;
    constexpr double k_deg = M_PI / 180.0;
    const WaveQ q_hang_l = -k_wave_hang;
    const WaveQ q_chest = wave_q(0.3856, 1.0966, -0.8351, 1.9461, -0.4402, 0.6800, 0.1030);
    const WaveQ q_via = wave_q(0.18, 1.22, -0.70, 0.95, -0.50, 0.48, 0.10);
    WaveQ q_bow_l = q_hang_l;
    q_bow_l(3) -= 12.0 * k_deg;
    const Eigen::MatrixXd lift = bow_rest_via(
        k_wave_hang, q_via, q_chest, 1.0 * k_bow_slow, 0.8 * k_bow_slow);
    const Eigen::MatrixXd lower = bow_rest_via(
        q_chest, q_via, k_wave_hang, 0.9 * k_bow_slow, 1.1 * k_bow_slow);
    if (wave_log_over_limit(arm_r, 1, k_wave_hang, " 下垂") != 0 ||
        wave_log_over_limit(arm_r, 1, q_via, " 身前") != 0 ||
        wave_log_over_limit(arm_r, 1, q_chest, " 胸前") != 0 ||
        wave_log_over_limit(arm_l, 0, q_hang_l, " 下垂") != 0 ||
        wave_log_over_limit(arm_l, 0, q_bow_l, " 随腰弯肘") != 0 ||
        wave_check_traj(arm_r, 1, lift, " 收到胸前") != 0 ||
        wave_check_traj(arm_r, 1, lower, " 放回下垂") != 0)
        return -1;

    int rc = wave_head_zero();
    if (rc != 0)
        return rc;
    std::cout << "[bow] 双手进入下垂，速度按 " << k_bow_slow << " 倍放慢\n";
    rc = wave_goto(arm_l, q_hang_l, -1.0);
    if (rc != 0)
        return rc;
    rc = wave_goto(arm_r, k_wave_hang, -1.0);
    if (rc != 0)
        return rc;
    orch::wave_grip_ratio(false, 1.0);
    orch::wave_grip_ratio(true, 0.2);
    rc = wave_pause_ms(static_cast<int>(std::lround(200.0 * k_bow_slow)));
    if (rc != 0)
        return rc;

    std::cout << "[bow] 右手经身前收到胸前\n";
    rc = wave_play(1, lift, false);
    if (rc != 0)
        return rc;
    rc = wave_pause_ms(static_cast<int>(std::lround(250.0 * k_bow_slow)));
    if (rc != 0)
        return rc;

    const BowWaist w0 = bow_read_waist();
    BowWaist w1 = w0;
    w1.ankle += 5.0 * k_deg;
    w1.knee += -3.0 * k_deg;
    w1.hip += -27.0 * k_deg;
    w1.roll = 0.0;
    w1.yaw = 0.0;
    const double pitch0 = (-w0.ankle - w0.knee - w0.hip) / k_deg;
    const double pitch1 = (-w1.ankle - w1.knee - w1.hip) / k_deg;
    std::cout << std::fixed << std::setprecision(1)
              << "[bow] 腰前倾 " << (pitch1 - pitch0)
              << "°（现 " << pitch0 << "° → " << pitch1
              << "°），下去 " << (1.2 * k_bow_slow) << "s，停 "
              << (0.7 * k_bow_slow) << "s，起来 " << (1.4 * k_bow_slow) << "s\n";
    constexpr double k_head0 = 10.0;
    constexpr double k_head1 = 18.0;
    rc = bow_play(q_chest, q_chest, q_hang_l, q_bow_l, w0, w1, k_head0, k_head1,
                  1.2 * k_bow_slow);
    if (rc != 0)
        return rc;
    rc = bow_play(q_chest, q_chest, q_bow_l, q_bow_l, w1, w1, k_head1, k_head1,
                  0.7 * k_bow_slow);
    if (rc != 0)
        return rc;
    rc = bow_play(q_chest, q_chest, q_bow_l, q_hang_l, w1, w0, k_head1, k_head0,
                  1.4 * k_bow_slow);
    if (rc != 0)
        return rc;

    std::cout << "[bow] 右手放回下垂\n";
    rc = wave_play(1, lower, false);
    if (rc != 0)
        return rc;
    orch::wave_grip_ratio(true, 1.0);
    std::cout << "[bow] 完成，双手停在下垂\n";
    return hardware_abort_requested() ? -4 : 0;
}

int arm_welcome(Robot_Arm &arm_r, Robot_Arm &arm_l)
{
    if (arm_hand_side(arm_r) != 1 || arm_hand_side(arm_l) != 0)
    {
        std::cerr << "[welcome] 需要右臂和左臂\n";
        return -1;
    }
    if (right_arm_motors_locked())
    {
        std::cerr << "[welcome] 右臂电机锁定，不迎宾\n";
        return -6;
    }
    if (hardware_abort_requested())
        return -4;

    const WaveQ q_hang_l = -k_wave_hang;
    int rc = wave_head_zero();
    if (rc != 0)
        return rc;
    std::cout << "[welcome] 双手进入下垂准备，夹爪闭合\n";
    rc = wave_goto(arm_l, q_hang_l, -1.0);
    if (rc != 0)
        return rc;
    rc = wave_goto(arm_r, k_wave_hang, -1.0);
    if (rc != 0)
        return rc;
    orch::wave_grip_ratio(false, 1.0);
    orch::wave_grip_ratio(true, 1.0);
    if (hardware_abort_requested())
        return -4;

    for (int round = 1;; ++round)
    {
        std::cout << "[welcome] 第 " << round << " 轮 挥右手\n";
        rc = wave_one(arm_r, true);
        if (rc != 0)
            return rc;
        rc = wave_pause_ms(1000);
        if (rc != 0)
            return rc;
        std::cout << "[welcome] 第 " << round << " 轮 挥左手\n";
        rc = wave_one(arm_l, false);
        if (rc != 0)
            return rc;
        rc = wave_pause_ms(1000);
        if (rc != 0)
            return rc;
    }
}
