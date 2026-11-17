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
bool arm_motors_at_commanded_position(int hand, int tolerance = kMotorPositionReachTolerance)
{
    if (hand < 0 || hand > 1 || !g_arm_last_cmd_valid[hand])
        return true;

    int actual[7] = {0};
    get_motor_position(actual, hand);
    for (int i = 0; i < 7; ++i)
    {
        if (std::abs(actual[i] - g_arm_last_cmd_cnt[hand][i]) > tolerance)
            return false;
    }
    return true;
}
}


namespace {
void log_arms_motor_position_reach(bool log_r, bool log_l, const char *reason)
{
    std::cout << "[arm] " << reason;
    auto log_one = [](int hand, const char *tag) {
        if (!g_arm_last_cmd_valid[hand])
        {
            std::cout << ' ' << tag << "=no_cmd";
            return;
        }
        int actual[7] = {0};
        get_motor_position(actual, hand);
        std::cout << ' ' << tag << "_err=";
        for (int i = 0; i < 7; ++i)
        {
            std::cout << (actual[i] - g_arm_last_cmd_cnt[hand][i]);
            if (i + 1 < 7)
                std::cout << ',';
        }
    };
    if (log_r)
        log_one(1, "pos_r");
    if (log_l)
        log_one(0, "pos_l");
    std::cout << std::endl << std::flush;
}
}


namespace t170::motion::detail {
Eigen::MatrixXd arm_read_motor_joints(int hand)
{
    int motor_cnt[7] = {0};
    get_motor_position(motor_cnt, hand);

    Eigen::MatrixXd q_start(1, 7);
    for (int i = 0; i < 7; ++i)
        q_start(0, i) = encoder_cnt_to_rad(motor_cnt[i]);
    return q_start;
}
}






void log_arm_encoder_command_vs_actual(Robot_Arm &arm, const char *tag)
{
    const int hand = arm_hand_side(arm);
    const char *name = (tag && tag[0]) ? tag : (hand == 1 ? "右手" : "左手");
    if (hand < 0 || hand > 1 || !g_arm_last_cmd_valid[hand])
    {
        std::cout << "[tray2] " << name << " 没有本次下发的编码器理论值\n" << std::flush;
        return;
    }
    int actual[7] = {};
    get_motor_position(actual, hand);
    std::cout << "[tray2] " << name << " 编码器  理论  实际  差\n";
    for (int i = 0; i < 7; ++i)
    {
        const int theory = g_arm_last_cmd_cnt[hand][i];
        std::cout << "  J" << (i + 1)
                  << "  " << theory
                  << "  " << actual[i]
                  << "  " << (actual[i] - theory) << "\n";
    }
    std::cout << std::flush;
}

Eigen::Matrix<double, 1, 7> arm_get_joint_pos(Robot_Arm &arm)
{
    const Eigen::MatrixXd q = arm_read_motor_joints(arm_hand_side(arm));
    Eigen::Matrix<double, 1, 7> out = Eigen::Matrix<double, 1, 7>::Zero();
    const int n = static_cast<int>(std::min(q.cols(), static_cast<Eigen::Index>(7)));
    for (int i = 0; i < n; ++i)
        out(i) = q(0, i);
    return out;
}

namespace wrist_servo {
bool read_joint_feedback(int hand,Joints &q) {
    uint32_t ids[7]; int counts[7]{};
    const auto *source=hand==1?kRightArmCanIds:kLeftArmCanIds;
    std::copy(source,source+7,ids);
    if(socketcan_sendsimplecommand(0,7,ids,8,counts)!=1) return false;
    for(int j=0;j<7;++j) q(j)=encoder_cnt_to_rad(counts[j]);
    return q.allFinite();
}
}


namespace {
constexpr int kMotorStoppedMinZeroCount = 6;
}


namespace {
constexpr int kEncoderErrStableDelta = 10;
}


namespace {
constexpr int kEncoderStablePlateauPolls = 10;
}


namespace {
constexpr int kEncoderPlateauMaxAbsErr = 2500;
}


namespace {
constexpr int kEncoderPollIntervalMs = 50;
}


namespace {
struct ArmEncoderReachState
{
    int prev_err[7] = {0};
    bool have_prev = false;
    int plateau_count = 0;
};
}


namespace {
struct ArmEncoderReachResult
{
    bool ok = false;
    bool used_plateau = false;
};
}


namespace {
ArmEncoderReachResult check_arm_encoder_reach(int hand, ArmEncoderReachState &st)
{
    ArmEncoderReachResult res;
    if (hand < 0 || hand > 1 || !g_arm_last_cmd_valid[hand])
    {
        res.ok = true;
        return res;
    }

    int actual[7] = {0};
    get_motor_position(actual, hand);
    int err[7] = {0};
    for (int i = 0; i < 7; ++i)
        err[i] = actual[i] - g_arm_last_cmd_cnt[hand][i];

    bool all_strict = true;
    for (int i = 0; i < 7; ++i)
    {
        if (std::abs(err[i]) > kMotorPositionReachTolerance)
            all_strict = false;
    }

    if (all_strict)
    {
        st.plateau_count = 0;
        st.have_prev = false;
        res.ok = true;
        return res;
    }

    // |err|>550 且连续稳定：仅当误差仍在合理带内才视为到位（急停/失能使能后误差很大且不动）。
    bool err_too_large = false;
    for (int i = 0; i < 7; ++i)
    {
        if (std::abs(err[i]) > kEncoderPlateauMaxAbsErr)
            err_too_large = true;
    }
    if (st.have_prev && !err_too_large)
    {
        bool stable = true;
        for (int i = 0; i < 7; ++i)
        {
            if (std::abs(err[i]) > kMotorPositionReachTolerance)
            {
                if (std::abs(err[i] - st.prev_err[i]) > kEncoderErrStableDelta)
                    stable = false;
            }
        }
        if (stable)
            ++st.plateau_count;
        else
            st.plateau_count = 0;
    }
    else
    {
        st.plateau_count = 0;
    }

    for (int i = 0; i < 7; ++i)
        st.prev_err[i] = err[i];
    st.have_prev = true;

    if (st.plateau_count >= kEncoderStablePlateauPolls)
    {
        res.ok = true;
        res.used_plateau = true;
    }
    return res;
}
}


namespace {
bool arms_encoder_reach_ok(
    bool wait_r,
    bool wait_l,
    ArmEncoderReachState &st_r,
    ArmEncoderReachState &st_l,
    bool &used_plateau)
{
    used_plateau = false;
    if (wait_r)
    {
        const auto r = check_arm_encoder_reach(1, st_r);
        if (!r.ok)
            return false;
        if (r.used_plateau)
            used_plateau = true;
    }
    if (wait_l)
    {
        const auto l = check_arm_encoder_reach(0, st_l);
        if (!l.ok)
            return false;
        if (l.used_plateau)
            used_plateau = true;
    }
    return true;
}
}


namespace {

}


namespace {
bool motor_joint_speed_counts_stopped(int speed)
{
    return speed == 0;
}
}


namespace {

}


void log_phase_banner(const char *title)
{
    const char *t = (title != nullptr && title[0] != '\0') ? title : "(未命名阶段)";
    std::cout << kLogPhaseSep << '\n'
              << ">>> " << t << " <<<\n"
              << kLogPhaseSep << std::endl;
}







bool wait_arms_motors_stopped(
    bool wait_r,
    bool wait_l,
    double timeout_sec,
    int min_zero_count,
    const char *log_stage)
{
    if (right_arm_motors_locked())
        wait_r = false;
    if (!wait_r && !wait_l)
        return true;

    const char *stage = (log_stage != nullptr && log_stage[0] != '\0') ? log_stage : "手相机前";

    (void)min_zero_count;

    ArmEncoderReachState enc_r;
    ArmEncoderReachState enc_l;

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(timeout_sec));
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (hardware_abort_requested())
        {
            std::cerr << "[arm] STOP：停稳等待中止\n";
            return false;
        }
        if (can_io_faulted())
        {
            std::cerr << "[arm] CAN 失败：停稳等待中止\n";
            return false;
        }
                bool used_plateau = false;
                if (arms_encoder_reach_ok(wait_r, wait_l, enc_r, enc_l, used_plateau))
                {
                    const std::string ok_msg = used_plateau
                        ? std::string(stage) + "停稳OK(plateau>±550)"
                        : std::string(stage) + "停稳OK(±550)";
                    log_arms_motor_position_reach(wait_r, wait_l, ok_msg.c_str());
                    return true;
                }
        std::this_thread::sleep_for(std::chrono::milliseconds(kEncoderPollIntervalMs));
    }

    const std::string timeout_msg =
        std::string(stage) + "停稳失败(超时, 编码器±550/plateau>±550)";
    log_arms_motor_position_reach(wait_r, wait_l, timeout_msg.c_str());
    std::cerr << "[arm] 等待电机停稳超时 wait_r=" << wait_r << " wait_l=" << wait_l << std::endl;
    return false;
}

namespace {
std::mutex g_arm_arrival_mu;
}


namespace {
std::vector<ArmArrivalRecord> g_arm_arrivals;
}


namespace {
constexpr std::size_t kArmArrivalLogMax = 2000;
}


void arm_arrival_log_clear()
{
    std::lock_guard<std::mutex> lock(g_arm_arrival_mu);
    g_arm_arrivals.clear();
}

std::vector<ArmArrivalRecord> arm_arrival_log_copy()
{
    std::lock_guard<std::mutex> lock(g_arm_arrival_mu);
    return g_arm_arrivals;
}

bool record_arm_cartesian_arrival(
    Robot_Arm &arm,
    const Eigen::Matrix<double, 1, 6> &goal,
    const char *stage,
    double settle_sec)
{
    const int hand = arm_hand_side(arm);
    if (hand == 1 && right_arm_motors_locked())
        return false;

    std::string stage_txt;
    if (stage != nullptr && stage[0] != '\0')
        stage_txt = stage;
    else
    {
        std::lock_guard<std::mutex> lock(g_arm_line_move_stage_mu);
        stage_txt = (g_arm_line_move_debug_stage != nullptr && g_arm_line_move_debug_stage[0] != '\0')
                        ? g_arm_line_move_debug_stage
                        : "直线";
    }

    const bool is_right = hand == 1;
    const bool settled = wait_arms_motors_stopped(
        is_right, !is_right, settle_sec, 6, stage_txt.c_str());
    const Eigen::Matrix<double, 1, 6> actual = arm_get_tcp_pos(arm);

    ArmArrivalRecord rec;
    rec.stage = stage_txt;
    rec.hand = is_right ? "右" : "左";
    rec.goal_x = goal(0);
    rec.goal_y = goal(1);
    rec.goal_z = goal(2);
    rec.actual_x = actual(0);
    rec.actual_y = actual(1);
    rec.actual_z = actual(2);
    rec.err_x = actual(0) - goal(0);
    rec.err_y = actual(1) - goal(1);
    rec.err_z = actual(2) - goal(2);
    rec.err_m = std::sqrt(rec.err_x * rec.err_x + rec.err_y * rec.err_y + rec.err_z * rec.err_z);
    rec.settled = settled;

    {
        std::lock_guard<std::mutex> lock(g_arm_arrival_mu);
        if (g_arm_arrivals.size() < kArmArrivalLogMax)
            g_arm_arrivals.push_back(rec);
    }

    std::cout << std::fixed << std::setprecision(4)
              << "[arm] 到位 " << rec.hand << " " << rec.stage
              << " 目标xyz=(" << rec.goal_x << "," << rec.goal_y << "," << rec.goal_z
              << ") 实际xyz=(" << rec.actual_x << "," << rec.actual_y << "," << rec.actual_z
              << ") 误差xyz=(" << rec.err_x << "," << rec.err_y << "," << rec.err_z
              << ") |err|=" << rec.err_m << "m"
              << (settled ? " 停稳" : " 未停稳") << "\n";
    std::cout.flush();
    return settled;
}
