#include "head_control.h"

#include "function.h"
#include "move_box_config.h"
#include "Ti5_socketcan.h"
#include "Ti5_Motor.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <unistd.h>

namespace
{

constexpr double kHeadArriveTolDeg = 2.0;
constexpr double kHeadYawRollTolDeg = 1.0;
constexpr double kHeadLimitMarginDeg = 5.0;

bool head_read_cnt(int now_cnt[3])
{
    uint32_t can_id[] = {kHeadCanIds[0], kHeadCanIds[1], kHeadCanIds[2]};
    return socketcan_sendsimplecommand(head_id, 3, can_id, 8, now_cnt) == 1;
}

void head_log_axis(uint32_t id, const char *name)
{
    try
    {
        const bool err_bit = has_Error(id);
        const std::uint32_t err = get_Error_Status(id);
        const std::int32_t run = get_Motor_RunMode(id);
        const double q_deg = get_Position(id) * 180.0 / M_PI;
        const double max_deg = get_Max_Position(id) * 180.0 / M_PI;
        const double min_deg = get_Min_Position(id) * 180.0 / M_PI;
        std::cout << std::fixed << std::setprecision(1)
                  << "[head] " << name << " id=" << id
                  << " q=" << q_deg
                  << " deg  limit=[" << min_deg << "," << max_deg << "]"
                  << " run=" << run
                  << " err=0x" << std::hex << err << std::dec
                  << (err_bit ? " HAS_ERROR" : " ok") << "\n";
    }
    catch (const std::exception &ex)
    {
        std::cerr << "[head] " << name << " id=" << id << " 状态查询失败: " << ex.what() << "\n";
    }
}

void head_ensure_limit_window(uint32_t id, const char *name, double a_deg, double b_deg)
{
    try
    {
        const double lo_deg = std::min(a_deg, b_deg) - kHeadLimitMarginDeg;
        const double hi_deg = std::max(a_deg, b_deg) + kHeadLimitMarginDeg;
        const double lo_rad = lo_deg * M_PI / 180.0;
        const double hi_rad = hi_deg * M_PI / 180.0;
        const double max_rad = get_Max_Position(id);
        const double min_rad = get_Min_Position(id);
        if (hi_rad > max_rad - 1e-4)
        {
            std::cout << std::fixed << std::setprecision(1)
                      << "[head] " << name << " 软件上限 "
                      << (max_rad * 180.0 / M_PI) << "° 盖不住 "
                      << std::max(a_deg, b_deg) << "°，抬到 " << hi_deg << "°\n";
            set_Max_Position(id, hi_rad);
        }
        if (lo_rad < min_rad + 1e-4)
        {
            std::cout << std::fixed << std::setprecision(1)
                      << "[head] " << name << " 软件下限 "
                      << (min_rad * 180.0 / M_PI) << "° 盖不住 "
                      << std::min(a_deg, b_deg) << "°，放到 " << lo_deg << "°\n";
            set_Min_Position(id, lo_rad);
        }
    }
    catch (const std::exception &ex)
    {
        std::cerr << "[head] " << name << " 限位处理失败: " << ex.what() << "\n";
    }
}

bool head_hold_rpy_deg(double yaw_deg, double pitch_deg, double roll_deg)
{
    const double targets[3] = {yaw_deg, pitch_deg, roll_deg};
    bool ok = true;
    for (int i = 0; i < 3; ++i)
    {
        if (!motor_set_pos_rad(kHeadCanIds[static_cast<size_t>(i)],
                               targets[i] * M_PI / 180.0))
        {
            std::cerr << "[head] 锁位置失败 id=" << kHeadCanIds[static_cast<size_t>(i)] << "\n";
            ok = false;
        }
    }
    if (ok)
        std::cout << std::fixed << std::setprecision(1)
                  << "[head] 已锁位置 yaw=" << yaw_deg
                  << " pitch=" << pitch_deg
                  << " roll=" << roll_deg << " deg（用手扳不动才算使能）\n";
    return ok;
}

bool head_enable_motors()
{
    std::cout << "[head] 通道 can" << head_id
              << " 电机 ID " << kHeadYawCanId << "(偏航)/"
              << kHeadPitchCanId << "(俯仰)/" << kHeadRollCanId << "(横滚) 清错并使能\n";
    bool ok = true;
    for (uint32_t id : kHeadCanIds)
    {
        if (!motor_enable_id(id))
        {
            std::cerr << "[head] 使能失败 id=" << id << "\n";
            ok = false;
        }
    }
    int spd[3] = {2500, 2500, 2500};
    int nspd[3] = {-2500, -2500, -2500};
    uint32_t can_id[] = {kHeadCanIds[0], kHeadCanIds[1], kHeadCanIds[2]};
    socketcan_sendcommand(head_id, 3, can_id, 36, spd);
    socketcan_sendcommand(head_id, 3, can_id, 37, nspd);
    usleep(50000);
    can_io_fault_clear();
    head_log_axis(kHeadYawCanId, "偏航");
    head_log_axis(kHeadPitchCanId, "俯仰");
    head_log_axis(kHeadRollCanId, "横滚");
    return ok;
}

bool head_yaw_roll_ok(double yaw_now, double roll_now, double yaw_deg, double roll_deg)
{
    return std::abs(yaw_now - yaw_deg) < kHeadYawRollTolDeg &&
           std::abs(roll_now - roll_deg) < kHeadYawRollTolDeg;
}

bool head_arrived(double yaw_now, double pitch_now, double roll_now,
                  double yaw_deg, double pitch_deg, double roll_deg)
{
    return head_yaw_roll_ok(yaw_now, roll_now, yaw_deg, roll_deg) &&
           std::abs(pitch_now - pitch_deg) < kHeadArriveTolDeg;
}

} // namespace

bool read_head_rpy_deg(double &yaw_deg, double &pitch_deg, double &roll_deg)
{
    if (head_id < 0)
        return false;
    int now_cnt[3] = {};
    if (!head_read_cnt(now_cnt))
        return false;
    yaw_deg = motor_cnt_to_angle_deg(now_cnt[0]);
    pitch_deg = motor_cnt_to_angle_deg(now_cnt[1]);
    roll_deg = motor_cnt_to_angle_deg(now_cnt[2]);
    return true;
}

int move_head_to_rpy_deg(double yaw_deg, double pitch_deg, double roll_deg, bool enable_motors)
{
    if (head_id < 0)
    {
        std::cerr << "[head] CAN 未绑定，无法运动头部\n";
        return -1;
    }
    if (hardware_abort_requested())
        return -4;

    uint32_t can_id[] = {kHeadCanIds[0], kHeadCanIds[1], kHeadCanIds[2]};
    const auto &h = g_move_cfg.head;

    int now_cnt[3] = {};
    auto read_now = [&]() -> bool {
        if (!head_read_cnt(now_cnt))
            return false;
        return true;
    };
    auto decode = [&](double &yaw, double &pitch, double &roll) {
        yaw = motor_cnt_to_angle_deg(now_cnt[0]);
        pitch = motor_cnt_to_angle_deg(now_cnt[1]);
        roll = motor_cnt_to_angle_deg(now_cnt[2]);
    };

    if (enable_motors)
    {
        if (!head_enable_motors())
            std::cerr << "[head] 使能未全部成功，仍尝试走位置\n";
    }

    if (!read_now())
    {
        std::cerr << "[head] 读编码器失败，电机 "
                  << kHeadYawCanId << "/" << kHeadPitchCanId << "/" << kHeadRollCanId
                  << " 无回包\n";
        return -1;
    }
    double yaw_now = 0.0, pitch_now = 0.0, roll_now = 0.0;
    decode(yaw_now, pitch_now, roll_now);

    head_ensure_limit_window(kHeadYawCanId, "偏航", yaw_now, yaw_deg);
    head_ensure_limit_window(kHeadPitchCanId, "俯仰", pitch_now, pitch_deg);
    head_ensure_limit_window(kHeadRollCanId, "横滚", roll_now, roll_deg);

    std::cout << std::fixed << std::setprecision(1)
              << "[head] 当前 yaw=" << yaw_now
              << " pitch=" << pitch_now
              << " roll=" << roll_now
              << " → 目标 " << yaw_deg << "," << pitch_deg << "," << roll_deg << " deg\n";

    if (head_arrived(yaw_now, pitch_now, roll_now, yaw_deg, pitch_deg, roll_deg))
    {
        std::cout << "[head] 已在目标姿态，不下发轨迹，写位置锁住\n";
        if (!head_hold_rpy_deg(yaw_deg, pitch_deg, roll_deg))
            return -1;
        return 0;
    }

    auto do_move = [&](double y, double p, double r) -> int {
        double angles[3] = {y, p, r};
        can_io_fault_clear();
        const int rc = smooth_motor_move_deg(head_id, 3, can_id, angles, h.speed_deg_s);
        if (rc != 0)
        {
            std::cerr << "[head] 走到目标姿态失败 code=" << rc << "\n";
            return rc;
        }
        hardware_abort_sleep_ms(300);
        return hardware_abort_requested() ? -4 : 0;
    };

    const bool need_unroll =
        !head_yaw_roll_ok(yaw_now, roll_now, yaw_deg, roll_deg);
    const bool need_pitch = std::abs(pitch_now - pitch_deg) >= kHeadArriveTolDeg;
    if (need_unroll && need_pitch)
    {
        std::cout << std::fixed << std::setprecision(1)
                  << "[head] 横滚/偏航先回正（当前 roll=" << roll_now
                  << "°），再低头到 " << pitch_deg << "°\n";
        const int rc_level = do_move(yaw_deg, pitch_now, roll_deg);
        if (rc_level != 0)
            return rc_level;
        if (!read_now())
        {
            std::cerr << "[head] 回正后读编码器失败\n";
            return -1;
        }
        decode(yaw_now, pitch_now, roll_now);
        std::cout << std::fixed << std::setprecision(1)
                  << "[head] 回正后 yaw=" << yaw_now
                  << " pitch=" << pitch_now
                  << " roll=" << roll_now << " deg\n";
        if (!head_yaw_roll_ok(yaw_now, roll_now, yaw_deg, roll_deg))
        {
            std::cerr << "[head] 回正未到位，重新使能再走一次 yaw/roll\n";
            if (!head_enable_motors())
                std::cerr << "[head] 重试使能仍有失败\n";
            head_ensure_limit_window(kHeadYawCanId, "偏航", yaw_now, yaw_deg);
            head_ensure_limit_window(kHeadRollCanId, "横滚", roll_now, roll_deg);
            const int rc_retry = do_move(yaw_deg, pitch_now, roll_deg);
            if (rc_retry != 0)
                return rc_retry;
            if (!read_now())
                return -1;
            decode(yaw_now, pitch_now, roll_now);
        }
        if (!head_yaw_roll_ok(yaw_now, roll_now, yaw_deg, roll_deg))
        {
            std::cerr << std::fixed << std::setprecision(1)
                      << "[head] 偏航/横滚未回正 yaw=" << yaw_now
                      << " roll=" << roll_now << "，禁止低头\n";
            return -1;
        }
    }

    int rc = do_move(yaw_deg, pitch_deg, roll_deg);
    if (rc != 0)
        return rc;
    if (!read_now())
    {
        std::cerr << "[head] 运动后读编码器失败\n";
        return -1;
    }
    decode(yaw_now, pitch_now, roll_now);

    if (!head_arrived(yaw_now, pitch_now, roll_now, yaw_deg, pitch_deg, roll_deg))
    {
        std::cerr << std::fixed << std::setprecision(1)
                  << "[head] 第一次未到位 yaw=" << yaw_now
                  << " pitch=" << pitch_now
                  << " roll=" << roll_now
                  << "，重新使能再走一次\n";
        if (!head_enable_motors())
            std::cerr << "[head] 重试使能仍有失败\n";
        head_ensure_limit_window(kHeadYawCanId, "偏航", yaw_now, yaw_deg);
        head_ensure_limit_window(kHeadPitchCanId, "俯仰", pitch_now, pitch_deg);
        head_ensure_limit_window(kHeadRollCanId, "横滚", roll_now, roll_deg);
        rc = do_move(yaw_deg, pitch_deg, roll_deg);
        if (rc != 0)
            return rc;
        if (!read_now())
        {
            std::cerr << "[head] 重试后读编码器失败\n";
            return -1;
        }
        decode(yaw_now, pitch_now, roll_now);
    }

    std::cout << std::fixed << std::setprecision(1)
              << "[head] 编码器 yaw=" << yaw_now
              << " pitch=" << pitch_now
              << " roll=" << roll_now << " deg\n";
    head_log_axis(kHeadYawCanId, "偏航");
    head_log_axis(kHeadPitchCanId, "俯仰");
    head_log_axis(kHeadRollCanId, "横滚");

    if (!head_arrived(yaw_now, pitch_now, roll_now, yaw_deg, pitch_deg, roll_deg))
    {
        std::cerr << "[head] 未到达目标（电机可能未使能或限位卡住），禁止当作到位\n";
        return -1;
    }
    std::cout << "[head] 到位\n";
    if (!head_hold_rpy_deg(yaw_deg, pitch_deg, roll_deg))
        return -1;
    return hardware_abort_requested() ? -4 : 0;
}

int enable_head_calib_pose()
{
    const auto &h = g_move_cfg.head;
    return move_head_to_rpy_deg(h.yaw_deg, h.pitch_deg, h.roll_deg, true);
}

int enable_head_far_pitch()
{
    const auto &h = g_move_cfg.head;
    return move_head_to_rpy_deg(h.yaw_deg, h.far_pitch_deg, h.roll_deg, true);
}
