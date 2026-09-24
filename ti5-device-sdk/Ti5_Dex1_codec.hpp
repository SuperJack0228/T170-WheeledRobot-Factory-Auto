#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

/// Dex1-1 / M4010 HG 组解帧。不收发、不打开串口。
/// 字段缩放与官方 libUnitreeMotorSDK 的 modify_data / extract_data 对齐。

constexpr float kGearRatio = 25.0f;
constexpr std::size_t kCmdSize = 20;
constexpr std::size_t kStateSize = 26;
constexpr std::uint8_t kCmdHead0 = 0xFE;
constexpr std::uint8_t kCmdHead1 = 0xEE;
constexpr std::uint8_t kStateHead0 = 0xFC;
constexpr std::uint8_t kStateHead1 = 0xEE;

enum class Mode : std::uint8_t {
    Brake = 0,
    Foc = 1,
    Calibrate = 2,
};

/// 转子侧，与官方 MotorCmd 同语义。
struct RotorCmd {
    std::uint8_t id = 0;  // 0 右 / 1 左；15 广播无回包
    Mode mode = Mode::Foc;
    bool timeout = false;  // 1：置超时保护位；清故障时发 0
    float tau = 0;         // N·m
    float dq = 0;          // rad/s
    float q = 0;           // rad
    float kp = 0;          // 官方写入 k_pos 的值（不是 0~1 物理量）
    float kd = 0;
};

/// 输出侧（夹爪开合）。encode 时 q/dq *= gear，kp/kd /= gear²，tau /= gear。
struct OutCmd {
    std::uint8_t id = 0;
    Mode mode = Mode::Foc;
    bool timeout = false;
    float tau = 0;
    float dq = 0;
    float q = 0;
    float kp = 0;
    float kd = 0;
};

struct RotorState {
    std::uint8_t id = 0;
    Mode mode = Mode::Brake;
    bool timeout = false;
    float tau = 0;
    float dq = 0;
    float q = 0;
    int temp_shell = 0;     // ℃
    int temp_winding = 0;   // ℃
    float voltage = 0;      // V，原始 vol/2
    std::uint32_t merror = 0;
};

struct OutState {
    std::uint8_t id = 0;
    Mode mode = Mode::Brake;
    bool timeout = false;
    float tau = 0;  // 转子力矩 × 减速比（不取负；ROS 节点 effort 为 -tau）
    float dq = 0;
    float q = 0;
    int temp_shell = 0;
    int temp_winding = 0;
    float voltage = 0;
    std::uint32_t merror = 0;
};

/// 组 20 字节控制帧。id>15 返回 0。
std::size_t encode(const RotorCmd& cmd, std::span<std::uint8_t> out);
std::size_t encode(const OutCmd& cmd, std::span<std::uint8_t> out);

/// 解 26 字节反馈。头或 CRC 不对返回 false。
bool decode(std::span<const std::uint8_t> rx, RotorState& out);
bool decode(std::span<const std::uint8_t> rx, OutState& out);

/// 从缓冲看应丢掉多少字节。0 = 还不全。
/// 头在偏移 0 且已有 26 字节 → 返回 26（调用方 decode 后丢掉这帧）。
/// 头在偏移 i>0 → 返回 i。找不到头 → 丢掉多余字节，必要时留最后 1 字节（可能是 FC）。
std::size_t packet_complete(std::span<const std::uint8_t> buf);
