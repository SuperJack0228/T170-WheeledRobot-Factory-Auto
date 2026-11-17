#pragma once

#include <linux/can.h>

#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

/// 组包参数非法或拆包失败时抛出。SDK 不收发，没有总线超时。
struct Error : std::runtime_error {
    enum class Kind { Encode, Decode };

    Kind kind;
    Error(Kind k, const char* msg) : std::runtime_error(msg), kind(k) {}
    Error(Kind k, const std::string& msg) : std::runtime_error(msg), kind(k) {}
};

// 位置 rad、速度 rad/s、电流 A，均为输出轴。
// can_id 为 1–127。换算缓存按 ID 下标直取。
// 换算缓存只由 decode_* / set_gear_ratio / set_eight_byte_control_mode 写入；未解过对应回帧会抛 Decode。
// MIT 须已 decode 或 set `0x71=0`。控制环不要调 `classify_eight_byte_feedback`，按已缓存模式直接 decode_csp / decode_mit。

// ── DLC=1：首字节为指令码 ───────────────────────────────────
can_frame set_enable(uint32_t can_id);                              // 0x01
can_frame set_disable(uint32_t can_id);                             // 0x02
can_frame query_run_mode(uint32_t can_id);                          // 0x03
int32_t decode_run_mode(const can_frame& rx);
can_frame query_current(uint32_t can_id);                           // 0x04
double decode_current(const can_frame& rx);
can_frame query_velocity(uint32_t can_id);                          // 0x06
double decode_velocity(const can_frame& rx);
can_frame query_position(uint32_t can_id);                          // 0x08
int32_t decode_position_cnt(const can_frame& rx);
double decode_position(const can_frame& rx);
can_frame query_error(uint32_t can_id);                             // 0x0A
uint32_t decode_error_status(const can_frame& rx);
std::vector<std::string> decode_error(uint32_t error_code);
can_frame clear_error(uint32_t can_id);                             // 0x0B
can_frame set_save_parameters(uint32_t can_id);                     // 0x0E
can_frame query_speed_kp(uint32_t can_id);                          // 0x10
int32_t decode_speed_kp(const can_frame& rx);
can_frame query_speed_ki(uint32_t can_id);                          // 0x11
int32_t decode_speed_ki(const can_frame& rx);
can_frame query_position_kp(uint32_t can_id);                       // 0x12
int32_t decode_position_kp(const can_frame& rx);
can_frame query_position_kd(uint32_t can_id);                       // 0x13
int32_t decode_position_kd(const can_frame& rx);
can_frame query_bus_voltage(uint32_t can_id);                       // 0x14
int32_t decode_bus_voltage(const can_frame& rx);                    // 单位 V
can_frame query_max_accel(uint32_t can_id);                         // 0x16
double decode_max_accel(const can_frame& rx);
can_frame query_min_accel(uint32_t can_id);                         // 0x17
double decode_min_accel(const can_frame& rx);
can_frame query_max_speed_limit(uint32_t can_id);                   // 0x18
double decode_max_speed_limit(const can_frame& rx);
can_frame query_min_speed_limit(uint32_t can_id);                   // 0x19
double decode_min_speed_limit(const can_frame& rx);
can_frame query_max_position_limit(uint32_t can_id);                // 0x1A
double decode_max_position_limit(const can_frame& rx);
can_frame query_min_position_limit(uint32_t can_id);                // 0x1B
double decode_min_position_limit(const can_frame& rx);
can_frame query_motor_temperature(uint32_t can_id);                 // 0x31
int32_t decode_motor_temperature(const can_frame& rx);
can_frame query_driver_temperature(uint32_t can_id);                // 0x32
int32_t decode_driver_temperature(const can_frame& rx);
can_frame query_speed_kd(uint32_t can_id);                          // 0x33
int32_t decode_speed_kd(const can_frame& rx);
can_frame query_position_ki(uint32_t can_id);                       // 0x34
int32_t decode_position_ki(const can_frame& rx);
can_frame query_max_current_limit(uint32_t can_id);                 // 0x35
double decode_max_current_limit(const can_frame& rx);
can_frame query_min_current_limit(uint32_t can_id);                 // 0x36
double decode_min_current_limit(const can_frame& rx);
can_frame query_max_absolute_current(uint32_t can_id);              // 0x37
double decode_max_absolute_current(const can_frame& rx);
can_frame query_csp(uint32_t can_id);                               // 0x41
/// 只看该 ID 已缓存的 `0x71`（须先 `decode` 或 `set_eight_byte_control_mode`），不看帧布局。
enum class EightByteFeedbackKind { Csp, Mit };
EightByteFeedbackKind classify_eight_byte_feedback(const can_frame& rx);
/// 回帧 DLC=8，PVI：(位置 rad, 速度 rad/s, 电流 A)。须已是非 MIT（`0x71=1`）。`set_*_get_csp` 的回复也走这里。
std::tuple<double, double, double> decode_csp(const can_frame& rx);
can_frame query_position_offset(uint32_t can_id);                   // 0x54
int32_t decode_position_offset(const can_frame& rx);
can_frame query_motor_model(uint32_t can_id);                       // 0x64
int32_t decode_motor_model(const can_frame& rx);
can_frame query_software_version(uint32_t can_id);                  // 0x65
int32_t decode_software_version(const can_frame& rx);
can_frame query_hardware_version(uint32_t can_id);                  // 0x66
int32_t decode_hardware_version(const can_frame& rx);
can_frame query_encoder_mode(uint32_t can_id);                      // 0x70
int32_t decode_encoder_mode(const can_frame& rx);
can_frame query_eight_byte_control_mode(uint32_t can_id);           // 0x71
int32_t decode_eight_byte_control_mode(const can_frame& rx);
can_frame query_encoder_battery_voltage(uint32_t can_id);           // 0x78
int32_t decode_encoder_battery_voltage(const can_frame& rx);
can_frame query_current_kp(uint32_t can_id);                        // 0x85
int32_t decode_current_kp(const can_frame& rx);
can_frame query_current_ki(uint32_t can_id);                        // 0x86
int32_t decode_current_ki(const can_frame& rx);
can_frame query_max_voltage(uint32_t can_id);                       // 0x8A
int32_t decode_max_voltage(const can_frame& rx);
can_frame query_min_voltage(uint32_t can_id);                       // 0x8C
int32_t decode_min_voltage(const can_frame& rx);
can_frame query_max_motor_temperature(uint32_t can_id);             // 0x8F
int32_t decode_max_motor_temperature(const can_frame& rx);
can_frame query_max_driver_temperature(uint32_t can_id);            // 0x93
int32_t decode_max_driver_temperature(const can_frame& rx);
can_frame query_outer_encoder_single_turn(uint32_t can_id);         // 0x96，DLC=1
int32_t decode_outer_encoder_single_turn(const can_frame& rx);
can_frame query_outer_encoder_multi_turn(uint32_t can_id);          // 0x97，DLC=1；与死区 0x97 同码
int32_t decode_outer_encoder_multi_turn(const can_frame& rx);

// ── DLC=5：指令码 + int32 小端 ───────────────────────────────
can_frame set_current(uint32_t can_id, double current_a);           // 0x1C
/// 写入 `out[0, n)`，`n == can_ids.size()`。`out.size()` 须 ≥ n，否则 Encode。
std::size_t set_current(std::span<const uint32_t> can_ids, std::span<const double> currents_a,
                        std::span<can_frame> out);
can_frame set_velocity(uint32_t can_id, double velocity_rad_s);     // 0x1D
std::size_t set_velocity(std::span<const uint32_t> can_ids, std::span<const double> velocities_rad_s,
                         std::span<can_frame> out);
can_frame set_position(uint32_t can_id, double position_rad);       // 0x1E
std::size_t set_position(std::span<const uint32_t> can_ids, std::span<const double> positions_rad,
                         std::span<can_frame> out);
can_frame set_max_current_limit(uint32_t can_id, double current_a);  // 0x20
can_frame set_min_current_limit(uint32_t can_id, double current_a);  // 0x21
can_frame set_max_accel(uint32_t can_id, double accel_rad_s2);       // 0x22
can_frame set_min_accel(uint32_t can_id, double accel_rad_s2);       // 0x23
can_frame set_max_speed_limit(uint32_t can_id, double velocity_rad_s); // 0x24
can_frame set_min_speed_limit(uint32_t can_id, double velocity_rad_s); // 0x25
can_frame set_max_position_limit(uint32_t can_id, double position_rad); // 0x26
can_frame set_min_position_limit(uint32_t can_id, double position_rad); // 0x27
can_frame set_speed_kp(uint32_t can_id, int32_t value);              // 0x29
can_frame set_speed_ki(uint32_t can_id, int32_t value);              // 0x2A
can_frame set_position_kp(uint32_t can_id, int32_t value);           // 0x2B
can_frame set_position_ki(uint32_t can_id, int32_t value);           // 0x2C
can_frame set_position_kd(uint32_t can_id, int32_t value);           // 0x2D
can_frame set_can_new_id(uint32_t can_id, int32_t new_id);           // 0x2E
can_frame set_speed_kd(uint32_t can_id, int32_t value);              // 0x33
can_frame set_max_absolute_current(uint32_t can_id, double current_a); // 0x37
can_frame set_current_get_csp(uint32_t can_id, double current_a);    // 0x42
std::size_t set_current_get_csp(std::span<const uint32_t> can_ids, std::span<const double> currents_a,
                                std::span<can_frame> out);
can_frame set_velocity_get_csp(uint32_t can_id, double velocity_rad_s); // 0x43
std::size_t set_velocity_get_csp(std::span<const uint32_t> can_ids, std::span<const double> velocities_rad_s,
                                 std::span<can_frame> out);
can_frame set_position_get_csp(uint32_t can_id, double position_rad); // 0x44
std::size_t set_position_get_csp(std::span<const uint32_t> can_ids, std::span<const double> positions_rad,
                                 std::span<can_frame> out);
/// 绝对写入。新 offset = 原 offset + 当前反馈 cnt（`0x08`）。
can_frame set_position_offset(uint32_t can_id, int32_t offset_cnt);  // 0x53
can_frame set_pt_position(uint32_t can_id, double position_rad);     // 0x5A
can_frame set_eight_byte_control_mode(uint32_t can_id, int32_t mode); // 0x71，0=MIT；组包当下改缓存
can_frame set_current_kp(uint32_t can_id, int32_t value);            // 0x83
can_frame set_current_ki(uint32_t can_id, int32_t value);            // 0x84

// ── DLC=6：指令码 + 0x20写/0x40读 + 参数 ─────────────────────
can_frame set_gear_ratio(uint32_t can_id, int32_t ratio);            // 0x41
can_frame query_gear_ratio(uint32_t can_id);
int32_t decode_gear_ratio(const can_frame& rx);
can_frame set_pt_kp_max(uint32_t can_id, float kp);                  // 0x42
can_frame query_pt_kp_max(uint32_t can_id);
float decode_pt_kp_max(const can_frame& rx);
can_frame set_pt_kd_max(uint32_t can_id, float kd);                  // 0x43
can_frame query_pt_kd_max(uint32_t can_id);
float decode_pt_kd_max(const can_frame& rx);
can_frame set_pt_kt(uint32_t can_id, float kt);                      // 0x44
can_frame query_pt_kt(uint32_t can_id);
float decode_pt_kt(const can_frame& rx);
can_frame set_pt_torque_min(uint32_t can_id, float torque_nm);       // 0x46
can_frame query_pt_torque_min(uint32_t can_id);
float decode_pt_torque_min(const can_frame& rx);
can_frame set_pt_torque_max(uint32_t can_id, float torque_nm);       // 0x47
can_frame query_pt_torque_max(uint32_t can_id);
float decode_pt_torque_max(const can_frame& rx);
can_frame set_pt_current_min(uint32_t can_id, float current_a);      // 0x48
can_frame query_pt_current_min(uint32_t can_id);
float decode_pt_current_min(const can_frame& rx);
can_frame set_pt_current_max(uint32_t can_id, float current_a);      // 0x49
can_frame query_pt_current_max(uint32_t can_id);
float decode_pt_current_max(const can_frame& rx);
/// 位置环 PID 死区。协议 Int32，表未给单位；enable 一般为 0/1。0x97 与外圈多圈查询同码，靠 DLC=6 区分。
can_frame set_position_pid_deadzone_enable(uint32_t can_id, int32_t enable);  // 0x97
can_frame query_position_pid_deadzone_enable(uint32_t can_id);
int32_t decode_position_pid_deadzone_enable(const can_frame& rx);
can_frame set_position_pid_deadzone_min(uint32_t can_id, int32_t value);       // 0x98
can_frame query_position_pid_deadzone_min(uint32_t can_id);
int32_t decode_position_pid_deadzone_min(const can_frame& rx);
can_frame set_position_pid_deadzone_max(uint32_t can_id, int32_t value);       // 0x99
can_frame query_position_pid_deadzone_max(uint32_t can_id);
int32_t decode_position_pid_deadzone_max(const can_frame& rx);
can_frame query_position_control_mode(uint32_t can_id);              // 0x9D
int32_t decode_position_control_mode(const can_frame& rx);

// ── DLC=7 ────────────────────────────────────────────────────
can_frame set_pt_position_and_velocity(uint32_t can_id, double position_rad, double velocity_rad_s); // 0x02，速度 0.01 rad/s
can_frame set_position_with_current_feedforward(uint32_t can_id, double position_rad, double feedforward_a); // 0x10
can_frame set_position_with_velocity_feedforward(uint32_t can_id, double position_rad, double feedforward_rad_s,
                                                 double feedforward_scale = 1.0); // 0x11

// ── DLC=8：无命令字 MIT；CSP 三合一回帧见上面的 decode_csp（查询码 0x41）──

/// 力矩/电流量化量程随电机型号变化；位置、速度、Kp、Kd 的协议量程在库内，不对外。
/// 默认 ±18 表示「用缓存」。未手传量程时须先 `decode_pt_current_*` / `decode_pt_torque_*` 或 `cache_mit_limits`。
struct MitLimits {
    float torque_min{-18.0f};
    float torque_max{18.0f};
    float current_min{-18.0f};
    float current_max{18.0f};
};

void cache_mit_limits(uint32_t can_id, const MitLimits& limits);
MitLimits cached_mit_limits(uint32_t can_id);

can_frame set_mit(uint32_t can_id, double position_rad, double velocity_rad_s, double torque_nm, float kp, float kd,
                  const MitLimits& limits = {});

/// 前三项与 `decode_csp` 对齐：(位置 rad, 速度 rad/s, 电流 A, motor_error, 线圈℃, 驱动板℃)。
/// 须已是 MIT（`0x71=0`）。电流按电机 Imin/Imax 解，与 CSP 同为 Iq（A），只差 12bit 量化。
std::tuple<double, double, double, uint8_t, double, double> decode_mit(const can_frame& rx,
                                                                      const MitLimits& limits = {});
