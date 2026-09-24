#pragma once

#include "Ti5_Motor_codec.h"

#include <cstdint>
#include <span>
#include <string>
#include <vector>

/// 参数是设备 id。命令里：motor 表取通道和介质，再编解码收发。
/// 位置 rad、速度 rad/s、电流 A，均为输出轴。
/// 八字节回帧按已缓存的 0x71 解，不猜布局。配对后已自动 motor_Init。
/// CSP：position_rad / velocity_rad_s / current_a。须 0x71=非 MIT。未到为零。
/// MIT：(position_rad, velocity_rad_s, current_a, motor_error, 线圈℃, 驱动板℃)。须 0x71=0。

// ── 启动 ────────────────────────────────────────────────────
/// 读减速比、编码器模式、0x71、MIT 量程，写入编解码缓存。配对后会自动调用。
void motor_Init(std::uint32_t id);

/// MIT 量化量程。未手传时须先 get_PT_Current_* / get_PT_Torque_* 或本函数。
void cache_Mit_Limits(std::uint32_t id, const MitLimits& limits);
MitLimits cached_Mit_Limits(std::uint32_t id);

// ── 系统 ────────────────────────────────────────────────────
void Motor_Enable(std::uint32_t id);
void Motor_Disable(std::uint32_t id);
void Save_Parameters(std::uint32_t id);
/// 协议改从站地址。
void set_Can_ID(std::uint32_t id, std::int32_t new_id);

std::int32_t get_Motor_RunMode(std::uint32_t id);
std::int32_t get_Encoder_Mode(std::uint32_t id);
bool is_Dual_Encoder(std::uint32_t id);
void set_Eight_Bit_Mode(std::uint32_t id, std::int32_t mode);
std::int32_t get_Eight_Bit_Mode(std::uint32_t id);
std::int32_t get_Position_Control_Mode(std::uint32_t id);

// ── 报错 ────────────────────────────────────────────────────
std::uint32_t get_Error_Status(std::uint32_t id);
void Clear_Error(std::uint32_t id);
bool has_Error(std::uint32_t id);
std::vector<std::string> explain_Error(std::uint32_t error_code);
std::vector<std::string> get_Error_Messages(std::uint32_t id);

// ── 运动（单轴）─────────────────────────────────────────────
// 只发：set_Current / set_Speed / set_Position 等，不等回帧。
// 一问一答：get_*，发 1 等 1。
// CSP / MIT 反馈写入调用方变量，避免热路径 return 堆分配。收到返回 true，未到写 0。
void set_Current(std::uint32_t id, double current_a);
void set_Speed(std::uint32_t id, double velocity_rad_s);
void set_Position(std::uint32_t id, double position_rad);
bool set_Current_get_CSP(std::uint32_t id, double current_a, double& position_rad, double& velocity_rad_s,
                         double& current_a_fb, int timeout_ms = 5);
bool set_Speed_get_CSP(std::uint32_t id, double velocity_rad_s, double& position_rad, double& velocity_rad_s_fb,
                       double& current_a, int timeout_ms = 5);
bool set_Position_get_CSP(std::uint32_t id, double position_rad, double& position_rad_fb, double& velocity_rad_s,
                          double& current_a, int timeout_ms = 5);

void set_PT_Position(std::uint32_t id, double position_rad);
void set_PT_Position_and_Speed(std::uint32_t id, double position_rad, double velocity_rad_s);
void set_Position_Feedforward_Current(std::uint32_t id, double position_rad, double feedforward_a);
void set_Position_Feedforward_Speed(std::uint32_t id, double position_rad, double feedforward_rad_s,
                                    double feedforward_scale = 1.0);

/// 须已配置 0x71=0。limits 默认用缓存。发 MIT 等回帧；收到返回 true，未到写 0。
bool set_MIT_get_Feedback(std::uint32_t id, double position_rad, double velocity_rad_s, double torque_nm, float kp,
                         float kd, double& position_rad_fb, double& velocity_rad_s_fb, double& current_a,
                         std::uint8_t& motor_error, double& coil_c, double& board_c, const MitLimits& limits = {},
                         int timeout_ms = 5);

double get_Current(std::uint32_t id);
double get_Speed(std::uint32_t id);
double get_Position(std::uint32_t id);
std::int32_t get_Position_Cnt(std::uint32_t id);
bool get_CSP(std::uint32_t id, double& position_rad, double& velocity_rad_s, double& current_a);

// ── 运动（多轴，ids 与设定 / 反馈等长）──────────────────────
// set_*_get_CSP / get_CSP / set_MIT_get_Feedback：连发后按 id 批收齐；整批一个超时。
// 反馈写入调用方 span，与 ids 等长；未到写 0。返回收到的轴数。
void set_Current(std::span<const std::uint32_t> ids, std::span<const double> currents_a);
void set_Speed(std::span<const std::uint32_t> ids, std::span<const double> velocities_rad_s);
void set_Position(std::span<const std::uint32_t> ids, std::span<const double> positions_rad);
std::size_t set_Current_get_CSP(std::span<const std::uint32_t> ids, std::span<const double> currents_a,
                                std::span<double> position_rad, std::span<double> velocity_rad_s,
                                std::span<double> current_a, int timeout_ms = 5);
std::size_t set_Speed_get_CSP(std::span<const std::uint32_t> ids, std::span<const double> velocities_rad_s,
                              std::span<double> position_rad, std::span<double> velocity_rad_s,
                              std::span<double> current_a, int timeout_ms = 5);
std::size_t set_Position_get_CSP(std::span<const std::uint32_t> ids, std::span<const double> positions_rad,
                                 std::span<double> position_rad, std::span<double> velocity_rad_s,
                                 std::span<double> current_a, int timeout_ms = 5);
std::size_t get_CSP(std::span<const std::uint32_t> ids, std::span<double> position_rad,
                    std::span<double> velocity_rad_s, std::span<double> current_a, int timeout_ms = 5);
std::size_t set_MIT_get_Feedback(std::span<const std::uint32_t> ids, std::span<const double> positions_rad,
                                 std::span<const double> velocities_rad_s, std::span<const double> torques_nm,
                                 std::span<const float> kp, std::span<const float> kd,
                                 std::span<double> position_rad, std::span<double> velocity_rad_s,
                                 std::span<double> current_a, std::span<std::uint8_t> motor_error,
                                 std::span<double> coil_c, std::span<double> board_c, const MitLimits& limits = {},
                                 int timeout_ms = 5);

// ── 减速比 ──────────────────────────────────────────────────
void set_Gear_Ratio(std::uint32_t id, std::int32_t ratio);
std::int32_t get_Gear_Ratio(std::uint32_t id);

// ── Pt 参数 ─────────────────────────────────────────────────
void set_PT_KP_Max(std::uint32_t id, float kp);
void set_PT_KD_Max(std::uint32_t id, float kd);
void set_PT_KT(std::uint32_t id, float kt);
void set_PT_Torque_Min(std::uint32_t id, float torque_nm);
void set_PT_Torque_Max(std::uint32_t id, float torque_nm);
void set_PT_Current_Min(std::uint32_t id, float current_a);
void set_PT_Current_Max(std::uint32_t id, float current_a);

float get_PT_KP_Max(std::uint32_t id);
float get_PT_KD_Max(std::uint32_t id);
float get_PT_KT(std::uint32_t id);
float get_PT_Torque_Min(std::uint32_t id);
float get_PT_Torque_Max(std::uint32_t id);
float get_PT_Current_Min(std::uint32_t id);
float get_PT_Current_Max(std::uint32_t id);

// ── PID ─────────────────────────────────────────────────────
void set_Speed_KP(std::uint32_t id, std::int32_t value);
void set_Speed_KI(std::uint32_t id, std::int32_t value);
void set_Speed_KD(std::uint32_t id, std::int32_t value);
void set_Position_KP(std::uint32_t id, std::int32_t value);
void set_Position_KI(std::uint32_t id, std::int32_t value);
void set_Position_KD(std::uint32_t id, std::int32_t value);
void set_Current_KP(std::uint32_t id, std::int32_t value);
void set_Current_KI(std::uint32_t id, std::int32_t value);

std::int32_t get_Speed_KP(std::uint32_t id);
std::int32_t get_Speed_KI(std::uint32_t id);
std::int32_t get_Speed_KD(std::uint32_t id);
std::int32_t get_Position_KP(std::uint32_t id);
std::int32_t get_Position_KI(std::uint32_t id);
std::int32_t get_Position_KD(std::uint32_t id);
std::int32_t get_Current_KP(std::uint32_t id);
std::int32_t get_Current_KI(std::uint32_t id);

void set_Position_PID_Deadzone_Enable(std::uint32_t id, std::int32_t enable);
void set_Position_PID_Deadzone_Min(std::uint32_t id, std::int32_t value);
void set_Position_PID_Deadzone_Max(std::uint32_t id, std::int32_t value);
std::int32_t get_Position_PID_Deadzone_Enable(std::uint32_t id);
std::int32_t get_Position_PID_Deadzone_Min(std::uint32_t id);
std::int32_t get_Position_PID_Deadzone_Max(std::uint32_t id);

// ── 限幅 ────────────────────────────────────────────────────
void set_Max_ACC(std::uint32_t id, double accel_rad_s2);
void set_Min_ACC(std::uint32_t id, double accel_rad_s2);
void set_Max_Speed(std::uint32_t id, double velocity_rad_s);
void set_Min_Speed(std::uint32_t id, double velocity_rad_s);
void set_Max_Position(std::uint32_t id, double position_rad);
void set_Min_Position(std::uint32_t id, double position_rad);
void set_Max_Current(std::uint32_t id, double current_a);
void set_Min_Current(std::uint32_t id, double current_a);
void set_AbsLimit_Current(std::uint32_t id, double current_a);

double get_Max_ACC(std::uint32_t id);
double get_Min_ACC(std::uint32_t id);
double get_Max_Speed(std::uint32_t id);
double get_Min_Speed(std::uint32_t id);
double get_Max_Position(std::uint32_t id);
double get_Min_Position(std::uint32_t id);
double get_Max_Current(std::uint32_t id);
double get_Min_Current(std::uint32_t id);
double get_AbsLimit_Current(std::uint32_t id);

// ── 零位 / 偏移（须下使能）──────────────────────────────────
void set_Position_Offset(std::uint32_t id, std::int32_t offset_cnt);
/// 读当前 cnt，写 offset = 原 offset + 当前 cnt。
void set_Zero(std::uint32_t id);
std::int32_t get_Position_Offset(std::uint32_t id);

// ── 设备信息 ────────────────────────────────────────────────
std::int32_t get_Bus_Voltage(std::uint32_t id);
std::int32_t get_Max_Voltage(std::uint32_t id);
std::int32_t get_Min_Voltage(std::uint32_t id);
std::int32_t get_Motor_Temperature(std::uint32_t id);
std::int32_t get_Board_Temperature(std::uint32_t id);
std::int32_t get_Motor_Max_Temperature(std::uint32_t id);
std::int32_t get_Board_Max_Temperature(std::uint32_t id);
std::int32_t get_Motor_Model(std::uint32_t id);
std::int32_t get_Software_Version(std::uint32_t id);
std::int32_t get_Hardware_Version(std::uint32_t id);
std::int32_t get_Encoder_Voltage(std::uint32_t id);
std::int32_t get_Outside_Encoder_Cycle(std::uint32_t id);
std::int32_t get_Outside_Encoder_Cycles(std::uint32_t id);
