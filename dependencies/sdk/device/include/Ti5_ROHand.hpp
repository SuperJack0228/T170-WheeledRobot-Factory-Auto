#pragma once

#include <array>
#include <cstdint>
#include <tuple>
#include <vector>

/// 参数是设备 id。按配对后的通道和介质组包下发。
/// finger_id：0 拇指 … 5 拇指根。逻辑位置 0～65535；角度=实际×100；speed 0～255。
/// 只发：set_* / finger_start/stop / reset 等，不等回帧。
/// 一问一答：get_*，发 1 等完整 55 AA。失败回零值。
/// 无力控。CAN 发往手 can_id=手 ID；回包 can_id 可能是主机 1。

struct FingerPid {
    float p{};
    float i{};
    float d{};
    float g{};
};

struct FingerStopParams {
    std::uint16_t speed{};
    std::uint16_t stop_current_ma{};
    std::uint16_t stop_after_ms{};
    std::uint16_t retry_interval_ms{};
};

struct SpeedCtrlParams {
    std::uint16_t brake_distance{};
    std::uint16_t accel_distance{};
    std::uint16_t speed_ratio{};
};

void set_Finger_Pos(std::uint8_t id, std::uint8_t finger_id, std::uint16_t pos, std::uint8_t speed);
void set_Finger_PosAbs(std::uint8_t id, std::uint8_t finger_id, std::uint16_t pos, std::uint8_t speed);
void set_Finger_Angle(std::uint8_t id, std::uint8_t finger_id, std::int16_t angle, std::uint8_t speed);
void set_Finger_PosAll(std::uint8_t id, const std::array<std::uint16_t, 6>& pos,
                       const std::array<std::uint8_t, 6>& speed);
void set_Finger_PosAbsAll(std::uint8_t id, const std::array<std::uint16_t, 6>& pos,
                          const std::array<std::uint8_t, 6>& speed);
void set_Finger_AngleAll(std::uint8_t id, const std::array<std::int16_t, 6>& angle,
                         const std::array<std::uint8_t, 6>& speed);
void set_Thumb_RootPos(std::uint8_t id, std::uint8_t preset);
void Finger_Start(std::uint8_t id);
void Finger_Stop(std::uint8_t id);

std::tuple<std::uint16_t, std::uint16_t> get_Finger_Pos(std::uint8_t id, std::uint8_t finger_id);
std::tuple<std::uint16_t, std::uint16_t> get_Finger_PosAbs(std::uint8_t id, std::uint8_t finger_id);
std::tuple<std::int16_t, std::int16_t> get_Finger_Angle(std::uint8_t id, std::uint8_t finger_id);
std::tuple<std::array<std::uint16_t, 6>, std::array<std::uint16_t, 6>> get_Finger_PosAll(std::uint8_t id);
std::tuple<std::array<std::uint16_t, 6>, std::array<std::uint16_t, 6>> get_Finger_PosAbsAll(std::uint8_t id);
std::tuple<std::array<std::int16_t, 6>, std::array<std::int16_t, 6>> get_Finger_AngleAll(std::uint8_t id);
std::uint8_t get_Thumb_RootPos(std::uint8_t id);
std::uint16_t get_Finger_Current(std::uint8_t id, std::uint8_t finger_id);

void set_Finger_CurrentLimit(std::uint8_t id, std::uint8_t finger_id, std::uint16_t ma);
void set_Finger_PosLimit(std::uint8_t id, std::uint8_t finger_id, std::uint16_t pos);
void set_Finger_PID(std::uint8_t id, std::uint8_t finger_id, const FingerPid& pid);
void set_Finger_StopParams(std::uint8_t id, std::uint8_t finger_id, const FingerStopParams& p);
void set_Speed_CtrlParams(std::uint8_t id, const SpeedCtrlParams& p);

std::uint16_t get_Finger_CurrentLimit(std::uint8_t id, std::uint8_t finger_id);
std::uint16_t get_Finger_PosLimit(std::uint8_t id, std::uint8_t finger_id);
FingerPid get_Finger_PID(std::uint8_t id, std::uint8_t finger_id);
FingerStopParams get_Finger_StopParams(std::uint8_t id, std::uint8_t finger_id);
SpeedCtrlParams get_Speed_CtrlParams(std::uint8_t id);

void Reset_Hand(std::uint8_t id, std::uint8_t mode = 0);
void set_Node_ID(std::uint8_t id, std::uint8_t node_id);
void set_SelfTest_Level(std::uint8_t id, std::uint8_t level);
void Start_Init(std::uint8_t id);
void set_Beep_Switch(std::uint8_t id, std::uint8_t on);
void Beep(std::uint8_t id, std::uint16_t period_ms);

std::tuple<std::uint8_t, std::uint8_t> get_Protocol_Version(std::uint8_t id);
std::tuple<std::uint16_t, std::uint8_t, std::uint8_t> get_Fw_Version(std::uint8_t id);
std::tuple<std::uint8_t, std::uint8_t, std::uint8_t, std::uint8_t> get_Hw_Version(std::uint8_t id);
std::tuple<char, char> get_Vendor_ID(std::uint8_t id);
std::tuple<std::uint32_t, std::uint32_t, std::uint32_t> get_UID(std::uint8_t id);
std::vector<std::uint8_t> get_Motor_Status(std::uint8_t id);
std::uint8_t get_SelfTest_Switch(std::uint8_t id);
std::uint8_t get_Beep_Switch(std::uint8_t id);
