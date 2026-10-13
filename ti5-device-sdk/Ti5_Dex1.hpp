#pragma once

#include <cstdint>

/// 参数是总线 id：0 右 / 1 左；15 广播无回包。量在输出侧（开合）。
/// 全开约 5 rad，全闭 0 rad，以现场标定为准。只串口 6 Mbps。
/// 只发：set_* / Brake / Calibrate / clear_Gripper_Error，不等回帧。
/// 一问一答：set_*_get_State / get_State，发 20 等 26。收到返回 true，未到写 0。
/// get_Gripper_State 协议没有只读查询，会再发 Brake 收状态。
/// timeout_protect：true 置超时保护位；清故障用 clear_Gripper_Error（发 timeout=0）。

enum class GripperMode : std::uint8_t { Brake = 0, Foc = 1, Calibrate = 2 };

struct GripperState {
    std::uint8_t id{};
    GripperMode mode{GripperMode::Brake};
    bool timeout{};
    float q{};
    float dq{};
    float tau{};
    int temp_shell{};
    int temp_winding{};
    float voltage{};
    std::uint32_t merror{};
};

void set_Gripper_Pos(std::uint8_t id, float q, float dq = 0, float tau = 0, float kp = 2.0f, float kd = 0.1f,
                     bool timeout_protect = false);
void set_Gripper_Brake(std::uint8_t id, bool timeout_protect = false);
void set_Gripper_Calibrate(std::uint8_t id, bool timeout_protect = false);
/// 发 Brake 且 timeout=0，清超时保护 / 故障位。
void clear_Gripper_Error(std::uint8_t id);

bool set_Gripper_Pos_get_State(std::uint8_t id, float q, float dq, float tau, float kp, float kd, GripperState& st,
                               int timeout_ms = 20, bool timeout_protect = false);
bool set_Gripper_Brake_get_State(std::uint8_t id, GripperState& st, int timeout_ms = 20,
                                 bool timeout_protect = false);
bool set_Gripper_Calibrate_get_State(std::uint8_t id, GripperState& st, int timeout_ms = 20,
                                     bool timeout_protect = false);
bool get_Gripper_State(std::uint8_t id, GripperState& st, int timeout_ms = 20);
