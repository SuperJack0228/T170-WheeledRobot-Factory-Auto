#pragma once

#include <cstdint>

/// 参数是总线 id：0 右 / 1 左；15 广播无回包。量在输出侧（开合）。
/// 全开约 5 rad，全闭 0 rad，以现场标定为准。只串口 6 Mbps。
/// 只发：set_* / Brake / Calibrate，不等回帧。
/// 一问一答：set_*_get_State / get_State，发 20 等 26。收到返回 true，未到写 0。

void set_Gripper_Pos(std::uint8_t id, float q, float dq = 0, float tau = 0, float kp = 2.0f, float kd = 0.1f);
void set_Gripper_Brake(std::uint8_t id);
void set_Gripper_Calibrate(std::uint8_t id);

bool set_Gripper_Pos_get_State(std::uint8_t id, float q, float dq, float tau, float kp, float kd, float& q_fb,
                               float& dq_fb, float& tau_fb, int& temp_shell, int& temp_winding, float& voltage,
                               std::uint32_t& merror, int timeout_ms = 20);
bool set_Gripper_Brake_get_State(std::uint8_t id, float& q, float& dq, float& tau, int& temp_shell, int& temp_winding,
                                 float& voltage, std::uint32_t& merror, int timeout_ms = 20);
bool get_Gripper_State(std::uint8_t id, float& q, float& dq, float& tau, int& temp_shell, int& temp_winding,
                       float& voltage, std::uint32_t& merror, int timeout_ms = 20);
