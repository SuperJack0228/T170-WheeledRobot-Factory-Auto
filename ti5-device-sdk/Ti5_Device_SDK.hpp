#pragma once

#include "Ti5_Dex1.hpp"
#include "Ti5_Motor.hpp"
#include "Ti5_ROHand.hpp"

#include <cstdint>

/// 包含本头后进 main 前按 devices.json 自动配对；电机编解码缓存一并初始化。
/// 已配对再调 pair() 直接返回。close() 之后可以再 pair()。
void pair();
/// 放开本进程持有的通道。未捕获异常进 terminate 时会先走这里。
void close();

bool motor_paired(std::uint32_t id);
bool hand_paired(std::uint8_t id);
bool gripper_paired(std::uint8_t id);
/// 已配对返回 "can" / "serial"，否则空串。
const char* hand_medium(std::uint8_t id);

namespace router_detail {
extern int boot;
#if defined(__GNUC__)
__attribute__((used))
#endif
inline const int boot_ref = boot;
}
