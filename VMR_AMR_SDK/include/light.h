#pragma once
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "client/cmd_future.hpp"

// 包含公共客户端接口（单client版本）
#include "client/common/client_common.h"

namespace light {

// ============================================
// 单 Client 版本 API（无 handle 参数）
// 用于只连接单个服务器
// ============================================

// handle（单client版本，无handle参数）
// 检查设备是否存在（单client版本）
bool HasHandleLightDev();

// RPC/Pub函数声明（包含结构体定义）
enum LightState {
  WHITE_BOOTING = 0,
  BLINK_LEFT_BLUE = 1,
  BLINK_RIGHT_BLUE = 2,
  PURPLE_TRAFFIC = 3,
  RED_SAFE_STOP = 4,
  YELLOW_WARNING = 5,
  YELLOW_LOW_POWER = 6,
  RED_EXCEPTION = 7,
  GREEN_CHARGING = 8,
  RED_SELF_FAULT = 9,
  OFF = 10,
  BLUE_IDLE = 11,
  BLUE_RUNNING = 12,
  WHITE_MANUAL = 13
};

struct LightInfo {
  // Timestamp (nanoseconds)
  uint64_t timestamp_ns;
  // Light status
  LightState light_status;
};

/*
 @brief 获取灯带状态 使用回调函数获取
 @param cb: 回调函数, LightInfo
*/
void light_state(const std::function<void(LightInfo&&)>& cb);

/*
 @brief 设置灯带状态
 @param LightInfo: 灯带状态
*/
void ctr_light(const LightInfo& light_info);

// Action 相关定义（包含结构体定义）

// 注册所有 action 完成回调处理器（单 client 版本）

}  // namespace light
