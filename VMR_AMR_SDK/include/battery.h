#pragma once
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "client/cmd_future.hpp"

// 包含公共客户端接口（单client版本）
#include "client/common/client_common.h"

namespace battery {

// ============================================
// 单 Client 版本 API（无 handle 参数）
// 用于只连接单个服务器
// ============================================

// handle（单client版本，无handle参数）
// BatteryDev有多个设备实例
using HandleBatteryDev = int;
// 获取所有实例的 handle（单client版本）
std::vector<HandleBatteryDev> GetAllHandleBatteryDev();

// RPC/Pub函数声明（包含结构体定义）
enum ChargeStatus {
  CHARGE_STATUS_NONE = 0,
  CHARGE_STATUS_CHARGING = 1,
  CHARGE_STATUS_FULL = 2
};

struct BatteryState {
  // Timestamp (nanoseconds)
  uint64_t timestamp_ns;
  // Voltage (Volts)
  float voltage = 0.0;
  // Current (Amperes)
  float current = 0.0;
  // Current charge on battery (mAh)
  float charge = 0.0;
  // Battery capacity (mAh)
  float capacity = 0.0;
  //  Rated battery capacity (mAh)
  float design_capacity = 0.0;
  // Power percentage
  float percentage = 0.0;
  // Charge Status
  ChargeStatus power_supply_status;
  // Battery temperature, unit: °C
  double battery_temp;
  // Battery Id
  std::string battery_id = "";
};

/*
 @brief 获取电池状态, 使用回调函数获取
 @param cb: 回调函数, 函数接收一个BatteryState作为输入
*/
void battery_state(HandleBatteryDev id,
                   const std::function<void(BatteryState&&)>& cb);

// Action 相关定义（包含结构体定义）

// 注册所有 action 完成回调处理器（单 client 版本）

}  // namespace battery
