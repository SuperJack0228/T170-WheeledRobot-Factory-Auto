#pragma once
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "client/cmd_future.hpp"

// 包含公共客户端接口（单client版本）
#include "client/common/client_common.h"

namespace sensor {

// ============================================
// 单 Client 版本 API（无 handle 参数）
// 用于只连接单个服务器
// ============================================

// handle（单client版本，无handle参数）
// weight有多个设备实例
using Handleweight = int;
// 获取所有实例的 handle（单client版本）
std::vector<Handleweight> GetAllHandleweight();

// MotorInfoPub有多个设备实例
using HandleMotorInfoPub = int;
// 获取所有实例的 handle（单client版本）
std::vector<HandleMotorInfoPub> GetAllHandleMotorInfoPub();

// RPC/Pub函数声明（包含结构体定义）
enum WheelPlacement {
  kUnknow = 0,
  kLeft = 1,
  kRight = 2,
  kLift = 3,
  kRotate = 4
};

struct WeightInfo {
  // 总重量 单位kg
  float total_weight;
};

struct MotorInfo {
  WheelPlacement placement;
  // 总重量 单位kg
  double current;
  // 温度(摄氏度)
  double temperature;
};

// 称重传感器数据

void single_weight(Handleweight id,
                   const std::function<void(WeightInfo&&)>& cb);

// 电机电流 单位ma

void motor(HandleMotorInfoPub id, const std::function<void(MotorInfo&&)>& cb);

// Action 相关定义（包含结构体定义）

// 注册所有 action 完成回调处理器（单 client 版本）

}  // namespace sensor
