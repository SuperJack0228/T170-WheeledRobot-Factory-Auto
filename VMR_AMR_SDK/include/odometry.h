#pragma once
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "client/cmd_future.hpp"

// 包含公共客户端接口（单client版本）
#include "client/common/client_common.h"

namespace odometry {

// ============================================
// 单 Client 版本 API（无 handle 参数）
// 用于只连接单个服务器
// ============================================

// handle（单client版本，无handle参数）
// 检查设备是否存在（单client版本）
bool HasHandleOdomDev();

// RPC/Pub函数声明（包含结构体定义）
/*
 @brief 里程计信息
 @param uint64_t timestamp_ns: Timestamp (nanoseconds)
 @param float x: Odometry coordinate x (meters)
 @param float y: Odometry coordinate y (meters)
 @param float theta: Odometry angle (radians)
 @param Linear velocity in x direction (m/s)
 @param Linear velocity in y direction (m/s)
 @param Angular velocity (rad/s)
*/
struct OdomInfo {
  // Timestamp (nanoseconds)
  uint64_t timestamp_ns;
  // Odometry coordinate x (meters)
  float x;
  //  Odometry coordinate y (meters)
  float y;
  // Odometry angle (radians)
  float theta;
  // Linear velocity in x direction (m/s)
  float vx;
  // Linear velocity in y direction (m/s)
  float vy;
  // Angular velocity (rad/s)
  float w;
};

/*
 @brief 实时获取里程计状态,  使用回调函数获取
 @param cb: 回调函数, 接受一个 OdomInfo 参数
*/
void odom_state(const std::function<void(OdomInfo&&)>& cb);

// Action 相关定义（包含结构体定义）

// 注册所有 action 完成回调处理器（单 client 版本）

}  // namespace odometry
