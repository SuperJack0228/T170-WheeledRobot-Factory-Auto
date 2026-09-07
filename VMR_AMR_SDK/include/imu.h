#pragma once
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "client/cmd_future.hpp"

// 包含公共客户端接口（单client版本）
#include "client/common/client_common.h"

namespace imu {

// ============================================
// 单 Client 版本 API（无 handle 参数）
// 用于只连接单个服务器
// ============================================

// handle（单client版本，无handle参数）
// ImuDev有多个设备实例
using HandleImuDev = int;
// 获取所有实例的 handle（单client版本）
std::vector<HandleImuDev> GetAllHandleImuDev();

// RPC/Pub函数声明（包含结构体定义）
enum VmrImuType { kROS = 0, kHipnuc = 1, kMid360 = 2 };

struct Quaternion {
  double x;
  double y;
  double z;
  double w;
};

struct Vector3 {
  double x;
  double y;
  double z;
};

struct VmrImuInfo {
  uint64_t timestamp_ns;
  VmrImuType type;
  // 姿态四元数
  Quaternion orientation;
  // Angular velocity (rad/s)
  Vector3 angular_velocity;
  // Linear acceleration (m/s²)
  Vector3 linear_acceleration;
};

/*
 @brief 实时获取IMU数据状态
 @param cb: 回调函数, 函数接收一个ImuInfo作为输入
*/
void imu_state(HandleImuDev id, const std::function<void(VmrImuInfo&&)>& cb);

// Action 相关定义（包含结构体定义）

// 注册所有 action 完成回调处理器（单 client 版本）

}  // namespace imu
