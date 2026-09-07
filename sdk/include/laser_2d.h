#pragma once
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "client/cmd_future.hpp"

// 包含公共客户端接口（单client版本）
#include "client/common/client_common.h"

namespace laser_2d {

// ============================================
// 单 Client 版本 API（无 handle 参数）
// 用于只连接单个服务器
// ============================================

// handle（单client版本，无handle参数）
// Laser2DDev有多个设备实例
using HandleLaser2DDev = int;
// 获取所有实例的 handle（单client版本）
std::vector<HandleLaser2DDev> GetAllHandleLaser2DDev();

// RPC/Pub函数声明（包含结构体定义）
struct Header {
  uint32_t seq;
  uint64_t timestamp_ns;
  std::string frame_id;
};

struct BasicLaserScan {
  Header header;
  // Start radian (-π~π)
  float angle_min;
  // End radian (-π~π)
  float angle_max;
  // Angular resolution (radians/sample)
  float angle_increment;
  // Time interval (seconds/sample)
  float time_increment;
  // Total scanning time (seconds)
  float scan_time;
  // Minimum effective distance (meters)
  float range_min;
  // Maximum effective distance (meters)
  float range_max;
  // Distance data (meters)
  std::vector<float> ranges;
  // Intensity data (optional)
  std::vector<float> intensities;
};

void laser_scan(HandleLaser2DDev id,
                const std::function<void(BasicLaserScan&&)>& cb);

// Action 相关定义（包含结构体定义）

// 注册所有 action 完成回调处理器（单 client 版本）

}  // namespace laser_2d
