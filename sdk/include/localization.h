#pragma once
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "client/cmd_future.hpp"

// 包含公共客户端接口（单client版本）
#include "client/common/client_common.h"

namespace localization {

// ============================================
// 单 Client 版本 API（无 handle 参数）
// 用于只连接单个服务器
// ============================================

// handle（单client版本，无handle参数）
// 检查设备是否存在（单client版本）
bool HasHandleLocationDev();

// RPC/Pub函数声明（包含结构体定义）
struct ReLocationInfo {
  // Timestamp (nanoseconds)
  uint64_t timestamp_ns;
  // Localization coordinate x (meters)
  float x;
  // Localization coordinate y (meters)
  float y;
  // Localization angle (radians)
  float theta;
};

struct LocationInfo {
  // Timestamp (nanoseconds)
  uint64_t timestamp_ns;
  // Localization coordinate x (meters)
  float x;
  // Localization coordinate y (meters)
  float y;
  // Localization angle (radians)
  float theta;
  // Confidence level 0~2.5
  float confidence;
  /*
      Localization status
      status       Meaning
      0            Normal
      1            Relocalization failed
      2            Slippage
      6            Relocalizing
      Relocalization Information
    */
  int32_t status;
};

/*
 @brief 实时获取定位状态,  使用回调函数获取
 @param cb: 回调函数, 接受一个 LocationInfo 参数
*/
void loc_state(const std::function<void(LocationInfo&&)>& cb);

void relocate(const ReLocationInfo& loc_info);

// Action 相关定义（包含结构体定义）

// 注册所有 action 完成回调处理器（单 client 版本）

}  // namespace localization
