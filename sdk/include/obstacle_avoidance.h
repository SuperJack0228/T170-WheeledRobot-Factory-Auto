#pragma once
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "client/cmd_future.hpp"

// 包含公共客户端接口（单client版本）
#include "client/common/client_common.h"

namespace obstacle_avoidance {

// ============================================
// 单 Client 版本 API（无 handle 参数）
// 用于只连接单个服务器
// ============================================

// handle（单client版本，无handle参数）
// 检查设备是否存在（单client版本）
bool HasHandleObstacleAvoidanceDev();

// RPC/Pub函数声明（包含结构体定义）
struct PoseXY {
  float x;
  float y;
};

struct VirtualObstacleInfo {
  // Obstacle ID
  int32_t obstacle_id;
  // Top-left corner
  PoseXY top_left;
  // Bottom-right corner
  PoseXY bottom_right;
};

struct VirtualObstacleInfoList {
  // Virtual Obstacle List
  std::vector<VirtualObstacleInfo> virtual_obstacle_list;
};

/*
 @brief 新增虚拟障碍物
 @param VirtualObstacleInfo: 障碍物信息
*/
bool add_virtual_obstacle(const VirtualObstacleInfo& obstacle_info);

/*
 @brief 删除虚拟障碍物
 @param int32_t: 虚拟障碍物id
*/
bool remove_virtual_obstacle(int32_t obstacle_id);

/*
 @brief 查询虚拟障碍物列表
 @param VirtualObstacleInfoList: 虚拟障碍物列表
*/
VirtualObstacleInfoList query_virtual_obstacle();

// Action 相关定义（包含结构体定义）

// 注册所有 action 完成回调处理器（单 client 版本）

}  // namespace obstacle_avoidance
