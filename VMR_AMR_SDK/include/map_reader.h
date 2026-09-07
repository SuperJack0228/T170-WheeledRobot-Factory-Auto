#pragma once
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "client/cmd_future.hpp"

// 包含公共客户端接口（单client版本）
#include "client/common/client_common.h"

namespace map_reader {

// ============================================
// 单 Client 版本 API（无 handle 参数）
// 用于只连接单个服务器
// ============================================

// handle（单client版本，无handle参数）
// 检查设备是否存在（单client版本）
bool HasHandleMapReader();

// RPC/Pub函数声明（包含结构体定义）
// 拓扑点

struct Goal {
  // 地图点id
  int64_t id;
  // 点位[x,y] 单位m
  std::array<float, 2> center;
  // 点位名称
  std::string name;
};

// 路径

struct Line {
  // 路径id
  int64_t id;
  // 起点名称
  std::string start_name;
  // 终点名称
  std::string end_name;
};

struct MapPoints {
  // 所有点
  std::vector<Goal> goals;
  // 所有路径
  std::vector<Line> lines;
};

// 获取当前地图的所有点位 和 路径

MapPoints get_current_map_info();

// Action 相关定义（包含结构体定义）

// 注册所有 action 完成回调处理器（单 client 版本）

}  // namespace map_reader
