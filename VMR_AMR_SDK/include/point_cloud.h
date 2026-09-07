#pragma once
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "client/cmd_future.hpp"

// 包含公共客户端接口（单client版本）
#include "client/common/client_common.h"

namespace point_cloud {

// ============================================
// 单 Client 版本 API（无 handle 参数）
// 用于只连接单个服务器
// ============================================

// handle（单client版本，无handle参数）
// 检查设备是否存在（单client版本）
bool HasHandlePointCloudDev();

// RPC/Pub函数声明（包含结构体定义）
/*
 @brief 基于小车坐标系的点云坐标
 @param float x: x轴坐标  单位m
 @param float x: y轴坐标  单位m
 @param float x: z轴坐标  单位m
*/
struct PointXYZ {
  float x;
  float y;
  float z;
};

/*
 @brief 基于小车坐标系的点云坐标
 @param int32_t location: 点云器件标识  (-1 ~ -8 为深度相机  -9 为前3d激光  -10
 为后3d激光)
 @param std::vector<PointXYZ> points: 点云数组
*/
struct PointCloud {
  int32_t location;
  uint64_t timestamp_ns;
  std::vector<PointXYZ> points;
};

/*
 @brief 实时获取点云状态,  使用回调函数获取
 @param cb: 回调函数, 接受一个 PointCloud 参数
*/
void point_cloud_pub(const std::function<void(PointCloud&&)>& cb);

// Action 相关定义（包含结构体定义）

// 注册所有 action 完成回调处理器（单 client 版本）

}  // namespace point_cloud
