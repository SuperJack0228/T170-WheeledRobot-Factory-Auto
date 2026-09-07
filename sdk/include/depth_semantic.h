#pragma once
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "client/cmd_future.hpp"

// 包含公共客户端接口（单client版本）
#include "client/common/client_common.h"

namespace depth_semantic {

// ============================================
// 单 Client 版本 API（无 handle 参数）
// 用于只连接单个服务器
// ============================================

// handle（单client版本，无handle参数）
// DepthSemanticDev有多个设备实例
using HandleDepthSemanticDev = int;
// 获取所有实例的 handle（单client版本）
std::vector<HandleDepthSemanticDev> GetAllHandleDepthSemanticDev();

// RPC/Pub函数声明（包含结构体定义）
struct Mat {
  // 图像 高
  int32_t rows = 0;
  // 图像 宽
  int32_t cols = 0;
  // 通道数 1=灰度 3=RGB
  int32_t channels = 1;
  // 图像实际数据
  std::vector<uint8_t> data;
};

/*
    坐标结构体
*/
struct BoundingBox {
  int32_t x1;
  int32_t y1;
  int32_t x2;
  int32_t y2;
};

struct PointXYZ {
  float x;
  float y;
  float z;
};

struct PointCloud {
  std::vector<PointXYZ> points;
};

struct RecognitionResult {
  // 时间戳 单位 us
  uint64_t timestamp_us;
  // AI识别结果图
  Mat original_img;
  // 图中所有类别结果列表
  std::vector<std::string> class_names;
  // 每个物体的框在图像中的坐标
  std::vector<BoundingBox> obj_box;
  // 每个物体最近的点的距离，如果没有点则为-99
  std::vector<float> obj_dis;
  // 点云结构体
  PointCloud pcloud;
};

/*
 @brief 实时获取语义识别结果
 @param cb: 回调函数, 函数接收一个RecognitionResult作为输入
*/
void recognition_state(HandleDepthSemanticDev id,
                       const std::function<void(RecognitionResult&&)>& cb);

/*
 @brief 设置rgb是否读取
 @param bool:  开关
*/
void set_rgb_enable(HandleDepthSemanticDev id, bool value);

// Action 相关定义（包含结构体定义）

// 注册所有 action 完成回调处理器（单 client 版本）

}  // namespace depth_semantic
