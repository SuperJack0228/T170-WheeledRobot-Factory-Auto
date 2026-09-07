#pragma once
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "client/cmd_future.hpp"

// 包含公共客户端接口（单client版本）
#include "client/common/client_common.h"

namespace qrcamera {

// ============================================
// 单 Client 版本 API（无 handle 参数）
// 用于只连接单个服务器
// ============================================

// handle（单client版本，无handle参数）
// 检查设备是否存在（单client版本）
bool HasHandleQRCameraDev();

// RPC/Pub函数声明（包含结构体定义）
/*
 @brief 二维码相对位姿
 @param float x: x 方向偏移, 单位 m
 @param float y: y 方向偏移, 单位 m
 @param float theta: 航向角偏移, 单位 rad
*/
struct QRPose2D {
  float x = 0.0;
  float y = 0.0;
  float theta = 0.0;
};

/*
 @brief 二维码相机识别结果
 @param uint64_t timestamp_ns: 时间戳 ns
 @param int32_t location: 相机安装位置 上相机为0 下相机为1
 @param bool detected: 是否识别到二维码
 @param std::string qr_info: 二维码完整内容
 @param std::string qr_type: 二维码类型
 @param std::string qr_id: 二维码 ID
 @param QRPose2D base_in_qr: 车身在二维码坐标系下的位姿
 @param QRPose2D camera_in_qr: 相机在二维码坐标系下的位姿
 @param float depth: 二维码深度, 单位 m
 @param int32_t memb_total_cnt: 二维码组合总成员数
 @param int32_t memb_obs_cnt: 当前识别到的成员数
 @param int32_t tag_type: 标签类型, 0=DataMatrix, 1=Apriltag
 @param int32_t encode_type: 编码类型
*/
struct QRCameraResult {
  uint64_t timestamp_ns;
  int32_t location;
  bool detected = false;
  std::string qr_info;
  std::string qr_type;
  std::string qr_id;
  QRPose2D base_in_qr;
  QRPose2D camera_in_qr;
  float depth = 0.0;
  int32_t memb_total_cnt = 0;
  int32_t memb_obs_cnt = 0;
  int32_t tag_type = 0;
  int32_t encode_type = 0;
};

/*
 @brief 二维码相机开关请求
 @param bool enable: true 打开二维码识别, false 关闭二维码识别
 @param int32_t location: 相机安装位置 上相机为0 下相机为1
*/
struct QRCameraSwitchRequest {
  bool enable = false;
  int32_t location = 1;
};

/*
 @brief 实时获取二维码相机识别结果
 @param cb: 回调函数, 接收 QRCameraResult 参数
*/
void qrcamera_result(const std::function<void(QRCameraResult&&)>& cb);

/*
 @brief 开关二维码相机识别
 @param QRCameraSwitchRequest: 二维码相机开关请求
*/
bool set_qrcamera_enable(const QRCameraSwitchRequest& request);

// Action 相关定义（包含结构体定义）

// 注册所有 action 完成回调处理器（单 client 版本）

}  // namespace qrcamera
