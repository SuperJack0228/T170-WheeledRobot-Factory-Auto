#pragma once
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "client/cmd_future.hpp"

// 包含公共客户端接口（单client版本）
#include "client/common/client_common.h"

namespace nav {

// ============================================
// 单 Client 版本 API（无 handle 参数）
// 用于只连接单个服务器
// ============================================

// handle（单client版本，无handle参数）
// 检查设备是否存在（单client版本）
bool HasHandleNavDev();

// RPC/Pub函数声明（包含结构体定义）
struct Vector3 {
  double x;
  double y;
  double z;
};

struct Twist {
  // linear (m/s)
  Vector3 linear;
  // angular (rad/s)
  Vector3 angular;
};

/*
 @brief 定距移动参数
 @param double move_dist: 移动距离(m)
 @param double rotate_angle: 移动角度(°)
*/
struct MoveRelativeInfo {
  double move_dist;
  double rotate_angle;
};

struct ResultMoveRelative {
  // 错误码，默认0表示成功
  int32_t err_code;
  // 错误消息
  std::string err_message;
  // 返回数据
  int32_t data;
};

/*
 @brief 设置当前速度的百分比
 @param double: 速度百分比 (0~1)
*/
bool ctrl_speed_factor(double speed_factor);

/*
 @brief 开启SDK控制速度
*/
bool set_third_speed_ctrl();

/*
 @brief 关闭SDK控制速度
*/
bool close_third_speed_ctrl();

/*
 @brief 下发SDK速度 (下发频率大于20hz)
 @param Twist: 速度
*/
bool send_third_expected_speed(const Twist& third_expected_speed);

// Action 相关定义（包含结构体定义）
/*
 @brief 定距移动
 @param MoveRelativeInfo: 定距移动参数
 @return 导航任务结果码, 0表示成功, 非0表示失败/取消
*/
// 异步执行 action，返回 future
CmdFuture<ResultMoveRelative> moveRelative(const MoveRelativeInfo& arg1);
// 异步执行 action，带回调（用于 Python 绑定，避免创建线程）
uint64_t moveRelative_with_cb(const MoveRelativeInfo& arg1,
                              std::function<void(ResultMoveRelative)> callback);
// 取消 cmd
void cancel_moveRelative(uint64_t task_id);

// 注册所有 action 完成回调处理器（单 client 版本）

}  // namespace nav
