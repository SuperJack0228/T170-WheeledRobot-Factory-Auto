#pragma once
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "client/cmd_future.hpp"

// 包含公共客户端接口（单client版本）
#include "client/common/client_common.h"

namespace robot_task {

// ============================================
// 单 Client 版本 API（无 handle 参数）
// 用于只连接单个服务器
// ============================================

// handle（单client版本，无handle参数）
// 检查设备是否存在（单client版本）
bool HasHandleRobotTaskDev();

// RPC/Pub函数声明（包含结构体定义）
/*
 @brief 地图点位
 @param double x: 地图x轴 单位 m
 @param double y: 地图y轴 单位 m
 @param double theta: 姿态 单位 弧度
*/
struct RobotPose {
  double x;
  double y;
  double theta;
};

/*
 @brief 拓扑点位任务
 @param std::string pose_name: 点位名
 @param int32_t action: 动作 id  默认0 无动作仅移动  3:充电  4:停止充电
*/
struct TopoPose {
  std::string pose_name;
  int32_t action;
};

/*
 @brief 充电任务
 @param result: 执行结果 1为成功 3为取消  其余都是失败
*/
struct CmdResult {
  int32_t result;
};

struct ResultChargeTask {
  // 错误码，默认0表示成功
  int32_t err_code;
  // 错误消息
  std::string err_message;
  // 返回数据
  CmdResult data;
};

struct ResultTopoMoveTask {
  // 错误码，默认0表示成功
  int32_t err_code;
  // 错误消息
  std::string err_message;
  // 返回数据
  CmdResult data;
};

/*
 @brief 控制充电继电器开关
 @param bool: 控制充电继电器  true为开 false为关
 @return bool: 是否成功
*/
bool charge_relay(bool relay_state);

// Action 相关定义（包含结构体定义）
/*
 @brief 下发充电任务 小车会根据自由导航 直接到点充电
 @param RobotPose: 地图上充电点位
 @return 返回一个 SimpleNamespace 对象，包含以下属性：
         - id: 命令唯一标识符
         - future: 异步 future 可通过 await 等待执行结果
         - cancel: 可调用对象（callable），调用后尝试取消该命令
*/
// 异步执行 action，返回 future
CmdFuture<ResultChargeTask> charge_task(const RobotPose& arg1);
// 异步执行 action，带回调（用于 Python 绑定，避免创建线程）
uint64_t charge_task_with_cb(const RobotPose& arg1,
                             std::function<void(ResultChargeTask)> callback);
// 取消 cmd
void cancel_charge_task(uint64_t task_id);

/*
 @brief 下发拓扑路径移动任务 小车会根据地图规划拓扑路径 执行任务
 任务会以队列形式执行
 @param TopoPose: 地图点位
 @return 返回一个 SimpleNamespace 对象，包含以下属性：
         - id: 命令唯一标识符
         - future: 异步 future 可通过 await 等待执行结果
         - cancel: 可调用对象（callable），调用后取消所有拓扑移动任务
*/
// 异步执行 action，返回 future
CmdFuture<ResultTopoMoveTask> topo_move_task(const TopoPose& arg1);
// 异步执行 action，带回调（用于 Python 绑定，避免创建线程）
uint64_t topo_move_task_with_cb(
    const TopoPose& arg1, std::function<void(ResultTopoMoveTask)> callback);
// 取消 cmd
void cancel_topo_move_task(uint64_t task_id);

// 注册所有 action 完成回调处理器（单 client 版本）

}  // namespace robot_task
