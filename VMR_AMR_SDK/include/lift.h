#pragma once
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "client/cmd_future.hpp"

// 包含公共客户端接口（单client版本）
#include "client/common/client_common.h"

namespace lift {

// ============================================
// 单 Client 版本 API（无 handle 参数）
// 用于只连接单个服务器
// ============================================

// handle（单client版本，无handle参数）
// 检查设备是否存在（单client版本）
bool HasHandleLiftDev();

// RPC/Pub函数声明（包含结构体定义）
/*
 @brief 顶升设备状态
 @param timestamp_ns: 时间戳 ns
 @param lift_status: 顶升高度 单位 mm
*/
struct LiftInfo {
  // Timestamp (nanoseconds)
  uint64_t timestamp_ns;
  // 顶升高度 单位 mm
  int32_t lift_status;
};

struct ResultLiftCtr {
  // 错误码，默认0表示成功
  int32_t err_code;
  // 错误消息
  std::string err_message;
  // 返回数据
  int32_t data;
};

/*
 @brief 获取顶升设备状态, 服务端发布和客户端订阅, 使用回调函数获取
 @param cb: 回调函数, 函数接收一个LiftInfo作为输入
*/
void lift_state(const std::function<void(LiftInfo&&)>& cb);

// Action 相关定义（包含结构体定义）
/*
 @brief 下达顶升设备控制命令
 @param LiftInfo: 顶升设备控制命令，通过设置 lift_status 字段指定目标状态
 @return 返回执行结果 顶升高度
*/
// 异步执行 action，返回 future
CmdFuture<ResultLiftCtr> lift_ctr(const LiftInfo& arg1);
// 异步执行 action，带回调（用于 Python 绑定，避免创建线程）
uint64_t lift_ctr_with_cb(const LiftInfo& arg1,
                          std::function<void(ResultLiftCtr)> callback);
// 取消 cmd
void cancel_lift_ctr(uint64_t task_id);

// 注册所有 action 完成回调处理器（单 client 版本）

}  // namespace lift
