#pragma once
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "client/cmd_future.hpp"

// 包含公共客户端接口（单client版本）
#include "client/common/client_common.h"

namespace exception {

// ============================================
// 单 Client 版本 API（无 handle 参数）
// 用于只连接单个服务器
// ============================================

// handle（单client版本，无handle参数）
// 检查设备是否存在（单client版本）
bool HasHandleExceptionDev();

// RPC/Pub函数声明（包含结构体定义）
enum ExceptionLevel { NORMAL = 0, STOP = 1, EMERGENCY_STOP = 2 };

struct Exception {
  // exception moudle
  int32_t exception_moudle;
  // exception code
  int32_t exception_code;
};

struct ExceptionsInfo {
  // Timestamp (nanoseconds)
  uint64_t timestamp_ns;
  ExceptionLevel robot_exception;
  // Exception status
  std::vector<Exception> exceptions;
};

/*
 @brief 实时获取异常状态
 @param cb: 回调函数, 函数接收一个ExceptionsInfo作为输入
*/
void exception_state(const std::function<void(ExceptionsInfo&&)>& cb);

/*
 @brief 设置异常状态
 @param ExceptionLevel exception_level:  设置异常等级  设置急停需要车型支持
*/
bool set_exception_level(const ExceptionLevel& exception_level);

// Action 相关定义（包含结构体定义）

// 注册所有 action 完成回调处理器（单 client 版本）

}  // namespace exception
