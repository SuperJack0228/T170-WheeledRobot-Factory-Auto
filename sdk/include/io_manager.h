#pragma once
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "client/cmd_future.hpp"

// 包含公共客户端接口（单client版本）
#include "client/common/client_common.h"

namespace io_manager {

// ============================================
// 单 Client 版本 API（无 handle 参数）
// 用于只连接单个服务器
// ============================================

// handle（单client版本，无handle参数）
// IoDev有多个设备实例
using HandleIoDev = int;
// 获取所有实例的 handle（单client版本）
std::vector<HandleIoDev> GetAllHandleIoDev();

// RPC/Pub函数声明（包含结构体定义）
struct IoDataReport {
  // 时间戳 单位 us
  uint64_t timestamp_us;
  // di 状态
  std::vector<bool> di_;
  // d0 状态
  std::vector<bool> do_;
};

/*
 @brief 实时获取IO数据状态
 @param cb: 回调函数, 函数接收一个IoDataReport作为输入
*/
void io_state(HandleIoDev id, const std::function<void(IoDataReport&&)>& cb);

/*
 @brief 设置do状态
 @param uint8_t: do 偏移量
 @param bool:   触发与否
*/
void set_do_bit(HandleIoDev id, uint8_t bit_offset, bool value);

// Action 相关定义（包含结构体定义）

// 注册所有 action 完成回调处理器（单 client 版本）

}  // namespace io_manager
