// 多 client 版本头文件 - 包含单 client 版本并添加多 client 支持
#pragma once

// 包含单 client 版本的基础定义
#include "client/common/client_common.h"

namespace rpc_client {

// ============================================
// 客户端句柄定义
// ============================================
using ClientHandle = int;
static constexpr ClientHandle DEFAULT_CLIENT_HANDLE = 0;

// ============================================
// RPC 多客户端管理接口
// 实现在 client/internal/internal_ex.cpp 中
// ============================================

// 创建新的客户端连接，返回 client handle（失败返回 -1）
extern ClientHandle CreateClient(const ClientConfig& config);

// 关闭指定客户端
extern void CloseClient(ClientHandle handle);

// 检查指定客户端是否已连接
extern bool IsClientConnected(ClientHandle handle);

// 重新连接指定客户端
extern bool Reconnect(ClientHandle handle);

// 更新指定客户端的服务器地址
extern void UpdateServerAddr(ClientHandle handle, const std::string& ip,
                             int port);

// 获取指定客户端的配置
extern const ClientConfig& GetClientConfig(ClientHandle handle);

// 关闭所有客户端
extern void CloseAllClients();

}  // namespace rpc_client
