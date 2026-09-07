// 单 client 版本头文件
#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <system_error>

namespace rpc_client {

// ============================================
// RPC 客户端配置结构体
// 所有 namespace 共用此配置
// ============================================
struct ClientConfig {
  // 服务器 IP 地址，默认为本地回环地址
  std::string ip = "127.0.0.1";
  // 服务器端口，默认为 9000
  int port = 9000;
  // 连接超时时间(毫秒)，默认 3000ms
  int connect_timeout_ms = 3000;
  // 调用超时时间(毫秒)，默认 5000ms
  int call_timeout_ms = 5000;
  // 通信错误回调函数类型定义
  // 参数: error_code - 错误码, message - 错误信息
  using ErrorCallback = std::function<void(const std::error_code& ec,
                                           const std::string& message)>;
  // 错误回调函数，在连接异常断开时触发
  ErrorCallback error_callback = nullptr;
  // 是否启用自动重连，默认为 true
  bool auto_reconnect = true;
  // 最大重连次数，-1 表示无限重连，默认为 -1
  int max_reconnect_count = -1;
  // KCP传输配置（复用RPC的ip和port，无需单独配置）
  bool use_kcp = false;
};

// ============================================
// RPC 客户端初始化和配置接口（单客户端模式）
// 所有 namespace 共用这些接口
// 实现在 client/internal/internal.cpp 中
// ============================================

// 初始化 RPC 客户端配置
// 必须在调用任何 RPC 函数之前调用，用于设置客户端的连接参数
// 返回: true - 连接成功, false - 连接失败
extern bool InitClient(const ClientConfig& config);

// 获取当前客户端配置
extern const ClientConfig& GetClientConfig();

// 关闭并清理 RPC 客户端
extern void CloseClient();

// 检查客户端是否已连接
extern bool IsClientConnected();

// ============================================
// RPC 调用异常
// ============================================
class RpcException : public std::runtime_error {
 public:
  enum class ErrorType {
    Timeout,       // 调用超时
    Disconnected,  // 连接断开
    Other          // 其他错误
  };

  RpcException(ErrorType type, const std::string& msg)
      : std::runtime_error(msg), error_type_(type) {}

  ErrorType error_type() const { return error_type_; }

 private:
  ErrorType error_type_;
};

}  // namespace rpc_client
