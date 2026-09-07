"""
py_client_common RPC 客户端公共模块 (stub)

该模块为 pybind 导出的 C++ 接口提供类型声明。
自动生成的文件，请勿手动修改。
"""

from typing import Optional, Callable, List, Dict, Any, Union
from enum import IntEnum

# =============================================================================
# 多客户端模式 (isEx=true)
# =============================================================================

# 类型别名
ClientHandle = int
DEFAULT_CLIENT_HANDLE: int

class RpcErrorType(IntEnum):
    """RPC error type enum"""
    Timeout = 0
    Disconnected = 1
    Other = 2

class RpcException(Exception):
    """RPC exception class"""
    error_type: RpcErrorType
    message: str

class ClientConfig:
    """RPC client configuration data class
    @param ip: 服务器 IP 地址，默认为本地回环地址
    @param port: 服务器端口，默认为 9000
    @param connect_timeout_ms: 连接超时时间(毫秒)，默认 3000ms
    @param call_timeout_ms: 调用超时时间(毫秒)，默认 5000ms
    @param auto_reconnect: 是否启用自动重连，默认为 True
    @param max_reconnect_count: 最大重连次数，-1 表示无限重连，默认为 -1
    @param use_kcp: 是否启用 KCP 传输，默认为 False
    """
    ip: str = "127.0.0.1"
    port: int = 9000
    connect_timeout_ms: int = 3000
    call_timeout_ms: int = 5000
    auto_reconnect: bool = True
    max_reconnect_count: int = -1
    use_kcp: bool = False

def create_client(config: ClientConfig) -> ClientHandle:
    """创建新的客户端连接，返回 client handle（失败返回 -1）"""

def close_client(handle: ClientHandle) -> None:
    """关闭指定客户端"""

def is_client_connected(handle: ClientHandle) -> bool:
    """检查指定客户端是否已连接"""

def reconnect(handle: ClientHandle) -> bool:
    """重新连接指定客户端"""

def update_server_addr(handle: ClientHandle, ip: str, port: int) -> None:
    """更新指定客户端的服务器地址"""

def get_client_config(handle: ClientHandle) -> Dict[str, Any]:
    """获取指定客户端的配置"""

def close_all_clients() -> None:
    """关闭所有客户端"""

