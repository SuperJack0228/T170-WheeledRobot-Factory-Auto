"""
py_client_common RPC 客户端公共模块 (stub)

该模块为 pybind 导出的 C++ 接口提供类型声明。
自动生成的文件，请勿手动修改。
"""

from typing import Optional, Callable, List, Dict, Any, Union
from enum import IntEnum

# =============================================================================
# 单客户端模式 (isEx=false)
# =============================================================================

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

def init_client(config: ClientConfig) -> bool:
    """初始化 RPC 客户端配置，必须在调用任何 RPC 函数之前调用"""

def close_client() -> None:
    """关闭并清理 RPC 客户端"""

def is_client_connected() -> bool:
    """检查客户端是否已连接"""

def reconnect() -> bool:
    """重新连接"""

def update_server_addr(ip: str, port: int) -> None:
    """更新服务器地址(用于动态切换服务器)"""

def get_client_config() -> Dict[str, Any]:
    """获取当前客户端配置dict"""

