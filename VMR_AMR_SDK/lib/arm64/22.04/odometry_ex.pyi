"""
odometry RPC 客户端 stub 模块

该模块为 pybind 导出的 C++ 接口提供类型声明。
自动生成的文件，请勿手动修改。
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union
from py_client_common_ex import ClientHandle



class OdomInfo:
    r"""
    @brief 里程计信息
    @param uint64_t timestamp_ns: Timestamp (nanoseconds)
    @param float x: Odometry coordinate x (meters) 
    @param float y: Odometry coordinate y (meters)
    @param float theta: Odometry angle (radians)
    @param Linear velocity in x direction (m/s)
    @param Linear velocity in y direction (m/s) 
    @param Angular velocity (rad/s)
    """
    timestamp_ns: int
    r""" Timestamp (nanoseconds)"""
    x: float
    r""" Odometry coordinate x (meters)"""
    y: float
    r"""  Odometry coordinate y (meters)"""
    theta: float
    r""" Odometry angle (radians)"""
    vx: float
    r""" Linear velocity in x direction (m/s)"""
    vy: float
    r""" Linear velocity in y direction (m/s)"""
    w: float
    r""" Angular velocity (rad/s)"""

    @staticmethod
    def from_json(json_str: str) -> "OdomInfo": 
        ...

    def to_json(self) -> str: 
        ...





def odom_state(handle: ClientHandle, cb: Callable[[OdomInfo], None]) -> None: 
    r"""
    @brief 实时获取里程计状态,  使用回调函数获取
    @param cb: 回调函数, 接受一个 OdomInfo 参数
    """
    ...

def has_odom_dev(handle: ClientHandle) -> bool:
    ...



