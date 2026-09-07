"""
nav RPC 客户端 stub 模块

该模块为 pybind 导出的 C++ 接口提供类型声明。
自动生成的文件，请勿手动修改。
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union
from py_client_common_ex import ClientHandle



class Vector3:
    x: float
    y: float
    z: float

    @staticmethod
    def from_json(json_str: str) -> "Vector3": 
        ...

    def to_json(self) -> str: 
        ...


class Twist:
    linear: Vector3
    r""" linear (m/s)"""
    angular: Vector3
    r""" angular (rad/s)"""

    @staticmethod
    def from_json(json_str: str) -> "Twist": 
        ...

    def to_json(self) -> str: 
        ...


class MoveRelativeInfo:
    r"""
    @brief 定距移动参数
    @param double move_dist: 移动距离(m)
    @param double rotate_angle: 移动角度(°)
    """
    move_dist: float
    rotate_angle: float

    @staticmethod
    def from_json(json_str: str) -> "MoveRelativeInfo": 
        ...

    def to_json(self) -> str: 
        ...


class ResultMoveRelative:
    err_code: int = 0
    r""" 错误码，默认0表示成功"""
    err_message: str
    r""" 错误消息"""
    data: int
    r""" 返回数据"""

    @staticmethod
    def from_json(json_str: str) -> "ResultMoveRelative": 
        ...

    def to_json(self) -> str: 
        ...




class AsyncMoveRelativeTask:
    r"""
    @brief 定距移动
    @param MoveRelativeInfo: 定距移动参数
    @return 导航任务结果码, 0表示成功, 非0表示失败/取消
    """
    id: int
    future: Future[ResultMoveRelative]
    cancel: Callable[[], None]


def has_nav_dev(handle: ClientHandle) -> bool:
    ...


def ctrl_speed_factor(handle: ClientHandle, speed_factor: float) -> bool:
    r"""
    @brief 设置当前速度的百分比
    @param double: 速度百分比 (0~1)
    """
    ...

async def async_ctrl_speed_factor(handle: ClientHandle, speed_factor: float) -> bool: 
    r"""
    @brief 设置当前速度的百分比
    @param double: 速度百分比 (0~1)
    """
    ...


def set_third_speed_ctrl(handle: ClientHandle, ) -> bool:
    r"""
    @brief 开启SDK控制速度
    """
    ...

async def async_set_third_speed_ctrl(handle: ClientHandle, ) -> bool: 
    r"""
    @brief 开启SDK控制速度
    """
    ...


def close_third_speed_ctrl(handle: ClientHandle, ) -> bool:
    r"""
    @brief 关闭SDK控制速度
    """
    ...

async def async_close_third_speed_ctrl(handle: ClientHandle, ) -> bool: 
    r"""
    @brief 关闭SDK控制速度
    """
    ...


def send_third_expected_speed(handle: ClientHandle, third_expected_speed: Twist) -> bool:
    r"""
    @brief 下发SDK速度 (下发频率大于20hz)
    @param Twist: 速度  
    """
    ...

async def async_send_third_expected_speed(handle: ClientHandle, third_expected_speed: Twist) -> bool: 
    r"""
    @brief 下发SDK速度 (下发频率大于20hz)
    @param Twist: 速度  
    """
    ...


def async_moveRelative(handle: ClientHandle, arg1: MoveRelativeInfo) -> AsyncMoveRelativeTask:
    r"""
    @brief 定距移动
    @param MoveRelativeInfo: 定距移动参数
    @return 导航任务结果码, 0表示成功, 非0表示失败/取消
    """
    ...


