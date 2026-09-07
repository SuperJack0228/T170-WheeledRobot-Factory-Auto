"""
bridge_drive RPC 客户端 stub 模块

该模块为 pybind 导出的 C++ 接口提供类型声明。
自动生成的文件，请勿手动修改。
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union
from py_client_common_ex import ClientHandle



class Header:
    seq: int
    timestamp_ns: int
    frame_id: str

    @staticmethod
    def from_json(json_str: str) -> "Header": 
        ...

    def to_json(self) -> str: 
        ...


class Vector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @staticmethod
    def from_json(json_str: str) -> "Vector3": 
        ...

    def to_json(self) -> str: 
        ...


class Quaternion:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 0.0

    @staticmethod
    def from_json(json_str: str) -> "Quaternion": 
        ...

    def to_json(self) -> str: 
        ...


class Pose:
    position: Vector3
    orientation: Quaternion

    @staticmethod
    def from_json(json_str: str) -> "Pose": 
        ...

    def to_json(self) -> str: 
        ...


class Twist:
    r""" 底盘控制"""
    linear: Vector3
    angular: Vector3

    @staticmethod
    def from_json(json_str: str) -> "Twist": 
        ...

    def to_json(self) -> str: 
        ...


class BasicOdometry:
    r""" 里程计数据 (纯基本类型)"""
    header: Header
    child_frame_id: str
    pose: Pose
    twist: Twist

    @staticmethod
    def from_json(json_str: str) -> "BasicOdometry": 
        ...

    def to_json(self) -> str: 
        ...





def has_bridge_drive_dev(handle: ClientHandle) -> bool:
    ...


def setRobotTwist(handle: ClientHandle, cb: Callable[[Twist], None]) -> None: 
    r""" 输入：速度"""
    ...

def updateOdomInfo(handle: ClientHandle, odom: BasicOdometry) -> None:
    r""" 上传：里程计"""
    ...



