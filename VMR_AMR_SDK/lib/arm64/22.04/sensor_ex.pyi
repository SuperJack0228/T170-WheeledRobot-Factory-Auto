"""
sensor RPC 客户端 stub 模块

该模块为 pybind 导出的 C++ 接口提供类型声明。
自动生成的文件，请勿手动修改。
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union
from py_client_common_ex import ClientHandle

Handleweight = int
HandleMotorInfoPub = int


class WheelPlacement(Enum):
    kUnknow = 0
    kLeft = 1
    kRight = 2
    kLift = 3
    kRotate = 4


class WeightInfo:
    total_weight: float
    r""" 总重量 单位kg"""

    @staticmethod
    def from_json(json_str: str) -> "WeightInfo": 
        ...

    def to_json(self) -> str: 
        ...


class MotorInfo:
    placement: WheelPlacement
    current: float
    r""" 总重量 单位kg"""
    temperature: float
    r""" 温度(摄氏度)"""

    @staticmethod
    def from_json(json_str: str) -> "MotorInfo": 
        ...

    def to_json(self) -> str: 
        ...





def single_weight(handle: ClientHandle, id: int, cb: Callable[[WeightInfo], None]) -> None: 
    r""" 称重传感器数据"""
    ...

def get_all_weight(handle: ClientHandle) -> List[Handleweight]:
    ...


def motor(handle: ClientHandle, id: int, cb: Callable[[MotorInfo], None]) -> None: 
    r""" 电机电流 单位ma"""
    ...

def get_all_motor_info_pub(handle: ClientHandle) -> List[HandleMotorInfoPub]:
    ...



