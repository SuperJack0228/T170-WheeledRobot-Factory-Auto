"""
light RPC 客户端 stub 模块

该模块为 pybind 导出的 C++ 接口提供类型声明。
自动生成的文件，请勿手动修改。
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union


class LightState(Enum):
    WHITE_BOOTING = 0
    BLINK_LEFT_BLUE = 1
    BLINK_RIGHT_BLUE = 2
    PURPLE_TRAFFIC = 3
    RED_SAFE_STOP = 4
    YELLOW_WARNING = 5
    YELLOW_LOW_POWER = 6
    RED_EXCEPTION = 7
    GREEN_CHARGING = 8
    RED_SELF_FAULT = 9
    OFF = 10
    BLUE_IDLE = 11
    BLUE_RUNNING = 12
    WHITE_MANUAL = 13


class LightInfo:
    timestamp_ns: int
    r""" Timestamp (nanoseconds)"""
    light_status: LightState
    r""" Light status"""

    @staticmethod
    def from_json(json_str: str) -> "LightInfo": 
        ...

    def to_json(self) -> str: 
        ...





def light_state(cb: Callable[[LightInfo], None]) -> None: 
    r"""
    @brief 获取灯带状态 使用回调函数获取
    @param cb: 回调函数, LightInfo
    """
    ...

def has_light_dev() -> bool:
    ...


def ctr_light(light_info: LightInfo) -> None:
    r"""
    @brief 设置灯带状态
    @param LightInfo: 灯带状态
    """
    ...



