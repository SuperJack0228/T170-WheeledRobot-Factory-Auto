"""
light RPC ??? stub ??

???? pybind ??? C++ ?????????
???????????????
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union
from py_client_common_ex import ClientHandle


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





def light_state(handle: ClientHandle, cb: Callable[[LightInfo], None]) -> None: 
    r"""
    @brief ?????? ????????
    @param cb: ????, LightInfo
    """
    ...

def has_light_dev(handle: ClientHandle) -> bool:
    ...


def ctr_light(handle: ClientHandle, light_info: LightInfo) -> None:
    r"""
    @brief ??????
    @param LightInfo: ????
    """
    ...



