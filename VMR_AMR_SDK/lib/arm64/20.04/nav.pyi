"""
nav RPC ??? stub ??

???? pybind ??? C++ ?????????
???????????????
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union



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
    @brief ??????
    @param double move_dist: ????(m)
    @param double rotate_angle: ????(?)
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
    r""" ??????0????"""
    err_message: str
    r""" ????"""
    data: int
    r""" ????"""

    @staticmethod
    def from_json(json_str: str) -> "ResultMoveRelative": 
        ...

    def to_json(self) -> str: 
        ...




class AsyncMoveRelativeTask:
    r"""
    @brief ????
    @param MoveRelativeInfo: ??????
    @return ???????, 0????, ?0????/??
    """
    id: int
    future: Future[ResultMoveRelative]
    cancel: Callable[[], None]


def has_nav_dev() -> bool:
    ...


def ctrl_speed_factor(speed_factor: float) -> bool:
    r"""
    @brief ??????????
    @param double: ????? (0~1)
    """
    ...

async def async_ctrl_speed_factor(speed_factor: float) -> bool: 
    r"""
    @brief ??????????
    @param double: ????? (0~1)
    """
    ...


def set_third_speed_ctrl() -> bool:
    r"""
    @brief ??SDK????
    """
    ...

async def async_set_third_speed_ctrl() -> bool: 
    r"""
    @brief ??SDK????
    """
    ...


def close_third_speed_ctrl() -> bool:
    r"""
    @brief ??SDK????
    """
    ...

async def async_close_third_speed_ctrl() -> bool: 
    r"""
    @brief ??SDK????
    """
    ...


def send_third_expected_speed(third_expected_speed: Twist) -> bool:
    r"""
    @brief ??SDK?? (??????20hz)
    @param Twist: ??  
    """
    ...

async def async_send_third_expected_speed(third_expected_speed: Twist) -> bool: 
    r"""
    @brief ??SDK?? (??????20hz)
    @param Twist: ??  
    """
    ...


def async_moveRelative(arg1: MoveRelativeInfo) -> AsyncMoveRelativeTask:
    r"""
    @brief ????
    @param MoveRelativeInfo: ??????
    @return ???????, 0????, ?0????/??
    """
    ...


