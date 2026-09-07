"""
odometry RPC ??? stub ??

???? pybind ??? C++ ?????????
???????????????
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union
from py_client_common_ex import ClientHandle



class OdomInfo:
    r"""
    @brief ?????
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
    @brief ?????????,  ????????
    @param cb: ????, ???? OdomInfo ??
    """
    ...

def has_odom_dev(handle: ClientHandle) -> bool:
    ...



