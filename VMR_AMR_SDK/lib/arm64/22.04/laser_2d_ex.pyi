"""
laser_2d RPC 客户端 stub 模块

该模块为 pybind 导出的 C++ 接口提供类型声明。
自动生成的文件，请勿手动修改。
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union
from py_client_common_ex import ClientHandle

HandleLaser2DDev = int



class Header:
    seq: int
    timestamp_ns: int
    frame_id: str

    @staticmethod
    def from_json(json_str: str) -> "Header": 
        ...

    def to_json(self) -> str: 
        ...


class BasicLaserScan:
    header: Header
    angle_min: float
    r""" Start radian (-π~π)"""
    angle_max: float
    r""" End radian (-π~π)"""
    angle_increment: float
    r""" Angular resolution (radians/sample)"""
    time_increment: float
    r""" Time interval (seconds/sample)"""
    scan_time: float
    r""" Total scanning time (seconds)"""
    range_min: float
    r""" Minimum effective distance (meters)"""
    range_max: float
    r""" Maximum effective distance (meters)"""
    ranges: list[float]
    r""" Distance data (meters)"""
    intensities: list[float]
    r""" Intensity data (optional)"""

    @staticmethod
    def from_json(json_str: str) -> "BasicLaserScan": 
        ...

    def to_json(self) -> str: 
        ...





def get_all_laser2_d_dev(handle: ClientHandle) -> List[HandleLaser2DDev]:
    ...


def laser_scan(handle: ClientHandle, id: int, cb: Callable[[BasicLaserScan], None]) -> None: 
    ...


