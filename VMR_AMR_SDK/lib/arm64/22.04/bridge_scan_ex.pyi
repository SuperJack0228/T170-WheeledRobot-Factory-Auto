"""
bridge_scan RPC 客户端 stub 模块

该模块为 pybind 导出的 C++ 接口提供类型声明。
自动生成的文件，请勿手动修改。
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union
from py_client_common_ex import ClientHandle



class Scan:
    timestamp_ns: int
    r""" Timestamp (nanoseconds)"""
    location: int
    r""" Scanning Parameters"""
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
    r"""
    Scanning Data
       Distance data (meters) 
    """
    intensities: list[float]
    r""" Intensity data (optional)"""

    @staticmethod
    def from_json(json_str: str) -> "Scan": 
        ...

    def to_json(self) -> str: 
        ...





def has_bridge_scan_dev(handle: ClientHandle) -> bool:
    ...


def updateLaserInfo(handle: ClientHandle, scan_info: Scan) -> None:
    r""" 上传：laser数据"""
    ...


def laser_scan(handle: ClientHandle, cb: Callable[[Scan], None]) -> None: 
    ...


