"""
laser_2d RPC ??? stub ??

???? pybind ??? C++ ?????????
???????????????
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union

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
    r""" Start radian (-?~?)"""
    angle_max: float
    r""" End radian (-?~?)"""
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





def get_all_laser2_d_dev() -> List[HandleLaser2DDev]:
    ...


def laser_scan(id: int, cb: Callable[[BasicLaserScan], None]) -> None: 
    ...


