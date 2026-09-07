"""
localization RPC ??? stub ??

???? pybind ??? C++ ?????????
???????????????
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union



class ReLocationInfo:
    timestamp_ns: int
    r""" Timestamp (nanoseconds)"""
    x: float
    r""" Localization coordinate x (meters)"""
    y: float
    r""" Localization coordinate y (meters)"""
    theta: float
    r""" Localization angle (radians)"""

    @staticmethod
    def from_json(json_str: str) -> "ReLocationInfo": 
        ...

    def to_json(self) -> str: 
        ...


class LocationInfo:
    timestamp_ns: int
    r""" Timestamp (nanoseconds)"""
    x: float
    r""" Localization coordinate x (meters)"""
    y: float
    r""" Localization coordinate y (meters)"""
    theta: float
    r""" Localization angle (radians)"""
    confidence: float
    r""" Confidence level 0~2.5"""
    status: int
    r"""
       Localization status
       status       Meaning
       0            Normal
       1            Relocalization failed
       2            Slippage
       6            Relocalizing
       Relocalization Information  
     
    """

    @staticmethod
    def from_json(json_str: str) -> "LocationInfo": 
        ...

    def to_json(self) -> str: 
        ...





def loc_state(cb: Callable[[LocationInfo], None]) -> None: 
    r"""
    @brief ????????,  ????????
    @param cb: ????, ???? LocationInfo ??
    """
    ...

def has_location_dev() -> bool:
    ...


def relocate(loc_info: ReLocationInfo) -> None:
    ...



