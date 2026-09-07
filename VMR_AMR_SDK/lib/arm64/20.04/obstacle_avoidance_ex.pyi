"""
obstacle_avoidance RPC ??? stub ??

???? pybind ??? C++ ?????????
???????????????
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union
from py_client_common_ex import ClientHandle



class PoseXY:
    x: float
    y: float

    @staticmethod
    def from_json(json_str: str) -> "PoseXY": 
        ...

    def to_json(self) -> str: 
        ...


class VirtualObstacleInfo:
    obstacle_id: int
    r""" Obstacle ID"""
    top_left: PoseXY
    r""" Top-left corner"""
    bottom_right: PoseXY
    r""" Bottom-right corner"""

    @staticmethod
    def from_json(json_str: str) -> "VirtualObstacleInfo": 
        ...

    def to_json(self) -> str: 
        ...


class VirtualObstacleInfoList:
    virtual_obstacle_list: list[VirtualObstacleInfo]
    r""" Virtual Obstacle List"""

    @staticmethod
    def from_json(json_str: str) -> "VirtualObstacleInfoList": 
        ...

    def to_json(self) -> str: 
        ...





def has_obstacle_avoidance_dev(handle: ClientHandle) -> bool:
    ...


def add_virtual_obstacle(handle: ClientHandle, obstacle_info: VirtualObstacleInfo) -> bool:
    r"""
    @brief ???????
    @param VirtualObstacleInfo: ?????
    """
    ...

async def async_add_virtual_obstacle(handle: ClientHandle, obstacle_info: VirtualObstacleInfo) -> bool: 
    r"""
    @brief ???????
    @param VirtualObstacleInfo: ?????
    """
    ...


def remove_virtual_obstacle(handle: ClientHandle, obstacle_id: int) -> bool:
    r"""
    @brief ???????
    @param int32_t: ?????id
    """
    ...

async def async_remove_virtual_obstacle(handle: ClientHandle, obstacle_id: int) -> bool: 
    r"""
    @brief ???????
    @param int32_t: ?????id
    """
    ...


def query_virtual_obstacle(handle: ClientHandle, ) -> VirtualObstacleInfoList:
    r"""
    @brief ?????????
    @param VirtualObstacleInfoList: ???????
    """
    ...

async def async_query_virtual_obstacle(handle: ClientHandle, ) -> VirtualObstacleInfoList: 
    r"""
    @brief ?????????
    @param VirtualObstacleInfoList: ???????
    """
    ...



