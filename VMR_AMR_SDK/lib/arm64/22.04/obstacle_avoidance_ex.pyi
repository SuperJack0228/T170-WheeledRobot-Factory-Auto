"""
obstacle_avoidance RPC 客户端 stub 模块

该模块为 pybind 导出的 C++ 接口提供类型声明。
自动生成的文件，请勿手动修改。
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
    @brief 新增虚拟障碍物
    @param VirtualObstacleInfo: 障碍物信息
    """
    ...

async def async_add_virtual_obstacle(handle: ClientHandle, obstacle_info: VirtualObstacleInfo) -> bool: 
    r"""
    @brief 新增虚拟障碍物
    @param VirtualObstacleInfo: 障碍物信息
    """
    ...


def remove_virtual_obstacle(handle: ClientHandle, obstacle_id: int) -> bool:
    r"""
    @brief 删除虚拟障碍物
    @param int32_t: 虚拟障碍物id
    """
    ...

async def async_remove_virtual_obstacle(handle: ClientHandle, obstacle_id: int) -> bool: 
    r"""
    @brief 删除虚拟障碍物
    @param int32_t: 虚拟障碍物id
    """
    ...


def query_virtual_obstacle(handle: ClientHandle, ) -> VirtualObstacleInfoList:
    r"""
    @brief 查询虚拟障碍物列表
    @param VirtualObstacleInfoList: 虚拟障碍物列表
    """
    ...

async def async_query_virtual_obstacle(handle: ClientHandle, ) -> VirtualObstacleInfoList: 
    r"""
    @brief 查询虚拟障碍物列表
    @param VirtualObstacleInfoList: 虚拟障碍物列表
    """
    ...



