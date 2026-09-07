"""
map_reader RPC 客户端 stub 模块

该模块为 pybind 导出的 C++ 接口提供类型声明。
自动生成的文件，请勿手动修改。
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union
from py_client_common_ex import ClientHandle



class Goal:
    r""" 拓扑点"""
    id: int
    r""" 地图点id"""
    center: list[float]
    r""" 点位[x,y] 单位m"""
    name: str
    r""" 点位名称"""

    @staticmethod
    def from_json(json_str: str) -> "Goal": 
        ...

    def to_json(self) -> str: 
        ...


class Line:
    r""" 路径"""
    id: int
    r""" 路径id"""
    start_name: str
    r""" 起点名称"""
    end_name: str
    r""" 终点名称"""

    @staticmethod
    def from_json(json_str: str) -> "Line": 
        ...

    def to_json(self) -> str: 
        ...


class MapPoints:
    goals: list[Goal]
    r""" 所有点"""
    lines: list[Line]
    r""" 所有路径"""

    @staticmethod
    def from_json(json_str: str) -> "MapPoints": 
        ...

    def to_json(self) -> str: 
        ...





def has_map_reader(handle: ClientHandle) -> bool:
    ...


def get_current_map_info(handle: ClientHandle, ) -> MapPoints:
    r""" 获取当前地图的所有点位 和 路径"""
    ...

async def async_get_current_map_info(handle: ClientHandle, ) -> MapPoints: 
    r""" 获取当前地图的所有点位 和 路径"""
    ...



