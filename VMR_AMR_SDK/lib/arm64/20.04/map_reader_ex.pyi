"""
map_reader RPC ??? stub ??

???? pybind ??? C++ ?????????
???????????????
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union
from py_client_common_ex import ClientHandle



class Goal:
    r""" ???"""
    id: int
    r""" ???id"""
    center: list[float]
    r""" ??[x,y] ??m"""
    name: str
    r""" ????"""

    @staticmethod
    def from_json(json_str: str) -> "Goal": 
        ...

    def to_json(self) -> str: 
        ...


class Line:
    r""" ??"""
    id: int
    r""" ??id"""
    start_name: str
    r""" ????"""
    end_name: str
    r""" ????"""

    @staticmethod
    def from_json(json_str: str) -> "Line": 
        ...

    def to_json(self) -> str: 
        ...


class MapPoints:
    goals: list[Goal]
    r""" ???"""
    lines: list[Line]
    r""" ????"""

    @staticmethod
    def from_json(json_str: str) -> "MapPoints": 
        ...

    def to_json(self) -> str: 
        ...





def has_map_reader(handle: ClientHandle) -> bool:
    ...


def get_current_map_info(handle: ClientHandle, ) -> MapPoints:
    r""" ??????????? ? ??"""
    ...

async def async_get_current_map_info(handle: ClientHandle, ) -> MapPoints: 
    r""" ??????????? ? ??"""
    ...



