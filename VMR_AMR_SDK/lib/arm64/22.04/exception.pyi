"""
exception RPC 客户端 stub 模块

该模块为 pybind 导出的 C++ 接口提供类型声明。
自动生成的文件，请勿手动修改。
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union


class ExceptionLevel(Enum):
    NORMAL = 0
    STOP = 1
    EMERGENCY_STOP = 2


class Exception:
    exception_moudle: int
    r""" exception moudle"""
    exception_code: int
    r""" exception code"""

    @staticmethod
    def from_json(json_str: str) -> "Exception": 
        ...

    def to_json(self) -> str: 
        ...


class ExceptionsInfo:
    timestamp_ns: int
    r""" Timestamp (nanoseconds)"""
    robot_exception: ExceptionLevel
    exceptions: list[Exception]
    r""" Exception status"""

    @staticmethod
    def from_json(json_str: str) -> "ExceptionsInfo": 
        ...

    def to_json(self) -> str: 
        ...





def exception_state(cb: Callable[[ExceptionsInfo], None]) -> None: 
    r"""
    @brief 实时获取异常状态
    @param cb: 回调函数, 函数接收一个ExceptionsInfo作为输入
    """
    ...

def has_exception_dev() -> bool:
    ...


def set_exception_level(exception_level: ExceptionLevel) -> bool:
    r"""
    @brief 设置异常状态
    @param ExceptionLevel exception_level:  设置异常等级  设置急停需要车型支持
    """
    ...

async def async_set_exception_level(exception_level: ExceptionLevel) -> bool: 
    r"""
    @brief 设置异常状态
    @param ExceptionLevel exception_level:  设置异常等级  设置急停需要车型支持
    """
    ...



