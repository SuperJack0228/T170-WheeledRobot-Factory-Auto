"""
exception RPC ??? stub ??

???? pybind ??? C++ ?????????
???????????????
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union
from py_client_common_ex import ClientHandle


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





def exception_state(handle: ClientHandle, cb: Callable[[ExceptionsInfo], None]) -> None: 
    r"""
    @brief ????????
    @param cb: ????, ??????ExceptionsInfo????
    """
    ...

def has_exception_dev(handle: ClientHandle) -> bool:
    ...


def set_exception_level(handle: ClientHandle, exception_level: ExceptionLevel) -> bool:
    r"""
    @brief ??????
    @param ExceptionLevel exception_level:  ??????  ??????????
    """
    ...

async def async_set_exception_level(handle: ClientHandle, exception_level: ExceptionLevel) -> bool: 
    r"""
    @brief ??????
    @param ExceptionLevel exception_level:  ??????  ??????????
    """
    ...



