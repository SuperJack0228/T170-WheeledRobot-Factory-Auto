"""
rpc_test RPC 客户端 stub 模块

该模块为 pybind 导出的 C++ 接口提供类型声明。
自动生成的文件，请勿手动修改。
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union
from py_client_common_ex import ClientHandle

HandlePalletArriveSensor = int



class ChildA:
    vec_data: list[int] = [0]
    r"""注释2"""

    @staticmethod
    def from_json(json_str: str) -> "ChildA": 
        ...

    def to_json(self) -> str: 
        ...


class RpcParam:
    a: Optional[str]
    r"""注释1"""
    vec_data: list[ChildA]
    r"""注释2"""
    vec_string_data: list[str] = ["0x01", "0x02"]
    array_string_data: list[str] = ["0x01", "0x02"]
    b: str
    c: ChildA

    @staticmethod
    def from_json(json_str: str) -> "RpcParam": 
        ...

    def to_json(self) -> str: 
        ...


class ReturnA:
    vec_data: list[int] = []
    cancel_flag: bool

    @staticmethod
    def from_json(json_str: str) -> "ReturnA": 
        ...

    def to_json(self) -> str: 
        ...


class CmdParam:
    a: list[int]

    @staticmethod
    def from_json(json_str: str) -> "CmdParam": 
        ...

    def to_json(self) -> str: 
        ...


class ResultCmdName:
    err_code: int = 0
    r""" 错误码，默认0表示成功"""
    err_message: str
    r""" 错误消息"""
    data: ReturnA
    r""" 返回数据"""

    @staticmethod
    def from_json(json_str: str) -> "ResultCmdName": 
        ...

    def to_json(self) -> str: 
        ...




class AsyncCmdNameTask:
    id: int
    future: Future[ResultCmdName]
    cancel: Callable[[], None]


def topic_name(handle: ClientHandle, id: HandlePalletArriveSensor, arg1: int, arg2: RpcParam) -> RpcParam:
    r"""
    入参1:...
    入参2:...
    """
    ...

async def async_topic_name(handle: ClientHandle, id: HandlePalletArriveSensor, arg1: int, arg2: RpcParam) -> RpcParam: 
    r"""
    入参1:...
    入参2:...
    """
    ...


def pub_name(handle: ClientHandle, id: int, cb: Callable[[RpcParam], None]) -> None: 
    r"""
    入参1:RpcParam
    """
    ...

def async_cmd_name(handle: ClientHandle, id: HandlePalletArriveSensor, arg1: int, arg2: CmdParam) -> AsyncCmdNameTask:
    ...

def get_all_pallet_arrive_sensor(handle: ClientHandle) -> List[HandlePalletArriveSensor]:
    ...



