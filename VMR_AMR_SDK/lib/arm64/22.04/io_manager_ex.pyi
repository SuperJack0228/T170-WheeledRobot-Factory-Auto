"""
io_manager RPC 客户端 stub 模块

该模块为 pybind 导出的 C++ 接口提供类型声明。
自动生成的文件，请勿手动修改。
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union
from py_client_common_ex import ClientHandle

HandleIoDev = int



class IoDataReport:
    timestamp_us: int
    r""" 时间戳 单位 us"""
    di_: list[bool]
    r""" di 状态"""
    do_: list[bool]
    r""" d0 状态"""

    @staticmethod
    def from_json(json_str: str) -> "IoDataReport": 
        ...

    def to_json(self) -> str: 
        ...





def get_all_io_dev(handle: ClientHandle) -> List[HandleIoDev]:
    ...


def io_state(handle: ClientHandle, id: int, cb: Callable[[IoDataReport], None]) -> None: 
    r"""
    @brief 实时获取IO数据状态
    @param cb: 回调函数, 函数接收一个IoDataReport作为输入
    """
    ...

def set_do_bit(handle: ClientHandle, id: HandleIoDev, bit_offset: int, value: bool) -> None:
    r"""
    @brief 设置do状态 
    @param uint8_t: do 偏移量
    @param bool:   触发与否
    """
    ...



