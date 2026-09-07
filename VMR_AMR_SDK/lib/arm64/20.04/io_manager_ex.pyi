"""
io_manager RPC ??? stub ??

???? pybind ??? C++ ?????????
???????????????
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union
from py_client_common_ex import ClientHandle

HandleIoDev = int



class IoDataReport:
    timestamp_us: int
    r""" ??? ?? us"""
    di_: list[bool]
    r""" di ??"""
    do_: list[bool]
    r""" d0 ??"""

    @staticmethod
    def from_json(json_str: str) -> "IoDataReport": 
        ...

    def to_json(self) -> str: 
        ...





def get_all_io_dev(handle: ClientHandle) -> List[HandleIoDev]:
    ...


def io_state(handle: ClientHandle, id: int, cb: Callable[[IoDataReport], None]) -> None: 
    r"""
    @brief ????IO????
    @param cb: ????, ??????IoDataReport????
    """
    ...

def set_do_bit(handle: ClientHandle, id: HandleIoDev, bit_offset: int, value: bool) -> None:
    r"""
    @brief ??do?? 
    @param uint8_t: do ???
    @param bool:   ????
    """
    ...



