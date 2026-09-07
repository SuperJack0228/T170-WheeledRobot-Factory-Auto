"""
audio RPC ??? stub ??

???? pybind ??? C++ ?????????
???????????????
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union
from py_client_common_ex import ClientHandle



class AudioPlayRequest:
    r"""
    @brief ??????
    @param string audio_name: ???????????; mp3 ??????, modbus ??? modbus:12.mp3 ? tts:??
    @param int32_t priority: ?????, ?????????
    @param bool loop: ??????
    @param int32_t volume: ????, ?? 0~100, ?? 0 ????????
    """
    audio_name: str
    priority: int = 0
    loop: bool = False
    volume: int = -1

    @staticmethod
    def from_json(json_str: str) -> "AudioPlayRequest": 
        ...

    def to_json(self) -> str: 
        ...


class AudioState:
    r"""
    @brief ??????
    @param uint64_t timestamp_ns: ??? ns
    @param bool playing: ??????
    @param string audio_name: ????????
    @param int32_t volume: ????, ?? 0~100
    @param int32_t exception: ???, 0 ????
    """
    timestamp_ns: int
    playing: bool = False
    audio_name: str
    volume: int = 0
    exception: int = 0

    @staticmethod
    def from_json(json_str: str) -> "AudioState": 
        ...

    def to_json(self) -> str: 
        ...





def audio_state(handle: ClientHandle, cb: Callable[[AudioState], None]) -> None: 
    r"""
    @brief ??????????
    @param cb: ????, ?? AudioState ??
    """
    ...

def has_audio_dev(handle: ClientHandle) -> bool:
    ...


def play_audio(handle: ClientHandle, audio_request: AudioPlayRequest) -> bool:
    r"""
    @brief ????
    @param AudioPlayRequest: ??????
    """
    ...

async def async_play_audio(handle: ClientHandle, audio_request: AudioPlayRequest) -> bool: 
    r"""
    @brief ????
    @param AudioPlayRequest: ??????
    """
    ...


def stop_audio(handle: ClientHandle, ) -> bool:
    r"""
    @brief ????????
    """
    ...

async def async_stop_audio(handle: ClientHandle, ) -> bool: 
    r"""
    @brief ????????
    """
    ...


def set_audio_volume(handle: ClientHandle, volume: int) -> bool:
    r"""
    @brief ??????
    @param int32_t: ??, ?? 0~100
    """
    ...

async def async_set_audio_volume(handle: ClientHandle, volume: int) -> bool: 
    r"""
    @brief ??????
    @param int32_t: ??, ?? 0~100
    """
    ...


def query_audio_volume(handle: ClientHandle, ) -> int:
    r"""
    @brief ??????
    @return int32_t: ????, ?? 0~100
    """
    ...

async def async_query_audio_volume(handle: ClientHandle, ) -> int: 
    r"""
    @brief ??????
    @return int32_t: ????, ?? 0~100
    """
    ...



