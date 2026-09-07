"""
audio RPC 客户端 stub 模块

该模块为 pybind 导出的 C++ 接口提供类型声明。
自动生成的文件，请勿手动修改。
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union
from py_client_common_ex import ClientHandle



class AudioPlayRequest:
    r"""
    @brief 音频播放请求
    @param string audio_name: 音频名称或音频文件路径; mp3 使用文件路径, modbus 可使用 modbus:12.mp3 或 tts:文本
    @param int32_t priority: 播放优先级, 数值越大优先级越高
    @param bool loop: 是否循环播放
    @param int32_t volume: 播放音量, 范围 0~100, 小于 0 表示使用当前音量
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
    @brief 音频设备状态
    @param uint64_t timestamp_ns: 时间戳 ns
    @param bool playing: 是否正在播放
    @param string audio_name: 当前播放音频名称
    @param int32_t volume: 当前音量, 范围 0~100
    @param int32_t exception: 异常码, 0 表示正常
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
    @brief 实时获取音频设备状态
    @param cb: 回调函数, 接收 AudioState 参数
    """
    ...

def has_audio_dev(handle: ClientHandle) -> bool:
    ...


def play_audio(handle: ClientHandle, audio_request: AudioPlayRequest) -> bool:
    r"""
    @brief 播放音频
    @param AudioPlayRequest: 音频播放请求
    """
    ...

async def async_play_audio(handle: ClientHandle, audio_request: AudioPlayRequest) -> bool: 
    r"""
    @brief 播放音频
    @param AudioPlayRequest: 音频播放请求
    """
    ...


def stop_audio(handle: ClientHandle, ) -> bool:
    r"""
    @brief 停止当前音频播放
    """
    ...

async def async_stop_audio(handle: ClientHandle, ) -> bool: 
    r"""
    @brief 停止当前音频播放
    """
    ...


def set_audio_volume(handle: ClientHandle, volume: int) -> bool:
    r"""
    @brief 设置音频音量
    @param int32_t: 音量, 范围 0~100
    """
    ...

async def async_set_audio_volume(handle: ClientHandle, volume: int) -> bool: 
    r"""
    @brief 设置音频音量
    @param int32_t: 音量, 范围 0~100
    """
    ...


def query_audio_volume(handle: ClientHandle, ) -> int:
    r"""
    @brief 查询音频音量
    @return int32_t: 当前音量, 范围 0~100
    """
    ...

async def async_query_audio_volume(handle: ClientHandle, ) -> int: 
    r"""
    @brief 查询音频音量
    @return int32_t: 当前音量, 范围 0~100
    """
    ...



