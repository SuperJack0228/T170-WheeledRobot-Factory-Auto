"""
lift RPC 客户端 stub 模块

该模块为 pybind 导出的 C++ 接口提供类型声明。
自动生成的文件，请勿手动修改。
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union
from py_client_common_ex import ClientHandle



class LiftInfo:
    r"""
    @brief 顶升设备状态
    @param timestamp_ns: 时间戳 ns
    @param lift_status: 顶升高度 单位 mm
    """
    timestamp_ns: int
    r""" Timestamp (nanoseconds)"""
    lift_status: int
    r""" 顶升高度 单位 mm"""

    @staticmethod
    def from_json(json_str: str) -> "LiftInfo": 
        ...

    def to_json(self) -> str: 
        ...


class ResultLiftCtr:
    err_code: int = 0
    r""" 错误码，默认0表示成功"""
    err_message: str
    r""" 错误消息"""
    data: int
    r""" 返回数据"""

    @staticmethod
    def from_json(json_str: str) -> "ResultLiftCtr": 
        ...

    def to_json(self) -> str: 
        ...




class AsyncLiftCtrTask:
    r"""
    @brief 下达顶升设备控制命令
    @param LiftInfo: 顶升设备控制命令，通过设置 lift_status 字段指定目标状态
    @return 返回执行结果 顶升高度
    """
    id: int
    future: Future[ResultLiftCtr]
    cancel: Callable[[], None]


def lift_state(handle: ClientHandle, cb: Callable[[LiftInfo], None]) -> None: 
    r"""
    @brief 获取顶升设备状态, 服务端发布和客户端订阅, 使用回调函数获取
    @param cb: 回调函数, 函数接收一个LiftInfo作为输入
    """
    ...

def has_lift_dev(handle: ClientHandle) -> bool:
    ...


def async_lift_ctr(handle: ClientHandle, arg1: LiftInfo) -> AsyncLiftCtrTask:
    r"""
    @brief 下达顶升设备控制命令
    @param LiftInfo: 顶升设备控制命令，通过设置 lift_status 字段指定目标状态
    @return 返回执行结果 顶升高度
    """
    ...


