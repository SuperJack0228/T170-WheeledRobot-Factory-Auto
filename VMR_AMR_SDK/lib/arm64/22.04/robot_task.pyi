"""
robot_task RPC 客户端 stub 模块

该模块为 pybind 导出的 C++ 接口提供类型声明。
自动生成的文件，请勿手动修改。
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union



class RobotPose:
    r"""
    @brief 地图点位
    @param double x: 地图x轴 单位 m
    @param double y: 地图y轴 单位 m
    @param double theta: 姿态 单位 弧度
    """
    x: float
    y: float
    theta: float

    @staticmethod
    def from_json(json_str: str) -> "RobotPose": 
        ...

    def to_json(self) -> str: 
        ...


class TopoPose:
    r"""
    @brief 拓扑点位任务
    @param string pose_name: 点位名
    @param int32_t action: 动作 id  默认0 无动作仅移动  3:充电  4:停止充电
    """
    pose_name: str
    action: int

    @staticmethod
    def from_json(json_str: str) -> "TopoPose": 
        ...

    def to_json(self) -> str: 
        ...


class CmdResult:
    r"""
    @brief 充电任务
    @param result: 执行结果 1为成功 3为取消  其余都是失败
    """
    result: int

    @staticmethod
    def from_json(json_str: str) -> "CmdResult": 
        ...

    def to_json(self) -> str: 
        ...


class ResultChargeTask:
    err_code: int = 0
    r""" 错误码，默认0表示成功"""
    err_message: str
    r""" 错误消息"""
    data: CmdResult
    r""" 返回数据"""

    @staticmethod
    def from_json(json_str: str) -> "ResultChargeTask": 
        ...

    def to_json(self) -> str: 
        ...


class ResultTopoMoveTask:
    err_code: int = 0
    r""" 错误码，默认0表示成功"""
    err_message: str
    r""" 错误消息"""
    data: CmdResult
    r""" 返回数据"""

    @staticmethod
    def from_json(json_str: str) -> "ResultTopoMoveTask": 
        ...

    def to_json(self) -> str: 
        ...




class AsyncChargeTaskTask:
    r"""
    @brief 下发充电任务 小车会根据自由导航 直接到点充电
    @param RobotPose: 地图上充电点位 
    @return 返回一个 SimpleNamespace 对象，包含以下属性：
            - id: 命令唯一标识符
            - future: 异步 future 可通过 await 等待执行结果
            - cancel: 可调用对象（callable），调用后尝试取消该命令
    """
    id: int
    future: Future[ResultChargeTask]
    cancel: Callable[[], None]
class AsyncTopoMoveTaskTask:
    r"""
    @brief 下发拓扑路径移动任务 小车会根据地图规划拓扑路径 执行任务  任务会以队列形式执行
    @param TopoPose: 地图点位
    @return 返回一个 SimpleNamespace 对象，包含以下属性：
            - id: 命令唯一标识符
            - future: 异步 future 可通过 await 等待执行结果
            - cancel: 可调用对象（callable），调用后取消所有拓扑移动任务
    """
    id: int
    future: Future[ResultTopoMoveTask]
    cancel: Callable[[], None]


def has_robot_task_dev() -> bool:
    ...


def async_charge_task(arg1: RobotPose) -> AsyncChargeTaskTask:
    r"""
    @brief 下发充电任务 小车会根据自由导航 直接到点充电
    @param RobotPose: 地图上充电点位 
    @return 返回一个 SimpleNamespace 对象，包含以下属性：
            - id: 命令唯一标识符
            - future: 异步 future 可通过 await 等待执行结果
            - cancel: 可调用对象（callable），调用后尝试取消该命令
    """
    ...

def charge_relay(relay_state: bool) -> bool:
    r"""
    @brief 控制充电继电器开关
    @param bool: 控制充电继电器  true为开 false为关
    @return bool: 是否成功
    """
    ...

async def async_charge_relay(relay_state: bool) -> bool: 
    r"""
    @brief 控制充电继电器开关
    @param bool: 控制充电继电器  true为开 false为关
    @return bool: 是否成功
    """
    ...


def async_topo_move_task(arg1: TopoPose) -> AsyncTopoMoveTaskTask:
    r"""
    @brief 下发拓扑路径移动任务 小车会根据地图规划拓扑路径 执行任务  任务会以队列形式执行
    @param TopoPose: 地图点位
    @return 返回一个 SimpleNamespace 对象，包含以下属性：
            - id: 命令唯一标识符
            - future: 异步 future 可通过 await 等待执行结果
            - cancel: 可调用对象（callable），调用后取消所有拓扑移动任务
    """
    ...


