"""
battery RPC 客户端 stub 模块

该模块为 pybind 导出的 C++ 接口提供类型声明。
自动生成的文件，请勿手动修改。
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union

HandleBatteryDev = int


class ChargeStatus(Enum):
    CHARGE_STATUS_NONE = 0
    CHARGE_STATUS_CHARGING = 1
    CHARGE_STATUS_FULL = 2


class BatteryState:
    timestamp_ns: int
    r""" Timestamp (nanoseconds)"""
    voltage: float = 0.0
    r""" Voltage (Volts)"""
    current: float = 0.0
    r""" Current (Amperes)"""
    charge: float = 0.0
    r""" Current charge on battery (mAh)"""
    capacity: float = 0.0
    r""" Battery capacity (mAh)"""
    design_capacity: float = 0.0
    r"""  Rated battery capacity (mAh)"""
    percentage: float = 0.0
    r""" Power percentage"""
    power_supply_status: ChargeStatus
    r""" Charge Status"""
    battery_temp: float
    r""" Battery temperature, unit: °C"""
    battery_id: str = ""
    r""" Battery Id"""

    @staticmethod
    def from_json(json_str: str) -> "BatteryState": 
        ...

    def to_json(self) -> str: 
        ...





def battery_state(id: int, cb: Callable[[BatteryState], None]) -> None: 
    r"""
    @brief 获取电池状态, 使用回调函数获取
    @param cb: 回调函数, 函数接收一个BatteryState作为输入
    """
    ...

def get_all_battery_dev() -> List[HandleBatteryDev]:
    ...



