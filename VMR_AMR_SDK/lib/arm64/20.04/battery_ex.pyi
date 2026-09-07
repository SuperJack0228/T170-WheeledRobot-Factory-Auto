"""
battery RPC ??? stub ??

???? pybind ??? C++ ?????????
???????????????
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union
from py_client_common_ex import ClientHandle

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
    r""" Battery temperature, unit: ?C"""
    battery_id: str = ""
    r""" Battery Id"""

    @staticmethod
    def from_json(json_str: str) -> "BatteryState": 
        ...

    def to_json(self) -> str: 
        ...





def battery_state(handle: ClientHandle, id: int, cb: Callable[[BatteryState], None]) -> None: 
    r"""
    @brief ??????, ????????
    @param cb: ????, ??????BatteryState????
    """
    ...

def get_all_battery_dev(handle: ClientHandle) -> List[HandleBatteryDev]:
    ...



