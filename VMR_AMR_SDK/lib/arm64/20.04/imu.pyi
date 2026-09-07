"""
imu RPC ??? stub ??

???? pybind ??? C++ ?????????
???????????????
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union

HandleImuDev = int


class VmrImuType(Enum):
    kROS = 0
    kHipnuc = 1
    kMid360 = 2


class Quaternion:
    x: float
    y: float
    z: float
    w: float

    @staticmethod
    def from_json(json_str: str) -> "Quaternion": 
        ...

    def to_json(self) -> str: 
        ...


class Vector3:
    x: float
    y: float
    z: float

    @staticmethod
    def from_json(json_str: str) -> "Vector3": 
        ...

    def to_json(self) -> str: 
        ...


class VmrImuInfo:
    timestamp_ns: int
    type: VmrImuType
    orientation: Quaternion
    r""" ?????"""
    angular_velocity: Vector3
    r""" Angular velocity (rad/s)"""
    linear_acceleration: Vector3
    r""" Linear acceleration (m/s?)"""

    @staticmethod
    def from_json(json_str: str) -> "VmrImuInfo": 
        ...

    def to_json(self) -> str: 
        ...





def imu_state(id: int, cb: Callable[[VmrImuInfo], None]) -> None: 
    r"""
    @brief ????IMU????
    @param cb: ????, ??????ImuInfo????
    """
    ...

def get_all_imu_dev() -> List[HandleImuDev]:
    ...



