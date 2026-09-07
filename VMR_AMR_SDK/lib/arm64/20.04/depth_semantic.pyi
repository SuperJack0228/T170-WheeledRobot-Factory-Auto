"""
depth_semantic RPC ??? stub ??

???? pybind ??? C++ ?????????
???????????????
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union

HandleDepthSemanticDev = int



class Mat:
    rows: int = 0
    r""" ?? ?"""
    cols: int = 0
    r""" ?? ?"""
    channels: int = 1
    r""" ??? 1=?? 3=RGB"""
    data: list[int]
    r""" ??????"""

    @staticmethod
    def from_json(json_str: str) -> "Mat": 
        ...

    def to_json(self) -> str: 
        ...


class BoundingBox:
    r"""
       ?????
    """
    x1: int
    y1: int
    x2: int
    y2: int

    @staticmethod
    def from_json(json_str: str) -> "BoundingBox": 
        ...

    def to_json(self) -> str: 
        ...


class PointXYZ:
    x: float
    y: float
    z: float

    @staticmethod
    def from_json(json_str: str) -> "PointXYZ": 
        ...

    def to_json(self) -> str: 
        ...


class PointCloud:
    points: list[PointXYZ]

    @staticmethod
    def from_json(json_str: str) -> "PointCloud": 
        ...

    def to_json(self) -> str: 
        ...


class RecognitionResult:
    timestamp_us: int
    r""" ??? ?? us"""
    original_img: Mat
    r""" AI?????"""
    class_names: list[str]
    r""" ??????????"""
    obj_box: list[BoundingBox]
    r""" ?????????????"""
    obj_dis: list[float]
    r""" ???????????????????-99"""
    pcloud: PointCloud
    r""" ?????"""

    @staticmethod
    def from_json(json_str: str) -> "RecognitionResult": 
        ...

    def to_json(self) -> str: 
        ...





def get_all_depth_semantic_dev() -> List[HandleDepthSemanticDev]:
    ...


def recognition_state(id: int, cb: Callable[[RecognitionResult], None]) -> None: 
    r"""
    @brief ??????????
    @param cb: ????, ??????RecognitionResult????
    """
    ...

def set_rgb_enable(id: HandleDepthSemanticDev, value: bool) -> None:
    r"""
    @brief ??rgb????
    @param bool:  ??
    """
    ...



