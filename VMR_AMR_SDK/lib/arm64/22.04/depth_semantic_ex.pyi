"""
depth_semantic RPC 客户端 stub 模块

该模块为 pybind 导出的 C++ 接口提供类型声明。
自动生成的文件，请勿手动修改。
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union
from py_client_common_ex import ClientHandle

HandleDepthSemanticDev = int



class Mat:
    rows: int = 0
    r""" 图像 高"""
    cols: int = 0
    r""" 图像 宽"""
    channels: int = 1
    r""" 通道数 1=灰度 3=RGB"""
    data: list[int]
    r""" 图像实际数据"""

    @staticmethod
    def from_json(json_str: str) -> "Mat": 
        ...

    def to_json(self) -> str: 
        ...


class BoundingBox:
    r"""
       坐标结构体
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
    r""" 时间戳 单位 us"""
    original_img: Mat
    r""" AI识别结果图"""
    class_names: list[str]
    r""" 图中所有类别结果列表"""
    obj_box: list[BoundingBox]
    r""" 每个物体的框在图像中的坐标"""
    obj_dis: list[float]
    r""" 每个物体最近的点的距离，如果没有点则为-99"""
    pcloud: PointCloud
    r""" 点云结构体"""

    @staticmethod
    def from_json(json_str: str) -> "RecognitionResult": 
        ...

    def to_json(self) -> str: 
        ...





def get_all_depth_semantic_dev(handle: ClientHandle) -> List[HandleDepthSemanticDev]:
    ...


def recognition_state(handle: ClientHandle, id: int, cb: Callable[[RecognitionResult], None]) -> None: 
    r"""
    @brief 实时获取语义识别结果
    @param cb: 回调函数, 函数接收一个RecognitionResult作为输入
    """
    ...

def set_rgb_enable(handle: ClientHandle, id: HandleDepthSemanticDev, value: bool) -> None:
    r"""
    @brief 设置rgb是否读取
    @param bool:  开关
    """
    ...



