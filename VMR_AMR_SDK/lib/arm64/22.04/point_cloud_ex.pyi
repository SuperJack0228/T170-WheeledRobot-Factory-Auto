"""
point_cloud RPC 客户端 stub 模块

该模块为 pybind 导出的 C++ 接口提供类型声明。
自动生成的文件，请勿手动修改。
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union
from py_client_common_ex import ClientHandle



class PointXYZ:
    r"""
    @brief 基于小车坐标系的点云坐标
    @param float x: x轴坐标  单位m
    @param float x: y轴坐标  单位m
    @param float x: z轴坐标  单位m
    """
    x: float
    y: float
    z: float

    @staticmethod
    def from_json(json_str: str) -> "PointXYZ": 
        ...

    def to_json(self) -> str: 
        ...


class PointCloud:
    r"""
    @brief 基于小车坐标系的点云坐标
    @param int32_t location: 点云器件标识  (-1 ~ -8 为深度相机  -9 为前3d激光  -10 为后3d激光)
    @param std::vector<PointXYZ> points: 点云数组
    """
    location: int
    timestamp_ns: int
    points: list[PointXYZ]

    @staticmethod
    def from_json(json_str: str) -> "PointCloud": 
        ...

    def to_json(self) -> str: 
        ...





def point_cloud_pub(handle: ClientHandle, cb: Callable[[PointCloud], None]) -> None: 
    r"""
    @brief 实时获取点云状态,  使用回调函数获取
    @param cb: 回调函数, 接受一个 PointCloud 参数
    """
    ...

def has_point_cloud_dev(handle: ClientHandle) -> bool:
    ...



