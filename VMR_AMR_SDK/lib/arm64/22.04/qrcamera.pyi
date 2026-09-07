"""
qrcamera RPC 客户端 stub 模块

该模块为 pybind 导出的 C++ 接口提供类型声明。
自动生成的文件，请勿手动修改。
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union



class QRPose2D:
    r"""
    @brief 二维码相对位姿
    @param float x: x 方向偏移, 单位 m
    @param float y: y 方向偏移, 单位 m
    @param float theta: 航向角偏移, 单位 rad
    """
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    @staticmethod
    def from_json(json_str: str) -> "QRPose2D": 
        ...

    def to_json(self) -> str: 
        ...


class QRCameraResult:
    r"""
    @brief 二维码相机识别结果
    @param uint64_t timestamp_ns: 时间戳 ns
    @param int32_t location: 相机安装位置 上相机为0 下相机为1
    @param bool detected: 是否识别到二维码
    @param string qr_info: 二维码完整内容
    @param string qr_type: 二维码类型
    @param string qr_id: 二维码 ID
    @param QRPose2D base_in_qr: 车身在二维码坐标系下的位姿
    @param QRPose2D camera_in_qr: 相机在二维码坐标系下的位姿
    @param float depth: 二维码深度, 单位 m
    @param int32_t memb_total_cnt: 二维码组合总成员数
    @param int32_t memb_obs_cnt: 当前识别到的成员数
    @param int32_t tag_type: 标签类型, 0=DataMatrix, 1=Apriltag
    @param int32_t encode_type: 编码类型
    """
    timestamp_ns: int
    location: int
    detected: bool = False
    qr_info: str
    qr_type: str
    qr_id: str
    base_in_qr: QRPose2D
    camera_in_qr: QRPose2D
    depth: float = 0.0
    memb_total_cnt: int = 0
    memb_obs_cnt: int = 0
    tag_type: int = 0
    encode_type: int = 0

    @staticmethod
    def from_json(json_str: str) -> "QRCameraResult": 
        ...

    def to_json(self) -> str: 
        ...


class QRCameraSwitchRequest:
    r"""
    @brief 二维码相机开关请求
    @param bool enable: true 打开二维码识别, false 关闭二维码识别
    @param int32_t location: 相机安装位置 上相机为0 下相机为1
    """
    enable: bool = False
    location: int = 1

    @staticmethod
    def from_json(json_str: str) -> "QRCameraSwitchRequest": 
        ...

    def to_json(self) -> str: 
        ...





def qrcamera_result(cb: Callable[[QRCameraResult], None]) -> None: 
    r"""
    @brief 实时获取二维码相机识别结果
    @param cb: 回调函数, 接收 QRCameraResult 参数
    """
    ...

def has_q_r_camera_dev() -> bool:
    ...


def set_qrcamera_enable(request: QRCameraSwitchRequest) -> bool:
    r"""
    @brief 开关二维码相机识别
    @param QRCameraSwitchRequest: 二维码相机开关请求
    """
    ...

async def async_set_qrcamera_enable(request: QRCameraSwitchRequest) -> bool: 
    r"""
    @brief 开关二维码相机识别
    @param QRCameraSwitchRequest: 二维码相机开关请求
    """
    ...



