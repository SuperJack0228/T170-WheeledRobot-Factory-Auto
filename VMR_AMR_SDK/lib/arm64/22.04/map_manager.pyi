"""
map_manager RPC 客户端 stub 模块

该模块为 pybind 导出的 C++ 接口提供类型声明。
自动生成的文件，请勿手动修改。
"""

from enum import Enum
from asyncio import Future
from typing import Optional, Callable, List, Dict, Any, Union



class ResultUploadMap:
    r""" 错误码，默认0表示成功"""
    err_code: int
    r""" 错误消息"""
    err_message: str
    r""" 返回数据"""
    data: str

class ResultDownloadMap:
    r""" 错误码，默认0表示成功"""
    err_code: int
    r""" 错误消息"""
    err_message: str
    r""" 返回数据"""
    data: str

class ResultChangeMap:
    r""" 错误码，默认0表示成功"""
    err_code: int
    r""" 错误消息"""
    err_message: str
    r""" 返回数据"""
    data: str

class MapInfo:
    name: str



class AsyncUploadMapTask:
    r"""
    @brief 上传地图
    @param map_name: 地图名称
    @param zip_data: 地图zip包
    @return 返回失败原因
    """
    id: int
    future: Future[ResultUploadMap]
    cancel: Callable[[], None]
class AsyncDownloadMapTask:
    r"""
    @brief 下载地图
    @param map_name: 地图名称
    @return 地图的zip包，如果为空，代表地图不存在
    """
    id: int
    future: Future[ResultDownloadMap]
    cancel: Callable[[], None]
class AsyncChangeMapTask:
    r"""
    @brief 切换地图
    @param map_name: 地图名称
    @return 返回失败原因，字符串空代表无成功
    """
    id: int
    future: Future[ResultChangeMap]
    cancel: Callable[[], None]


def has_map_manager() -> bool:
    ...


def get_map_list() -> list[str]:
    r"""
    @brief 获取地图列表
    @return 地图名称列表
    """
    ...

async def async_get_map_list() -> list[str]: 
    r"""
    @brief 获取地图列表
    @return 地图名称列表
    """
    ...


def get_current_map_name() -> str:
    r"""
    @brief 获取当前地图名称
    @return 地图名称
    """
    ...

async def async_get_current_map_name() -> str: 
    r"""
    @brief 获取当前地图名称
    @return 地图名称
    """
    ...


def async_upload_map(map_name: str, zip_data: str) -> AsyncUploadMapTask:
    r"""
    @brief 上传地图
    @param map_name: 地图名称
    @param zip_data: 地图zip包
    @return 返回失败原因
    """
    ...

def async_download_map(map_name: str) -> AsyncDownloadMapTask:
    r"""
    @brief 下载地图
    @param map_name: 地图名称
    @return 地图的zip包，如果为空，代表地图不存在
    """
    ...

def delete_map(map_name: str) -> str:
    r"""
    @brief 删除地图
    @param map_name: 地图名称
    @return 返回失败原因，字符串空代表成功
    """
    ...

async def async_delete_map(map_name: str) -> str: 
    r"""
    @brief 删除地图
    @param map_name: 地图名称
    @return 返回失败原因，字符串空代表成功
    """
    ...


def async_change_map(map_name: str) -> AsyncChangeMapTask:
    r"""
    @brief 切换地图
    @param map_name: 地图名称
    @return 返回失败原因，字符串空代表无成功
    """
    ...

def start_mapping(map_info: MapInfo) -> str:
    r"""
    @brief 开始建图
    @param map_name: 地图名称
    @return 返回失败原因，字符串空代表无成功
    """
    ...

async def async_start_mapping(map_info: MapInfo) -> str: 
    r"""
    @brief 开始建图
    @param map_name: 地图名称
    @return 返回失败原因，字符串空代表无成功
    """
    ...


def stop_mapping() -> str:
    r"""
    @brief 结束建图
    @return 返回失败原因，字符串空代表无成功
    """
    ...

async def async_stop_mapping() -> str: 
    r"""
    @brief 结束建图
    @return 返回失败原因，字符串空代表无成功
    """
    ...


def start_map_extension() -> str:
    r"""
    @brief 启动当前地图的地图续建功能(需定位成功)
    @return 返回失败原因，字符串空代表无成功
    """
    ...

async def async_start_map_extension() -> str: 
    r"""
    @brief 启动当前地图的地图续建功能(需定位成功)
    @return 返回失败原因，字符串空代表无成功
    """
    ...


def stop_map_extension() -> str:
    r"""
    @brief 停止当前地图的地图续建功能
    @return 返回失败原因，字符串空代表无成功
    """
    ...

async def async_stop_map_extension() -> str: 
    r"""
    @brief 停止当前地图的地图续建功能
    @return 返回失败原因，字符串空代表无成功
    """
    ...



