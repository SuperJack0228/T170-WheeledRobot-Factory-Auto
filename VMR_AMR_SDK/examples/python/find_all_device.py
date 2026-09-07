#!/usr/bin/env python3
# coding=utf-8
"""
设备发现脚本

扫描所有可用设备，调用所有 pyi 文件中定义的设备发现方法，
打印每个设备的类型和句柄。
"""
import sys
import py_client_common as rpc_common

# 导入所有设备模块
import battery
import depth_semantic
import exception
import imu
import lift
import light
import localization
import nav
import odometry
import robot_task


# 设备发现函数映射表：(模块名, 获取函数)
DEVICE_GETTERS = [
    ("battry", battery.get_all_battery_dev),
    ("depth_semantic", depth_semantic.get_all_depth_semantic_dev),
    ("exception", exception.has_exception_dev),
    ("imu", imu.get_all_imu_dev),
    ("lift", lift.has_lift_dev),
    ("light", light.has_light_dev),
    ("localization", localization.has_location_dev),
    ("nav", nav.has_nav_dev),
    ("odometry", odometry.has_odom_dev),
    ("robot_task", robot_task.has_robot_task_dev),
]


def discover_all_devices():
    """
    遍历所有设备模块，发现可用设备并打印句柄
    """
    print("=" * 60)
    print("设备发现扫描")
    print("=" * 60)

    for module_name, getter in DEVICE_GETTERS:
        try:
            handles = getter()

            print(f"[{module_name}]")
            print("    device list: " + str(handles))

        except Exception as e:
            print(f"\n[{module_name}]")
            print(f"  查询失败: {e}")

    print("\n" + "=" * 60)


def main():
    """
    主流程:
    1. 初始化 RPC 客户端连接
    2. 执行设备发现扫描
    """
    conf = rpc_common.ClientConfig()
    conf.ip = "192.168.22.244"
    conf.port = 9000

    if not rpc_common.init_client(conf):
        print("连接失败")
        return

    try:
        discover_all_devices()
    except Exception as e:
        print(f"err: {e}", file=sys.stderr)
        raise
    finally:
        rpc_common.close_client()


if __name__ == "__main__":
    main()
