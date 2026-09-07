"""
电池(Battery) RPC 调用示例

演示内容：
1. 订阅电池状态变化 (subscribe_battery)
2. 获取电池设备句柄

电池状态信息：
- timestamp_ns: 纳秒级时间戳
- voltage: 电压
- current: 电流
- charge: 电量
- capacity: 容量
- design_capacity: 设计容量
- percentage: 百分比
- power_supply_status: 电源状态
- serial_number: 序列号
"""
import sys
import asyncio

import py_client_common as rpc_common
import battery


async def subscribe_battery(dev_handle):
    """
    订阅电池状态变化

    服务器会定期推送 VmrBatteryState，包含：
    - timestamp_ns: 纳秒级时间戳
    - voltage: 电压
    - current: 电流
    - charge: 电量
    - capacity: 容量
    - design_capacity: 设计容量
    - percentage: 百分比
    - power_supply_status: 电源状态
    - serial_number: 序列号
    """
    def battery_callback(state: battery.BatteryState):
        print(f"callback timestamp: {state.timestamp_ns / 1e9:.3f}s")
        print(f"  voltage: {state.voltage:.2f}V")
        print(f"  current: {state.current:.2f}A")
        print(f"  charge: {state.charge:.2f}Ah")
        print(f"  capacity: {state.capacity:.2f}Ah")
        print(f"  design_capacity: {state.design_capacity:.2f}Ah")
        print(f"  percentage: {state.percentage:.1f}%")
        print(f"  power_supply_status: {state.power_supply_status}")
        print(f"  battery_temp: {state.battery_temp}")
        print(f"  battery_id: {state.battery_id}")


    battery.battery_state(dev_handle, battery_callback)


async def main():
    """
    主流程:
    1. 初始化 RPC 客户端连接
    2. 获取电池设备句柄
    3. 订阅状态并观察推送数据
    """
    conf = rpc_common.ClientConfig()
    conf.ip = "192.168.22.244"
    conf.port = 9000
    # 注：init_client 应在程序启动时调用一次
    if not rpc_common.init_client(conf):
        print("connect fail")
        return

    # 获取电池设备句柄
    handles = battery.get_all_battery_dev()

    if not handles:
        print("no battery device found")
        return

    battery_handle = handles[0]
    print(f"Using battery handle: {battery_handle}")

    try:
        # 订阅状态 3 秒，观察是否正常接收推送
        await subscribe_battery(battery_handle)
        await asyncio.sleep(3)

    except Exception as e:
        print(f"err: {e}", file=sys.stderr)
        raise


if __name__ == "__main__":
    try:
        asyncio.run(main())
        print("close")
        rpc_common.close_client()
    except Exception as e:
        print("Error:", e, file=sys.stderr)
