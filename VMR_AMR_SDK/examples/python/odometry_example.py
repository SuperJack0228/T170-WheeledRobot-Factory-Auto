"""
里程计(Odometry) RPC 调用示例

演示内容：
1. 订阅里程计状态变化 (subscribe_odom)
2. 获取里程计设备句柄

里程计状态信息：
- timestamp_ns: 纳秒级时间戳
- x, y: 里程计坐标 (米)
- theta: 里程计角度 (弧度)
- vx, vy: 线性速度 (米/秒)
- w: 角速度 (弧度/秒)
"""
import sys
import asyncio

import py_client_common as rpc_common
import odometry


async def subscribe_odom():
    """
    订阅里程计状态变化

    服务器会定期推送 VmrOdomInfo，包含：
    - timestamp_ns: 纳秒级时间戳
    - x, y: 里程计坐标 (米)
    - theta: 里程计角度 (弧度)
    - vx, vy: 线性速度 (米/秒)
    - w: 角速度 (弧度/秒)
    """
    def odom_callback(state: odometry.OdomInfo):
        print(f"callback timestamp: {state.timestamp_ns / 1e9:.3f}s")
        print(f"  position: x={state.x:.3f}m, y={state.y:.3f}m, theta={state.theta:.3f}rad")
        print(f"  velocity: vx={state.vx:.3f}m/s, vy={state.vy:.3f}m/s, w={state.w:.3f}rad/s")

    odometry.odom_state(odom_callback)


async def main():
    """
    主流程:
    1. 初始化 RPC 客户端连接
    2. 获取里程计设备句柄
    3. 订阅状态并观察推送数据
    """
    conf = rpc_common.ClientConfig()
    conf.ip = "192.168.22.244"
    conf.port = 9000
    if not rpc_common.init_client(conf):
        print("connect fail")
        return

    # 获取里程计设备句柄
    handle_exist = odometry.has_odom_dev()

    if not handle_exist:
        print("no odometry device found")
        return

    print(f"odometry exist")

    try:
        # 订阅状态 3 秒，观察是否正常接收推送
        await subscribe_odom()
        await asyncio.sleep(3000)

    except Exception as e:
        print(f"err: {e}", file=sys.stderr)
        raise


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        print("Error:", e, file=sys.stderr)
    print("close")
    rpc_common.close_client()