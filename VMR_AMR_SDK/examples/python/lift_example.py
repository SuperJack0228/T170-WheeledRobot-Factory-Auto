"""
顶升设备(Lift) RPC 调用示例

演示内容：
1. 订阅顶升设备状态变化 (subscribe_lift)
2. 发送控制命令并等待执行结果 (call_lift_cmd)

控制命令说明：
- lift_status 作为命令参数时表示目标位置/高度
- 示例中设置为 0 表示下降到地面位置
"""
import sys
import asyncio

import py_client_common as rpc_common
import lift

CUR_LIFT_STATE = 0

async def subscribe_lift():
    """
    订阅顶升设备状态变化

    服务器会定期推送 VmrLiftInfo，包含：
    - timestamp_ns: 纳秒级时间戳
    - lift_status: 当前顶升位置状态
    """
    def lift_callback(state: lift.LiftInfo):
        global CUR_LIFT_STATE
        print("callback timestamp ", state.timestamp_ns / 1e9)
        print("callback status ", state.lift_status)
        CUR_LIFT_STATE = state.lift_status

    lift.lift_state(lift_callback)


async def call_lift_cmd():
    """
    发送顶升控制命令并等待执行结果

    参数:
        dev_handle: 顶升设备句柄

    说明:
        lift_status 表示目标位置/高度
        设置为 0 表示下降到地面位置
    """

    global CUR_LIFT_STATE
    cmd = lift.LiftInfo()
    if CUR_LIFT_STATE == 0:
        cmd.lift_status = 1 # 目标位置：0 表示下降到地面, 1 表示最高
    else:
        cmd.lift_status = 0

    task = lift.async_lift_ctr(cmd)
    print(f"send command, target lift_status: {cmd.lift_status}, wait for 5 second...")
    success = False

    try:
        result = await asyncio.wait_for(task.future, timeout=5.0)
        print("lift command execute result", result.data)
        success = True
    except asyncio.TimeoutError:
        print("cmd timeout")

    return success


async def main():
    """
    主流程:
    1. 初始化 RPC 客户端连接
    2. 获取顶升设备句柄
    3. 订阅状态 3 秒（观察是否正常接收）
    4. 发送下降命令到位置 0
    """
    conf = rpc_common.ClientConfig()
    conf.ip = "192.168.22.244"
    conf.port = 9000
    # 注：init_client 应在程序启动时调用一次
    if not rpc_common.init_client(conf):
        print("connect fail")
        return

    # 获取顶升设备句柄
    handle_exist = lift.has_lift_dev()

    if not handle_exist:
        print("no lift device found")
        return

    print(f"lift exist")

    try:
        # 订阅状态 3 秒，观察是否正常接收推送
        await subscribe_lift()
        await asyncio.sleep(3)
        # 发送下降到位置 0 的命令
        await call_lift_cmd()
        await asyncio.sleep(3)

    except Exception as e:
        print(f"err: {e}", file=sys.stderr)
        raise


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        print("Error:", e, file=sys.stderr)
    finally:
        print("close")
        rpc_common.close_client()
