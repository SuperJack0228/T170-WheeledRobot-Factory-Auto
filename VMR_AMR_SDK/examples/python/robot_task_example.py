"""
机器人任务(RobotTask) RPC 调用示例

演示内容：
1. 订阅机器人任务状态变化 (subscribe_robot_task)
2. 发送导航任务并等待执行结果 (call_topo_move_task)

控制命令说明：
- pose_name: 目标点位名称
- action: 0:仅移动 3:充电 4:停止充电
"""
import sys
import asyncio

import py_client_common as rpc_common
import robot_task as robot_task


async def call_topo_move_task():
    """
    发送导航任务并等待执行结果

    参数:
        dev_handle: 机器人任务设备句柄

    说明:
        pose_name 表示目标点位名称
        action: 0 仅移动, 3 充电, 4 停止充电
    """
    pose = robot_task.TopoPose()
    # pose.pose_name = "Goal_96xQi"
    pose.pose_name = "Goal_rpiSI"
    pose.action = 0  # 0:仅移动 3:充电 4:停止充电

    # 异步调用 topo_move_task
    result = robot_task.async_topo_move_task(pose)
    print(f"cmd task id: {result.id}")
    success = False

    try:
        cmd_result = await asyncio.wait_for(result.future, timeout=60.0)
        if cmd_result.err_code:
            print(f"Error: {cmd_result.err_message}")
        print(f"cmd result {cmd_result.data}")
        success = True
    except asyncio.TimeoutError:
        print("cmd timeout")
        result.cancel()

    return success


async def main():
    """
    主流程:
    1. 初始化 RPC 客户端连接
    2. 获取机器人任务设备句柄
    3. 订阅状态 3 秒（观察是否正常接收）
    4. 发送导航任务
    """
    conf = rpc_common.ClientConfig()
    conf.ip = "192.168.22.244"
    conf.port = 9000
    conf.connect_timeout_ms = 3000
    conf.call_timeout_ms = 60000

    if not rpc_common.init_client(conf):
        print("connect fail")
        return

    handle_exist = robot_task.has_robot_task_dev()

    if not handle_exist:
        print("no robot task device found")
        return

    print(f"robot task device exist")

    try:
        # 发送导航任务
        await call_topo_move_task()
    except Exception as e:
        print(f"err: {e}", file=sys.stderr)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        print("Error:", e, file=sys.stderr)
