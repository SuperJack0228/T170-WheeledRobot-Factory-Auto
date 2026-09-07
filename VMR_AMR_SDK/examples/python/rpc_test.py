#!/usr/bin/env python3
# coding=utf-8
"""
RPC 客户端示例程序

演示如何使用 rpc_test 模块进行：
- cmd 异步调用
- topic 发布/订阅
"""
import rpc_test as rt
import py_client_common as rpc_common
import sys
import time
import asyncio


async def cmd_example_basic(handle):
    """示例1：基本 cmd 调用"""
    print("\n========== Example 1: Basic cmd call ==========")

    cmd_input = rt.CmdParam()
    # 直接赋值 Python list，会自动转换为 std::vector<uint8_t>
    cmd_input.a = [0x01, 0x02, 0x03, 0x04]

    # 异步调用 async_cmd_name，返回 task 对象
    task = rt.async_cmd_name(handle, 100, cmd_input)
    print(f"cmd task id: {task.id}")

    # 等待结果（最多5秒）
    try:
        result = await asyncio.wait_for(task.future, timeout=5.0)
        print(f"cmd result vec_data size: {len(result.vec_data)}")
        print(f"cmd result vec_data: {list(result.vec_data)}")
        print(f"cmd was cancelled: {result.cancel_flag}")
    except asyncio.TimeoutError:
        print("cmd timeout")

    return task


async def cmd_example_cancel(handle):
    """示例2：取消 cmd"""
    print("\n========== Example 2: Cancel cmd ==========")

    cmd_input = rt.CmdParam()
    cmd_input.a = [0x10, 0x20]

    task = rt.async_cmd_name(handle, 200, cmd_input)
    print(f"cmd task id: {task.id}")

    # 模拟一段时间后取消
    await asyncio.sleep(0.1)
    print("cancelling cmd...")
    task.cancel()

    # 等待结果
    try:
        result = await asyncio.wait_for(task.future, timeout=5.0)
        print(f"cmd result vec_data size (may be 0 if cancelled): {len(result.vec_data)}")
        print(f"cmd was cancelled: {result.cancel_flag}")
    except asyncio.TimeoutError:
        print("cmd timeout")

    return task


async def cmd_example_concurrent(handle):
    """示例3：并发执行多个 cmd"""
    print("\n========== Example 3: Concurrent cmds ==========")

    tasks = []

    # 提交3个并发任务
    for i in range(3):
        cmd_input = rt.CmdParam()
        cmd_input.a = [i * 10]
        task = rt.async_cmd_name(handle, 300 + i, cmd_input)
        print(f"submitted cmd task id: {task.id}")
        tasks.append(task)

    # 等待所有任务完成
    for i, task in enumerate(tasks):
        try:
            result = await asyncio.wait_for(task.future, timeout=10.0)
            print(f"cmd[{i}] result size: {len(result.vec_data)}")
            print(f"cmd[{i}] was cancelled: {result.cancel_flag}")
        except asyncio.TimeoutError:
            print(f"cmd[{i}] timeout")

    return tasks


async def topic_pub_example(handle):
    """示例4：topic 和 pub 示例"""
    print("\n========== Example 4: Topic and Pub ==========")

    # 构造输入参数
    inp = rt.RpcParam()
    inp.array_string_data[0] = "100"
    child = rt.ChildA()
    child.vec_data.append(101)
    inp.vec_data.append(child)
    inp.vec_string_data.append("hello")
    inp.c.vec_data.append(102)

    # 订阅回调
    def cb(sub_info: rt.RpcParam):
        print(f"sub_info.a: {sub_info.a}")
        print(f"sub_info.array_string_data[0]: {sub_info.array_string_data[0]}")
        if sub_info.vec_data:
            vd = sub_info.vec_data[0].vec_data
            print(f"sub_info.vec_data[0].vec_data[0]: {vd[0]}")

    rt.pub_name(handle, cb)

    # 同步/异步 RPC 调用
    ret = await rt.async_topic_name(handle, 100, inp)
    print(f"ret.a: {ret.a}")
    print(f"ret.array_string_data[0]: {ret.array_string_data[0]}")

    return ret


async def main():
    conf = rpc_common.ClientConfig()
    # 注：InitClient 应该在程序启动时调用一次
    # 这里假设已经初始化
    if not rpc_common.init_client(conf):
        print("connect fail")
        return

    # 获取句柄
    handles = rt.get_all_pallet_arrive_sensor()
    if not handles:
        print("no sensor handle")
        return

    handle = handles[0]
    print(f"Using handle: {handle}")

    try:
        # 运行各种示例
        await cmd_example_basic(handle)
        await cmd_example_cancel(handle)
        await cmd_example_concurrent(handle)
        await topic_pub_example(handle)

        print(f"\nHasHandlePalletArriveSensor: {rt.get_all_pallet_arrive_sensor()[0]}")

    except Exception as e:
        print(f"err: {e}", file=sys.stderr)
        raise


if __name__ == '__main__':
    try:
        asyncio.run(main())
        print("close")
        rpc_common.close_client()
    except Exception as e:
        print("err:", e, file=sys.stderr)
