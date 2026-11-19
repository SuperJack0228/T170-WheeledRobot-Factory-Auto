# 工厂任务与应用拆分：本地验证记录

日期：2026-11-16。主机：macOS/Darwin arm64。当前 HEAD：`9e023e775ecfb2d2742382f5a92427fc4f1e2907`；验证对象包含其后的本地配置、motion、skills 和本批任务/应用改动，尚未提交。

## 结果

离线构建成功；**34/34 项 CTest 通过，0 失败**。最终完整一轮耗时约 13.57 秒。原始记录见 [ctest.log](ctest.log)，机器可读摘要见 [summary.json](summary.json)。

```bash
cmake -S . -B /tmp/t170-deployment-build -DT170_SERVO_TESTS_ONLY=ON -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/t170-deployment-build -j 6
env PYTHONPATH=/tmp/t170-deployment-deps ctest --test-dir /tmp/t170-deployment-build --output-on-failure
```

以上临时目录是本机验证环境，不是部署依赖路径。测试环境具备 CMake、C++20、OpenCV、yaml-cpp、JsonCpp 和 PyYAML；网页接口测试在本机临时端口执行。没有连接机器人、相机、CAN、调度服务器或 SEER。

新增及扩展检查：

| 检查 | 实际覆盖 |
| --- | --- |
| `app_policy_test` | 明确模式、旧 `--dispatch` 兼容、非法参数、全部命令登记、生产权限和轮询不清到位记录的分类 |
| `factory_entrypoints_test` | 用无操作替身执行真实启动脚本；参数/根路径/含空格目录；系统服务生产入口；旧 67 个命令保留；CLI/Web 白名单和按钮有后端实现 |
| `factory_navigation_test` | 执行真实导航任务代码；途中站名提前变化不算完成、准备回调仅一次、最终状态 4、已在站点捷径、障碍/中止取消、失败与原生错误码 |
| `factory_assignment_test` | 执行真实点位选择代码；同一服务切换机床组、口二回退、缺点位拒绝、调度待命/充电覆盖、临时配置恢复 |
| `dispatch_adapter_test` | 真实适配器配测试专用 `robot_client.hpp` 替身；身份/申请超时、7 类原生异常映射、状态更新、关闭传输解除收信阻塞后 join |
| 标定 session/HTTP 测试扩展 | 真实标定会话及提取后的命令处理器；忙状态、停止确认、退出期间拒绝复位、缓存状态不读硬件；认证后的 capabilities 转发 |
| 模块边界和公共头检查扩展 | SDK 头仅在适配器；主程序只委托组装；权限判断先于动作锁/停止复位；旧实现不重复编译；公共端口不暴露 SDK |

既有伺服、撤离、技能、标定数学、孔定位、统计/报告、部署等价与路径检查也通过。它们验证软件行为和接口，不证明机械抓放精度。

## 真实实现编译与源码迁移核对

离线目标会编译新的真实 tasks、app、calibration、`chassis_seer.cpp` 和 `body_power.cpp`，以及此前的真实 motion/skills 实现。SEER 适配器使用仓库内真实 SEER 声明。本机缺少现场 `robot_client.hpp`，调度适配器仅在专用测试目标里使用明确标记的替身；替身目录没有进入生产构建的 include 路径。

`src/vision/bridge/python_runtime.cpp` 的嵌入 Python/GIL 适配仍需目标环境构建验证；本轮未通过假 Python/pybind11 头冒充验证。

对本批开始前备份的旧 `robot_runtime.cpp` 逐函数检查：

- 141 个函数均找到迁移后的实现位置，没有丢失。
- 忽略注释/空白、仅归一化明确的 `RobotClient`/`Seer` 端口名称与静态字段读取后，132 个函数体 token 完全相同。
- 其余 9 个为底盘服务获取、地图更新标记、位置/电量发布、状态线程循环与启动、全身电机适配、停止充电、调度主循环的连接关闭/线程回收；变更范围已人工检查。
- 原 `placement_session.cpp` 标定引擎、统计核心的声明/函数体 token 未改变；报告实现迁移后的文件与 HEAD 原报告文件字节一致。

逐函数去向及哈希见 [function_migration.json](function_migration.json)。这是对本批前本地备份的迁移审计，不等价于形式化证明，也不将协议端口替换假设当成真实 SDK 验收。

JavaScript 语法、改动 Python 的编译检查、启动脚本 `bash -n`、`git diff --check` 均通过。

## 外部验证待完成

1. 在匹配的 Linux/ARM64 SDK、Python 和运行依赖环境全量构建、链接 `t170_factory`，特别核对真实调度 SDK 的声明和解除阻塞契约。
2. 单实例验证生产/维护启动、状态查询、调度中止/退出、重新进入、标定会话和进程退出。
3. 对上料/转运/下料/充电、导航提前准备、双手与腰/底盘重叠流程做实机回归，再进行连续调度测试。

当前可以说明“拆分已有本地实现并通过离线检查”，不能说明“本轮已通过真机验收”或“第一版已经完成安装发布”。
