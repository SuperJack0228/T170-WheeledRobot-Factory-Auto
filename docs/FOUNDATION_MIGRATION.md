# 第一批基础组件迁移

日期：2026-11-02。基于 `f6b8edf`。本批是代码组织、构建和路径迁移，不代表未经实机验证的抓放功能已正式验收。

## 主程序与诊断程序

主程序由 `t170c_debug` 更名为 **`t170_factory`**，入口从 `orchestrator/cpp/orch_hw_main.cpp` 移至 `apps/t170_factory_main.cpp`。JSON 命令、8099 端口、网页操作和调度任务语义保持原样；日志前缀和启动就绪标志改为 `t170_factory` / `T170_FACTORY_READY`。

`arm_plane_repeat` 移至 `tools/arm_plane_repeat.cpp`，默认不构建。它是真机诊断工具：归位、到 standby、下腰，在固定高度与腕角的水平面重复走网格点，输出编码器 FK 与目标的偏差和重复性。它不执行视觉闭环，不使用外部测量，不能据此证明绝对抓取精度。生产任务不调用它。

保留该工具便于定位运动学、跟随和重复性问题；不把它作为生产入口，也不在测试中执行它。

## 新组件及原职责

| 文件/目标 | 职责 |
| --- | --- |
| `include/t170/common/paths.h`、`src/common/paths.cpp` | 资源根目录、配置目录、日志目录和历史路径接口 |
| `include/t170/common/logging.h`、`src/common/logging.cpp` | 运行日志目的路径、共用本地时间格式；不改写现有日志系统 |
| `include/t170/common/config.h`、`src/common/config.cpp` | 原参数结构、默认值、YAML 加载、校验和启动参数摘要 |
| `include/move_box_config.h` | 兼容旧 include，并保留目标选择相关声明 |
| `src/move_box_config.cpp` | 原列分配、逐行姿态/偏置及目标选择策略，暂不迁入公共配置层 |
| `python/t170_common/paths.py` | 网页与预检工具共享相同环境变量和目录规则 |
| `t170::common` | 不依赖 SDK、相机或嵌入式 Python 的路径/日志静态库 |
| `t170::servo_core` | 复用现有控制器与局部运动学实现，源码算法保持原样 |
| `t170::configuration` | YAML 配置加载及原伺服配置序列化；显式依赖控制器的配置校验 |
| `t170::runtime` | 共用机器人实现 OBJECT 库，主程序与诊断程序不再复制源码和链接清单 |

时间、CSV 字段、伺服日志缓冲、调度日志保留和控制线程的发送/交接机制保持原样。配置摘要不再夹带目标策略模块的实时盘心状态，盘心仍由原策略维护。

## 路径规则

默认资源布局和 YAML 数值不变。环境变量应在进程启动前设置：

| 变量 | 含义 | 默认 |
| --- | --- | --- |
| `T170_ROOT` | 工程资源根目录 | 工作目录/可执行文件祖先中查找；最后尝试构建时源码位置 |
| `MARKET_SIMPLE_ROOT` | 旧根目录变量，保留兼容 | 仅在 `T170_ROOT` 非空值未设置时使用 |
| `T170_CONFIG_DIR` | 原顶层 `config/` 的替代位置 | `<root>/config` |
| `T170_LOG_DIR` | 运行日志根目录 | `<root>/logs` |

相对配置/日志目录按资源根目录解析；显式根目录的相对路径按启动工作目录解析。显式指定的根目录优先，不因缺文件而静默换另一台机器的资源。

`T170_CONFIG_DIR` 当前覆盖运动参数、相机 SN、手臂模型和顶层相机/手眼 YAML。它**不**把模型、料盘板参数、`free_calib/config`、Python 算法配置或外部 SDK 的设备自动配对配置搬到该目录。新位置必须提供所需完整文件集合。

伺服 CSV、会话快照、调度日志、阶段计时和手相机跟踪 CSV 使用日志根目录；网页自身运行记录改为 `<logs>/web`。原调试图片、原始帧和数据集位置保留。终端 `tee startup.log` 仍由调用者选择路径，网页先找 `<logs>/startup.log`，再兼容根目录 `startup.log`。

网页和 `servo_preflight.py` 同步使用上述覆盖变量。网页和机器人进程应使用同一套环境变量；修改变量后重启相关进程。网页静态文件仍相对网页脚本定位。

`project_root_dir()` 和原 `default_*_path()` 继续保留，实现在公共模块中；视觉桥接头文件仅转引路径头，不再负责实现。

## 配置加载

保持原字段、默认值、单位、转换和校验规则，未修改任何现场 YAML。

新增失败保护：先完整加载和校验临时配置，成功后才写回调用者；缺文件、格式错误、值不合法时保留旧配置。回归测试覆盖这些失败情况。

这仅解决加载失败的部分覆盖问题；成功重载仍沿用现有全局 `g_move_cfg` 写回方式。任务级不可变配置快照、并发读写治理和参数分层合并尚未在本批实现。不要将本次迁移理解为热重载已完全线程安全。

## 编译和启动

在机器人对应的 Linux/ARM64 与原 SDK/Python 环境中，先停止现有机器人程序，再更新本次全部源码并重新配置、编译：

```bash
cmake -S . -B build_robot -DT170_BUILD_DIAGNOSTICS=OFF
cmake --build build_robot -j4 --target t170_factory
./build_robot/t170_factory 2>&1 | tee startup.log
```

也可前台启动（参数原样转发）：

```bash
./deploy/run_factory.sh
```

进入调度仍显式传入 `--dispatch`；它会执行原调度初始化/归位行为。程序名更改不等于抓放功能已经经过实机验收。

旧构建目录中可能仍残留旧名 `t170c_debug` 文件，重新编译不会把它自动变成新程序；后续请使用新名。旧版本进程需要先结束，避免同时使用机器人设备。

如确实需要运行水平面诊断工具，单独启用和构建：

```bash
cmake -S . -B build_robot -DT170_BUILD_DIAGNOSTICS=ON
cmake --build build_robot -j4 --target arm_plane_repeat
```

此处只给编译命令，诊断程序运行会真实移动头、腰和双臂，应在现场按其工作范围使用。

Ti5 SDK 根目录现在是 CMake CACHE PATH，可通过 `-DTI5_DEVICE_SDK_ROOT=...` 等变量覆盖；默认仍为原 `/opt/ti5-*`，没有替换 SDK 版本或二进制。

## 自启动与网页

- 新部署模板：`deploy/t170-factory.service`，通过 `deploy/run_factory.sh --dispatch` 启动。
- 模板默认项目路径仍为 `/home/ti5robot/T170Clean`，现场应按实际目录调整；可选读取 `/etc/t170-factory/runtime.env`。模板尚不是自动安装器。
- 新开机脚本：`deploy/t170_factory_boot.sh`，从脚本位置或根目录变量定位项目。保留原设备等待、最多三次启动前重试和进入调度后不自动重启的策略。
- 旧 `deploy/t170c_boot.sh` 作为兼容转发保留，现有 cron 若调用该路径会转入新脚本。脚本同时检查旧名进程。
- 机器上已经安装的旧 systemd 服务不会因为 Git 文件改名自动更新。需要现场更新其启动命令或迁移到新服务；只启用一套自启动方式。当前没有执行任何系统服务安装或重启。
- 网页默认 `--robot-command` 已更新为 `./build_robot/t170_factory`，调试客户端命令不变。

## 验证与未完成事项

离线构建明确要求 YAML、OpenCV 和 Python 解释器，避免缺依赖时静默少跑测试。无需机器人 SDK、相机、CAN 或模型权重。现有六项回归加上 C++ 路径和 Python 工具路径集成，共八项测试。

```bash
cmake -S . -B build_servo_tests -DT170_SERVO_TESTS_ONLY=ON
cmake --build build_servo_tests -j4
ctest --test-dir build_servo_tests --output-on-failure
```

本机验证范围：八项离线回归；配置失败保留旧值；外部目录和旧环境变量兼容；网页配置与日志路径；启动器在含空格路径下的参数/环境/工作目录传递；Python 与 Shell 语法；相关 C++ 语法检查。

还使用外部 SDK/pybind 占位依赖检查了完整 CMake 构建图和诊断选项；这仅验证目标和链接规则，不是完整机器人 SDK 编译。部分 C++ 语法检查也隔离了平台/SDK 声明，不能替代机器人环境构建。

未连接或执行机器人。新料盘放置、传送带抓取和收手仍待实机验证；本地缺少 `best1027.pt` 的资源交付问题仍待补齐。当前没有制作安装包、安装服务、提交或推送本批改动。
