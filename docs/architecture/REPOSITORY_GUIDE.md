# 仓库、模块与运行资源

更新：2026-11-17。以本地源码为准；现场行为基线为 `9e023e7`，本次为该基线之后的整理工作区，尚未完成机器人端全量构建和实机回归。

## 先理解这条调用链

`apps → app → tasks → skills → motion/control → hardware`。视觉从 `vision` 提交观测，统计由 `telemetry` 记录，调度通信由 `integration/dispatch` 适配。

例如接到下料任务：tasks 接收本次机床/料盘点位，导航到传送带，感知选件，调用 conveyor 抓取技能，再导航到料盘调用 tray 放置技能。技能组织伸手、对准、夹爪和撤离；motion 规划、解算、下发并读取反馈；control 计算图像误差对应的修正。不会为每台机器人复制一套算法。

## 目录对应什么

| 目录 | 实际内容 |
| --- | --- |
| `apps/` | 正式主程序，委托 app 装配与运行 |
| `include/t170/` | 自有模块的公开头，名称与 src 对应；internal 头仅供实现协作 |
| `src/app/` | 硬件所有权、生命周期、本机 JSON 服务、命令权限、维护单步和演示 |
| `src/tasks/` | 自动上下料、导航、充电、任务恢复、逐手抓放顺序 |
| `src/integration/dispatch/` | 外部调度客户端适配和收信线程；不负责机械臂动作 |
| `src/skills/` | 料盘/传送带抓放标准入口与公共夹爪、抬升、腰臂协同 |
| `src/motion/` | 运动学适配、规划、轨迹执行、反馈、伺服会话、保位和撤离 |
| `src/control/` | IBVS 图像映射、延迟补偿、PD、制动和下降协调 |
| `src/vision/` | C++ 相机、Python 桥、手相机观察器、料盘几何定位 |
| `python/t170_vision/` | Python 检测/几何/头变换实现；保留 PnP 与 mask 路线 |
| `python/t170_common/` | 路径与部署配置合成，无电机初始化 |
| `src/calibration/` | 料盘抓/放标定引擎、命令与动作锁；页面仍在 web_debug |
| `src/telemetry/` | 伺服日志、调度计时、状态面板和控制台捕获 |
| `src/config/` | 生效参数快照/对比；现有解析器仍在 common/config.cpp、servo/config.cpp |
| `src/common/` | 路径、日志、位姿数学和现有共用配置类型 |
| `src/hardware/` | 电机总线、夹爪、腰头、底盘、全身使能的适配 |
| `config/` | 明文外置参数、真实标定、部署选择与 schema，见部署指南 |
| `assets/models/` | 当前两份模型；archive/ 和 legacy/ 保留历史模型，未自动启用 |
| `assets/audio/` | 提示音，保留展示/维护功能所需音频 |
| `deploy/` | 生产、维护、分层配置启动脚本与 systemd 模板 |
| `tools/` | CLI、预检、日志分析、配置导出、定位回放、头变换标定工具 |
| `tests/` | 离线回归、固定证据 fixture、明确标识的 SDK doubles |
| `examples/skills/` | 类型化技能调用示例，不是新的调度服务 |
| `cmake/` | 模块构建清单、离线检查与源码身份生成 |
| `web_debug/` | 原网页服务/页面；本轮完全未修改 |
| `docs/` | 当前说明、保留的现场事实、SDK 评估和分日期验证记录 |
| `dependencies/sdk/` | 同事交付的手臂、腰腿、设备和底盘 SDK，按功能分类 |
| `dependencies/libraries/` | Eigen、Unitree 和串口依赖；按实现/架构存放 |
| `dependencies/manifest.json` | 本地文件摘要和仍由机器人环境提供的 SDK 说明 |
| `packaging/`、`.github/` 等只有 .gitkeep 的目录 | 后续安装包/CI 的预留骨架，当前没有正式发布能力 |

## 模块内部怎样读

motion 的 `planning.cpp` 生成轨迹，`trajectory_executor.cpp` 播放，`feedback.cpp` 读取实际状态；`servo_*`、`observation_session.cpp`、`final_hold.cpp` 管理闭环和指令交接；`retreat.cpp` 管理竖直脱离及关节准备位返回。局部 IK 在 `pose_solver.cpp`，手臂 SDK 适配在 `kinematics_adapter.cpp`。

skills 的公开接口是 `include/t170/skills/{tray,conveyor,types}.h`。输入包含目标、左右手选择、持件状态及元数据；输出为逐手结果、阶段、错误码及副作用，详见 [接口 v1](SKILLS_INTERFACE_V1.md)。`skills/legacy/` 中仍有正在使用的内部运行结构，并非可随意删除的废弃实现。

tasks 的 `automation.cpp` 分派任务；`load/transfer/unload/charging` 是任务流程；`assignment` 读取动态点位，`navigation` 判断到站，`pick_sequence/place_sequence` 排列申请与动作通知。`robot_client_adapter.cpp` 和 `chassis_seer.cpp` 分别隔离厂商调度/底盘头。

应用 State 持有设备和服务，任务借用端口。主程序不直接计算像素修正，也不在每个技能里重建一套设备。

## 保留和退役边界

保留挥手/鞠躬/迎宾/音频、腰腿 SDK、所有模型、PnP/mask 与指定的四份现场文档。删去旧原图抓取、抖动/中心/精度实验、手动整圈、纯类别 YOLO 门禁、逐行偏置及料盘开环回退。

传送带放置的正式开环路线保留；本轮不改变其业务。旧命令在登记表保留 retired 名称用于向未修改的网页解释“已退役”，没有保留其执行实现。网页参数页本轮仍在，待单独改造。

原日志、图片、临时会话和备份已独立归档，不再随源码管理。当前有效标定仍在 config/calibration。旧文档的有效原理归入当前指南，必要历史在 Git/外部备份中查询，不重复存一整套过时指南。

## 尚未完成

参数解析和部分兼容命名/全局状态仍需后续收口；字段唯一归属合成尚不支持任意覆盖。SDK 目录已统一；来源/ABI 核对、Linux ARM64 完整链接、安装路径、依赖锁定、CI、CPack/DEB、升级回滚与新机验收还未完成。当前是源码树 overlay，不能把目录整理完成称为正式生产发布。
