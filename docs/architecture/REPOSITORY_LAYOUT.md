# 第一版目标目录与模块边界

这是三步迁移完成后的目标，不是当前已经完成的源码分布。精确目录见 [layout.csv](layout.csv)，来源与迁入步骤见 [模块表](MODULE_MIGRATION.md)。

## 最终目录

```text
T170-WheeledRobot-Factory-Auto/
├── CMakeLists.txt
├── CMakePresets.json                 # 第三步提供已验证的构建入口
├── README.md                        # 当前版唯一入门索引
├── apps/
│   └── t170_factory_main.cpp         # 主程序，仅组装和启动
├── include/t170/
│   ├── app/                         # 服务、生命周期
│   ├── common/                      # 时间/单位/坐标等基础类型
│   ├── config/                      # 配置、校验、快照
│   ├── vision/                      # 相机、检测、几何、观测
│   ├── control/                     # 纯控制器
│   ├── motion/                      # 规划/执行/反馈/伺服会话
│   ├── hardware/                    # 设备适配
│   ├── skills/                      # 抓放技能
│   ├── tasks/                       # 上料/转运/下料/充电
│   └── integration/                 # 外部调度协议
├── src/
│   ├── app/
│   ├── common/
│   ├── config/
│   ├── vision/
│   │   ├── camera/
│   │   ├── detection/
│   │   ├── geometry/
│   │   ├── tracking/
│   │   └── bridge/
│   ├── control/
│   ├── motion/
│   ├── hardware/
│   ├── skills/
│   │   ├── common/
│   │   ├── tray/
│   │   └── conveyor/
│   ├── tasks/
│   └── integration/dispatch/
├── python/
│   ├── pyproject.toml               # 第三步完成安装元数据/环境约束
│   ├── t170_common/
│   └── t170_vision/
│       ├── detection/
│       ├── geometry/
│       ├── calibration/
│       └── visualization/
├── config/
│   ├── defaults/
│   ├── robot_models/t170/
│   ├── sites/                       # 每个 site_id 一个目录
│   ├── robots/TH-T170C-0124/         # 当前真实机器人 ID
│   ├── calibration/TH-T170C-0124/
│   ├── deployments/                 # 选择型号/工位/机器/标定
│   └── schemas/
├── assets/
│   ├── models/                      # 清单、哈希及受控提供的权重
│   └── audio/
├── libs/lower_body/                 # 独立腰腿运动学库
├── third_party/
│   ├── eigen/
│   ├── ti5_arm/
│   ├── ti5_device/
│   ├── ti5_seer/
│   ├── unitree/
│   └── libserialport/
├── tests/
│   ├── unit/
│   ├── integration/
│   └── fixtures/
├── tools/
│   ├── debug_client.py              # 保留少量明确的公开 CLI
│   ├── servo_preflight.py
│   ├── diagnostics/
│   ├── calibration/
│   ├── analysis/
│   ├── web_debug/
│   └── dev/
├── deploy/
│   ├── scripts/
│   └── systemd/
├── packaging/
│   ├── cpack/
│   └── deb/
├── cmake/
├── .github/workflows/
└── docs/
    ├── architecture/
    ├── deployment/
    ├── validation/
    └── archive/
```

当前只建立目录骨架；图中的未来实现文件（如 `CMakePresets.json`、`pyproject.toml`、部署 YAML）尚未创建。

## 各层只负责自己的问题

| 层 | 回答的问题 | 边界 |
| --- | --- | --- |
| app | 程序如何启动、收到命令、退出？ | 组装 RobotContext 与组件，不选孔、不写控制算法 |
| tasks | 上料、转运或下料按什么步骤进行？ | 管理任务与持件状态，不直接发关节命令 |
| skills | 当前工位抓哪里、放哪里、用哪只手？ | 选目标并调用共用运动，保留工位差异 |
| motion | 如何伸手、下降、保位、抬升和回程？ | 统一执行器与发送所有权，处理反馈和中止 |
| control | 根据误差和反馈，下一步修多少？ | 纯像素映射/PD/制动/下降协调，不打开相机或 CAN |
| vision | 看到了什么、在哪个坐标系、何时看到？ | 带时间戳和质量的观测，不控制夹爪或电机 |
| hardware | 如何调用 SDK，读取与发送设备数据？ | 不包含料盘行列、传送带窗口和调度任务 |
| integration | 外部调度消息如何转换成内部请求/结果？ | 协议与任务动作隔离，不深入 AGV/CNC 内部 |
| config | 本次运行使用哪套型号、工位和单机参数？ | 完整校验、版本/来源清单、任务快照 |
| common | 大家如何约定单位、坐标、时间和日志？ | 基础工具不反向依赖业务、SDK 或 Python 推理 |

应用组装调度适配和任务执行器，任务调用技能，技能调用 vision 与 motion；motion 使用 control 和 hardware。调度消息通过明确接口进入任务层。vision 查询状态/发布观测通过窄接口连接 motion，不能形成 vision↔motion 的 CMake 链接循环。

当前 `src/servo/runtime.cpp` 实际负责实测回读、求解、关节发送和保位，因此最终进入 **motion 的伺服执行器**；`visual_servo.cpp` 是 control；`tray_localizer.cpp` 是 vision。按职责划分，不能仅凭旧文件夹名整体搬迁。

## 类型、状态和构建约定

- 对外头文件统一 `include/t170/<module>/...`；实现文件在 `src/<module>`，模块内部细节不全部暴露。
- 内部长度 m、角度 rad、图像 px，时间使用明确时钟域；位姿和观测要标明坐标系、采集时间、有效性。旧 mm/deg 文件在边界按原规则转换。
- RobotContext 拥有设备、运动执行器、视觉服务、配置快照和任务会话。逐步消除散落的 `g_hw`、可写 `g_move_cfg` 和混合业务全局。
- 每只手同时只有一个运动发送者。SDK 轨迹、伺服、最终保位、撤离轨迹的切换由 motion 管理；日志或观察线程无发送权限。
- 保留原错误语义和数值，先提供结构化 Result 包含错误码/阶段/侧别/持件状态，再单独治理重复错误码。
- CMake 显式列出源码与目标依赖，不用全仓库 glob 掩盖模块边界。先用静态/OBJECT 库，不要求做插件或每模块一个 `.so`。
- 控制核心保持可离线构建。配置解析可以依赖控制参数结构/校验；控制核心不调用全局配置加载器。
- 专用伺服 trace 允许使用控制记录类型，但应单独成目标，不能让路径/时间等基础库依赖整个控制器或 SDK。
- 第一步可保留明确标注的兼容适配和最终组装目标；第二步必须消除模块链接循环。第三步仅保留列入兼容清单的公开工具/启动转发，不留两套独立算法实现。

## 旧顶层目录的最终去向

| 当前目录/文件 | 最终处理 |
| --- | --- |
| `src/servo`、`include/servo` | 按 control / motion / vision / common 分解后移除旧内部目录 |
| `include/vision` | 迁入 `include/t170/vision`，内部调用切换后去除旧转发 |
| `orchestrator` | 分入 app/tasks/skills/hardware/integration/common，移除混合运行时目录 |
| `feeding_cylindrical_parts_alg` | Python 进入 `t170_vision`，参数进入 config，权重进入 assets |
| `board_yf100_aruco`、`board_belt_aruco`、`online_pose` | 算法进入 vision 包，几何参数进入 sites，独立调试入口进入 tools |
| `free_calib` | 实际标定进入 calibration，计算进入 vision，命令工具进入 tools；备份确认后归档 |
| `LowerBody` | `libs/lower_body` |
| `eigen3`、`lib`、`ti5-device-sdk`、`ti5-agv-seer` | 进入相应 third_party；与实际 `/opt` 安装依赖分清 |
| `AUDIO` | `assets/audio` |
| `web_debug` | `tools/web_debug` |
| 旧配置和旧头文件 | 完成等价对照与调用切换后移除，不长期双写 |
| 日志、采样、备份、历史说明 | 精选离线样本/历史文档/外部归档，不进入生产安装包 |

## 安装后的布局

源码树与安装树是两件事。第三步必须实际做到离开源码目录运行：

```text
/opt/t170-factory/...    程序、Python 包、版本化资源、依赖清单
/etc/t170-factory/...    选定的单机部署配置和标定
/var/log/t170-factory/   日志
/var/lib/t170-factory/   需要持久化的运行状态
```

具体 TGZ 版本目录与 DEB 文件归属由第三步打包规则实现；不能只复制一个可执行文件，也不能假定 SDK、Conda 或模型会随 Git 自动齐备。
