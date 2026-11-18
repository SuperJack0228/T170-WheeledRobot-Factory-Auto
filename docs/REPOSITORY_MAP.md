# 仓库目录地图

更新：2026-11-18；核对源码基线：`b78a061`。用途：快速找到代码和参数；详细解释见 [模块指南](architecture/REPOSITORY_GUIDE.md)，本轮问题见 [复核记录](validation/repository_review_20261118/RESULTS.md)。

## 一条主线

```text
外部调度 → integration/dispatch → tasks → skills → motion → hardware → 机器人
                                  ↑        ↑          ↑
                           app 装配与管理   vision → control

网页 → web_debug/server.py → 本机 C++ 命令服务 → app → 标定或单步技能
```

这是职责关系示意。真实闭环中，vision 将带时间戳的观测送给 motion 管理的会话，motion 调用 control 计算修正，再解算、下发并读取反馈。

## 目录与用途

```text
T170 Factory Automation/
├── apps/                         可执行程序入口
│   └── t170_factory_main.cpp     委托 app::run，生产与维护共用同一主程序
├── src/                          C++ 实现
│   ├── app/                      程序装配、生命周期、命令权限、本机服务
│   ├── integration/dispatch/     厂商调度 SDK 的协议适配与任务收信
│   ├── tasks/                    上料/转运/下料/充电，导航、申请、任务反馈
│   ├── skills/                   可复用的抓放动作
│   │   ├── tray/                 料盘选目标、伸手、下降、抓取/放置
│   │   ├── conveyor/             传送带抓取、放置与独立撤离
│   │   └── common/               夹爪确认、抬升、准备位及腰臂协同
│   ├── vision/                   C++ 视觉流水线
│   │   ├── camera/               RealSense 采集、流配置和时间戳
│   │   ├── detection/            检测接口及模型登记
│   │   ├── geometry/             标记、孔环、邻孔和料盘几何定位
│   │   ├── tracking/             手相机观察线程、跟踪和观测提交
│   │   └── bridge/               C++ 调用 Python、数据与可视化转换
│   ├── control/                  图像映射、延迟补偿、PD、制动及下降协调
│   ├── motion/                   新 SDK 适配、IK、规划、播放、反馈与保位
│   ├── hardware/                 电机、夹爪、腰头、底盘及全身使能适配
│   ├── calibration/              料盘抓放标定会话和命令
│   ├── telemetry/                调度事件、耗时、状态及控制台记录
│   ├── config/                   生效参数快照与对比
│   ├── common/                   公共类型、路径、日志、数学、配置解析及伺服 CSV
│   └── servo/                    当前仍在使用的伺服配置解析器
├── include/t170/                 自有模块公开头，与 src 的职责对应
├── python/
│   ├── t170_vision/              YOLO、PnP/mask/框中心、料盘/传送带几何
│   └── t170_common/              部署配置合成、机型准备和路径管理
├── config/                       外置参数与真实标定，见下表
├── dependencies/
│   ├── sdk/arm/                  当前 Ti5_Robotics 头文件与 Linux 动态库
│   ├── sdk/arm_legacy/           旧手臂 SDK 归档，不参与正式链接
│   ├── sdk/lower_body/           腰腿 SDK 源码
│   ├── sdk/{device,chassis_seer}/ 交付头文件；完整实现仍依赖机器人 /opt 环境
│   ├── libraries/                Eigen、Unitree、串口库
│   └── manifest.json             依赖来源、文件摘要与验证状态
├── assets/                       模型权重与提示音
├── deploy/                       生产/维护/Web 启动脚本和 systemd 模板
├── web_debug/                    网页后端、操作页面、标定页面
├── tools/                        配置生成、SDK 纯计算检查、CLI、预检和日志分析
├── tests/                        实际离线测试及 fixtures，当前主要位于目录根层
├── examples/skills/              标准技能接口调用示例
├── cmake/                        各模块源码清单、依赖选择与离线检查
├── CMakeLists.txt                构建入口
├── docs/                         当前指南、复核/验证记录及历史现场资料
├── packaging/                    安装打包预留，尚未实现
└── .github/workflows/            CI 预留，尚未实现
```

## 改参数去哪里

| 需要调整的内容 | 来源 |
|---|---|
| 公共控制参数、门限和策略 | `config/defaults/factory_baseline.yaml` |
| 3 kg / 5 kg 臂型、URDF、DH、模型限位 | `config/robot_models/t170c_v2_3kg/`、`t170c_v2_5kg/` |
| 共用运动/相机规格 | `config/robot_models/t170/`；其中旧 `Robot_Arm_Model.yaml` 已退出运行链路 |
| 工厂料盘、传送带、导航及检测参数 | `config/sites/<site_id>/` |
| 单机身份、相机 SN、电机绑定、TCP、准备位 | `config/robots/<robot_id>/` |
| 手眼、头外参、料盘抓放参考 | `config/calibration/<robot_id>/` |
| 本次选择哪台机器人、哪种臂型、哪个场景 | `config/deployment.yaml` → `config/deployments/<robot_id>.yaml` |

生成链路：`deploy/runtime_context.sh` → `tools/resolve_deployment.py` → `python/t170_common/deployment.py` → 源码树外的运行目录。重复参数键会被拒绝，不是任意后写覆盖前写。

运行目录保存合成配置、`deployment_resolved.json`、`effective_config.yaml`、日志和调试结果；代码/模型等仍有链接指向源码树。任务标定目录链接到选定单机数据包，保存标定会更新该数据包。当前不是独立安装包。

查参数归属用 `python3 tools/parameter_index.py <参数名或关键词>`。安全重载、档案切换见 [参数操作](deployment/PARAMETER_OPERATIONS.md)；取图、语言桥接及并发说明见 [完整流水线](architecture/PIPELINE_AND_RUNTIME.md)。

## 常见问题从哪个文件开始读

| 想了解什么 | 首选入口 |
|---|---|
| 程序怎样启动、进入调度 | `src/app/application.cpp`、`src/app/hardware_factory.cpp` |
| 自动任务怎样安排 | `src/tasks/automation.cpp`，再看 `load.cpp` / `transfer.cpp` / `unload.cpp` |
| 抓放接口如何调用、如何报告左右手结果 | `include/t170/skills/{tray,conveyor,types}.h` |
| 料盘抓放具体动作 | `src/skills/tray/service.cpp` → `cycle.cpp`，再按阶段查对应文件 |
| 传送带抓放与撤离 | `src/skills/conveyor/execution.cpp`，双手顺序另看 `src/tasks/pick_sequence.cpp` |
| 手相机观测到运动修正 | `src/vision/tracking/wrist_observer.cpp` → `src/motion/servo_executor.cpp` → `src/control/visual_servo.cpp` |
| 被遮挡孔位怎样定位 | `src/vision/geometry/tray_localizer.cpp`、`tray_registration.cpp` |
| 新 SDK、模型角与电机角 | `src/motion/vendor_arm.cpp`、`arm.cpp`、`arm_configuration.cpp`、`arm_io.cpp` |
| 合爪后的保位与抬升交接 | `src/skills/common/grasp_finish.cpp`、`src/motion/final_hold.cpp`、`retreat.cpp` |
| 网页标定如何工作 | `web_debug/static/calibration.js` → `src/calibration/placement_session.cpp` |

## 阅读时避免混淆

- `include/t170/skills/legacy/` 是仍在使用的内部兼容接口，不可按名称直接删除。
- 展示动作仍在 `src/app/diagnostic_gestures.cpp`；已退役命令在登记表保留名称，用于返回说明，不等于仍有对应执行实现。
- 普通轨迹为 5 ms 采样；伺服配置目标周期目前为 10 ms，视觉观察是左右轮询、耗时独立，不代表 YOLO 每 10 ms 完成一次。
- 新 SDK 已接入源码；Linux 完整链接及真机回归尚未完成，四份旧料盘抓放参考仍待新模型下复核。
- `packaging/`、CI 及部分预留子目录只有骨架。现有测试不主要放在 `tests/unit/`、`tests/integration/`。

构建与启动见 [部署指南](deployment/CONFIGURATION.md)，新库计算检查见 [SDK 接入说明](deployment/ARM_SDK_MIGRATION.md)。
