<div align="center">
  <h1>T170 Factory Automation</h1>
  <p><b>轮式双臂机器人工厂上下料系统</b></p>
  <p>调度任务执行 · 双臂视觉伺服 · 标定与运维</p>
  <p>
    <img src="https://img.shields.io/badge/C%2B%2B-20-00599C?style=flat-square" alt="C++20">
    <img src="https://img.shields.io/badge/Python-3-3776AB?style=flat-square" alt="Python 3">
    <img src="https://img.shields.io/badge/Platform-Linux%20ARM64-333333?style=flat-square" alt="Linux ARM64">
  </p>
  <p>
    <a href="docs/architecture/REPOSITORY_GUIDE.md">系统架构</a> ·
    <a href="docs/deployment/CONFIGURATION.md">部署配置</a> ·
    <a href="docs/calibration/PLACEMENT_WIZARD.md">标定流程</a> ·
    <a href="docs/README.md">技术文档</a>
  </p>
</div>

---

## 系统概述

面向 T170 轮式双臂机器人的 CNC 工厂上下料应用。系统接收外部调度任务，完成导航、目标识别、抓取、放置及任务反馈，支持上料、工序转运、成品下料和充电流程。

```text
料盘一 ── 上料 ──▶ 半加工工位 ── 转运 ──▶ 精加工工位 ── 下料 ──▶ 料盘二
```

机器人身份、设备与标定独立配置，作业点位由调度下发。机器人与机床不固定绑定。

| 子系统 | 职责 |
| --- | --- |
| 任务执行 | 调度接入、导航、上下料编排、充电与异常恢复 |
| 视觉定位 | 头相机粗定位、手相机 XY 伺服、邻孔与部分孔环辅助放置 |
| 动作技能 | 料盘及传送带抓放，统一输入输出、逐手结果与取消接口 |
| 运动执行 | 轨迹规划、局部 IK、反馈、保位及抬升收手交接 |
| 标定运维 | 料盘抓放补偿与参考点校准、Web 单步控制和状态显示 |
| 运行统计 | 伺服 CSV、调度事件、阶段耗时与离线报告 |

当前为首版工程集成阶段，采用源码部署；安装包与发布验收尚未完成。

## 软件架构

```text
调度接口 → 任务编排 → 动作技能 → 运动规划与执行 → 硬件适配
                        ↑              ↑
                     视觉观测 ──→ 伺服控制

应用层负责设备装配与生命周期，统计模块记录任务状态及执行过程。
```

```text
apps/                  主程序入口
src/                   应用、任务、技能、运动、控制、视觉及硬件实现
include/t170/          模块接口
python/                Python 视觉算法与部署配置工具
config/                公共、型号、工厂、单机及标定配置
assets/                模型与音频资源
dependencies/          SDK、预编译库及通用依赖
cmake/                 模块构建清单与依赖路径
deploy/                生产、维护启动脚本及服务模板
web_debug/             Web 调试台与标定页面
tools/                 控制客户端、预检及日志分析
tests/                 离线回归与测试样本
examples/skills/       标准技能接口示例
docs/                  架构、部署、算法与验证文档
packaging/             安装打包预留
```

模块边界见 [仓库指南](docs/architecture/REPOSITORY_GUIDE.md)，技能契约见 [接口规范](docs/architecture/SKILLS_INTERFACE_V1.md)。

## 环境与构建

| 项目 | 要求 |
| --- | --- |
| 目标平台 | Linux ARM64，匹配 T170 机器人运行环境 |
| 构建工具 | CMake ≥ 3.16，支持 C++20 的编译器 |
| C++ 依赖 | OpenCV、yaml-cpp、JsonCpp、librealsense2、pybind11、Python 开发库 |
| Python 依赖 | 见 [requirements.txt](python/requirements.txt) |
| 机器人依赖 | 手臂、腰腿、设备、底盘及调度 SDK，见 [依赖清单](dependencies/README.md) |

在已配置 SDK 与视觉环境的机器人主机上执行：

```bash
cmake -S . -B build_robot -DCMAKE_BUILD_TYPE=Release
cmake --build build_robot -j4
```

生成 `build_robot/t170_factory` 和配置校验工具 `build_robot/t170_config_dump`。Python 环境通过 CMake 参数 `HUMAN_INTERACTION_ENV_ROOT` 指定；启动脚本使用的 Python 可通过 `T170_PYTHON` 指定。

## 运行入口

```bash
# 维护模式：初始化设备，等待单步、标定或人工命令
./deploy/run_maintenance.sh

# 生产模式：初始化设备，进入自动调度
./deploy/run_production.sh
```

同一机器人仅运行一个主程序进程。启动脚本读取 `config/deployment.yaml`，合成独立运行目录并输出 `T170_RUNTIME_DIR`。

Web 调试台使用同一运行目录，在另一终端启动；将路径替换为主程序启动输出中的实际值：

```bash
T170_RUNTIME_DIR=/absolute/path/to/runtime ./deploy/run_web.sh
```

Web 默认端口为 `8080`，机器人命令端口为 `8099`。参数、模式及显式部署入口见 [部署指南](docs/deployment/CONFIGURATION.md)。

## 配置管理

| 路径 | 内容 |
| --- | --- |
| `config/deployment.yaml` | 当前部署选择 |
| `config/deployments/` | 配置层与资源引用清单 |
| `config/defaults/` | 公共控制与策略参数 |
| `config/robot_models/` | 型号规格与运动学模型 |
| `config/sites/` | 工厂工位、料盘、传送带与视觉配置 |
| `config/robots/` | 单机身份、设备、相机与准备位 |
| `config/calibration/` | 单机手眼、抓放补偿与参考点 |

参数和标定保持外置。修改来源配置后重新生成运行目录并加载；运行目录中的合成文件不作为长期调参入口。当前合成器按字段唯一归属合并，重复定义会被拒绝。

日志、调试图和报告保存在运行目录；部署来源与生效参数分别记录于 `deployment_resolved.json`、`effective_config.yaml`。

## 离线验证

在具备测试依赖的开发环境中执行，无需连接机器人：

```bash
cmake -S . -B build_tests -DT170_SERVO_TESTS_ONLY=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build_tests -j4
ctest --test-dir build_tests --output-on-failure
```

离线检查覆盖控制、轨迹交接、孔位定位、配置、标定、技能、任务接口和统计。完整 SDK 链接、实机精度、连续运行成功率及节拍单独验收。

## 技术文档

| 文档 | 内容 |
| --- | --- |
| [系统架构](docs/architecture/REPOSITORY_GUIDE.md) | 模块职责、调用关系与目录说明 |
| [配置与部署](docs/deployment/CONFIGURATION.md) | 配置来源、运行目录与启动方式 |
| [视觉伺服](docs/WRIST_VISUAL_SERVO_ALGORITHM.md) | 图像映射、控制、目标定位与动作交接 |
| [技能接口](docs/architecture/SKILLS_INTERFACE_V1.md) | 标准输入输出、阶段、错误与逐手结果 |
| [料盘标定](docs/calibration/PLACEMENT_WIZARD.md) | 抓取与放置校准流程 |
| [调度流程](docs/dispatch_flow.md) | 上料、转运、下料及充电交互 |
| [发布验收](docs/architecture/RELEASE_ACCEPTANCE.md) | 构建、部署、回滚与现场验收要求 |
