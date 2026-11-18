# 配置选择、启动与调参

更新：2026-11-17。已统一资源来源并接入新手臂 SDK，尚未制作安装包。

部署一台新机器人时，先按 [新机器人完整部署引导](NEW_ROBOT_DEPLOYMENT_GUIDE.md) 建档、导入真实基础标定，再进行启动。本文保留现有配置机制和 No.124 示例。

2026-11-18：已补充空闲重载的统一读写保护及参数来源查询工具；操作方法见 [参数档案与重载](PARAMETER_OPERATIONS.md)。新机器人首次维护准备允许缺失四份抓放参考，缺失项写入部署记录，仍需完成现场标定。

## 配置不是编译进程序的常量

`config/defaults/factory_baseline.yaml` 是共有控制/策略；`robot_models/t170/` 是型号和运动模型；`sites/current_factory/` 是工厂料盘、传送带、导航与视觉参数；`robots/TH-T170C-0124/` 是身份、相机 SN、设备和准备位；`calibration/TH-T170C-0124/` 是头/手眼及料盘抓放结果。`deployments/TH-T170C-0124.yaml` 选择这些真实输入。

默认选择只在 `config/deployment.yaml` 这个数据文件里，C++ 没有固定选择 124。十台机器人可以共享型号/工厂配置，分别维护身份和标定；机床点位由每次调度任务提供，不固定机器人和机床组合。

V2.0 双臂的 3 kg / 5 kg 模型分别保存在 `robot_models/t170c_v2_3kg/` 和 `robot_models/t170c_v2_5kg/`，deployment 的 `robot_model` 选择其一，No.124 选择 3 kg。正式运动后端已切换为 `Ti5_Robotics`，运行目录生成实际 `Robot_Model.yaml`、电机角绑定、工具和手眼 frame 转换，启动时回读核验 SDK 参数。旧模型下的四份料盘任务参考已标记待复核，原记录保留。完整边界及机器人侧验证顺序见 [双臂机型配置](ARM_MODELS.md) 和 [SDK 接入说明](ARM_SDK_MIGRATION.md)。

合成器目前要求各字段只有一个归属。重复字段会报错，不存在随意“后面的覆盖前面的”。修改参数要改对应来源文件；来源记录见生成的 `deployment_resolved.json`。任意单机参数覆盖白名单属于后续工作。

## 运行目录与源码分开

启动时把分层参数合成为运行目录的 `config/move_box_params.yaml`。SDK、相机、手眼和视觉配置复制到运行目录的统一路径，标定 tasks 链接到选定机器人的可写数据包，模型和代码仍引用源码。旧根目录的大杂烩 YAML 和资源目录已删除；tests/fixtures/no124_factory_baseline.yaml 仅为固定回归输入，不能拿它调现场参数。

在已具备机器人依赖的环境构建：

```bash
cmake -S . -B build_robot
cmake --build build_robot -j4
```

明确选择机器人并生成一个不存在的、源码树外的新运行目录。以下会初始化硬件，production 还会进入调度，必须在现场按当前工作要求启动：

```bash
./deploy/run_deployment.sh config/deployments/TH-T170C-0124.yaml /absolute/path/to/new-runtime --production
```

也可以直接用 `./deploy/run_production.sh` 或 `./deploy/run_maintenance.sh`。源码目录启动时，它们读取 config/deployment.yaml 自动生成新会话目录，默认放在 `$XDG_STATE_HOME/t170`，未设置则为 `$HOME/.local/state/t170`；`T170_STATE_DIR` 可指定。启动输出会打印 `T170_RUNTIME_DIR`。生产自动收任务，维护等待人工命令；二者共用同一程序/设备，不能同时启动。

只生成配置、不访问硬件：

```bash
python3 tools/resolve_deployment.py config/deployments/TH-T170C-0124.yaml /absolute/path/to/new-runtime
./build_robot/t170_config_dump /absolute/path/to/new-runtime/config/move_box_params.yaml /absolute/path/to/new-runtime/effective_config.yaml
```

预检也使用同一个已生成目录：

```bash
T170_ROOT=/absolute/path/to/new-runtime python3 tools/servo_preflight.py
```

已生成目录重启：`T170_RUNTIME_DIR=/absolute/path/to/existing-runtime ./deploy/run_maintenance.sh`。要读取源文件新参数，应新生成会话；改来源文件不会自动改旧运行副本。参数首版优先重新生成/重启生效，避免误以为编辑旧副本能影响下一次启动。

## 网页本轮保持不变

网页与程序必须指向同一运行目录。新增启动包装：

```bash
T170_RUNTIME_DIR=/absolute/path/from/startup ./deploy/run_web.sh
```

只启动原 Web 服务，不自动发运动命令。生成目录临时包含旧 Web 所需的板参数/模型/头外参路径别名，清单为 `web_compat_paths.json`；源码中不保留旧算法转发目录。网页静态页面和服务实现本轮未改。Web 内部启动动作仍有原来的权限/模式约束；不要另启第二份机器人进程。

## 改动与验证边界

普通有效参数由真实 C++ loader 与固定基线对比。逐行偏置及最近排额外 Z 补偿已按本轮授权退役，因此“加载值一致”不代表所有旧回退数值仍被应用。最近排抓取高度需实机确认。

完整配置快照可复核来源和磁盘模型；不证明 SDK 内部已使用对应 DH，也不证明手眼真实精度。安装、打包、现场数据保留/升级回滚流程仍待发布阶段完成。
