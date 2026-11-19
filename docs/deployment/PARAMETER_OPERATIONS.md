# 参数档案、调参与重载

更新：2026-11-18。当前源码部署配置，无设备访问的查询工具为 `tools/parameter_index.py`。

## 调参入口

外置参数不是单一文件，而是有明确归属的档案。单机档案由 `config/robots/<robot_id>/`、`config/calibration/<robot_id>/` 与选择它们的 `config/deployments/<robot_id>.yaml` 共同构成。

| 常改内容 | 正式来源 |
|---|---|
| PD、延迟、制动、伺服门限、公共策略 | `config/defaults/factory_baseline.yaml` |
| 料盘高度、下降速度、导航、行姿态 | `config/sites/current_factory/tray_and_navigation.yaml` |
| 两条传送带的姿态、抓放偏移、窗口和速度 | `config/sites/current_factory/workcell_profiles.yaml` |
| 视觉权重、类别、推理尺寸和阈值 | `config/sites/current_factory/vision/pose_params.yaml` |
| 料盘、码和传送带几何定义 | `config/sites/current_factory/vision/` |
| 共用运动与相机型号规格 | `config/robot_models/t170/`；旧 `Robot_Arm_Model.yaml` 不再参与运行 |
| 臂长、DH 和模型关节限位 | `config/robot_models/t170c_v2_3kg/` 或 `t170c_v2_5kg/` |
| 单机身份、底盘地址 | `config/robots/<robot_id>/identity.yaml` |
| 相机序列号与设备绑定 | 单机档案中的 `cameras/`、`hardware/` |
| 电机方向、零位偏移、CAN ID | 单机档案中的 `arm_binding.yaml` |
| 实际 TCP、准备关节位、参考像素初值 | 单机档案中的 `arm_tools.yaml`、`taught_poses.yaml`、`reference_pixels.yaml` |
| 真实手眼、头外参、料盘抓放参考 | `config/calibration/<robot_id>/` |

不知道某个合成参数在哪里时，在仓库根目录查询：

```bash
python3 tools/parameter_index.py visual_servo.kp
python3 tools/parameter_index.py descend_vel
python3 tools/parameter_index.py --deployment config/deployments/TH-T170C-0124.yaml
```

第一条列出 kp 的有效值及源文件；第二条模糊匹配料盘和两条传送带下降速度；不提供查询词则列出全部已合成控制参数。工具复用正式部署合成和归属校验，临时目录用完即删，不修改来源参数/标定，不加载 SDK。

该索引覆盖 `move_box_params.yaml` 的合成字段；SDK、相机 SN、手眼和视觉资源仍通过 deployment 的 `files`、机型及单机档案选择。多个文件不能任意定义同一个合成参数键，重复会被拒绝。

## 来源、运行副本与内存

```text
正式来源 YAML → 生成运行目录 → 运行副本 move_box_params.yaml → C++ 内存配置
```

原来的大文件仍以运行合成文件存在，便于统一查阅；它由源参数生成。推荐长期调参流程为：查询归属 → 修改来源 → 生成新运行目录 → 重启维护模式验证。运行副本的临时修改不会自动回写源档案；`reload_config` 也不会重新合成源文件。

`deployment_resolved.json` 保存参数来源、输入摘要、机型和缺失标定项；`effective_config.yaml` 是启动校验时导出的 C++ 解析快照。已有 `effective_config.yaml` 不会因后来的热重载自动变成最新值，重新导出或查询内存状态时应说明时间。

## 当前重载保护

应用动作入口持硬件事务锁；重载先尝试取得同一把锁，再取得统一配置锁。标定也遵循硬件 → 配置的顺序。底盘重连、持件阈值和状态查询等后台读配置使用短暂配置锁；观察线程使用会话复制的配置及算法选择，不逐帧读可变全局值。

| 情况 | 返回 |
|---|---|
| 动作/自动调度持有事务锁 | `busy`，不排队等待，不替换参数 |
| 标定会话激活 | `calibration_active` |
| 服务正在退出 | `stopping` |
| 候选 YAML 无效 | `invalid_config`，保留完整旧对象 |
| 身份、底盘地址、相机流/所有权、Arm 实例轨迹限额改变 | `restart_required` |
| 已加载 SDK/手眼/相机/视觉资源文件改变 | `restart_required` |
| 空闲、候选有效且不改变启动绑定 | `reloaded`，在保护内整体替换 |

这里的“空闲”是没有动作、调度或标定占用。自动调度在等待任务期间也持有事务锁，因此要先退出调度并等待返回，再重载。停止请求不等待这两把锁，不会因 YAML 解析被阻塞。

底层模型、设备和 Python 引擎不会因替换一个 C++ 参数对象重新创建；因此对这些已加载对象的配置明确要求重启。重载成功也不自动修改旧运行目录所链接的源代码或外部算法库。

## 新机器人建档与切换

以另一个实际存在的机器人 ID 为例，按以下顺序准备。不得只改机器人名称就把 No.124 的现场结果当作新机验证结果。

1. 建立 `config/robots/<new_id>/`。可以复制 No.124 目录作为字段模板，逐项填写新机身份、底盘地址、相机 SN、设备绑定、工具及经过确认的准备位。参考像素只作为初值。
2. 建立 `config/calibration/<new_id>/`。填写该机头外参、手眼等实际数据；允许使用的型号初值需明确确认并现场复核。建立空的 `tasks/grasp/`、`tasks/placement/`，不复制另一台机器的已验证抓放参考。
3. 复制一份 deployment 清单，修改 `robot_id`、各单机/标定路径和 `calibration_directory`；`robot_model` 选 `t170c_v2_3kg` 或 `t170c_v2_5kg`。共享场景层可以复用，调度仍负责动态分配机床。
4. 运行配置生成及 C++ 校验，先检查来源、身份、型号和缺失标定记录。
5. 先进行 SDK 纯计算检查，再启动维护模式，核验反馈与短动作，完成该机双手抓/放参考。
6. 完成现场验证后进入生产模式。需要默认切换时修改 `config/deployment.yaml` 的 `manifest`；不能在进程运行中切换机器人身份。

已有完整新机档案后，显式选择它的命令如下，将 `<new_id>` 替换为真实 ID，运行目录必须不存在且位于源码树外：

```bash
python3 tools/parameter_index.py --deployment config/deployments/<new_id>.yaml
python3 tools/resolve_deployment.py config/deployments/<new_id>.yaml /absolute/new-runtime
./build_robot/t170_config_dump /absolute/new-runtime/config/move_box_params.yaml /absolute/new-runtime/effective_config.yaml
```

启动时使用另一个新目录：

```bash
./deploy/run_deployment.sh config/deployments/<new_id>.yaml /absolute/another-new-runtime --maintenance
```

生成器现在允许四份抓放任务标定缺失，并在 `missing_task_calibrations` 列出；仍要求选定的标定数据包目录及其他必需资源存在。允许进入维护准备不等于允许未校准的自动作业。

## 封装边界

外置表示运行时从文件读取，不等于这些 YAML 会被藏进二进制。正式安装包应保留型号/场景默认参数、独立单机档案和可写标定/日志区；核心 C++ 算法编译为程序或库，Python 算法可按独立包安装，权重作为资源管理。

目前已实现参数分层和源码运行目录，尚未完成上述独立安装包。少量场景数值及夹爪初始化值仍写在代码里，需继续外置；不能宣称首版所有部署差异都已成为 YAML。
