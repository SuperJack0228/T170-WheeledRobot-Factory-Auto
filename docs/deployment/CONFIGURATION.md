# 配置来源与统一应用

V1.0.0 的配置链路为“持久来源 → 校验候选 → 主程序空闲提交 → 运行快照”。新机与安装包步骤见 [V1 快速入口](V1_QUICK_START.md)。

## 参数归属

| 内容 | 持久来源 |
|---|---|
| 网页现有的 94 项常调参数 | `config/robots/机器人编号/quick_params.yaml` |
| 稳定控制默认值 | `config/defaults/factory_baseline.yaml` |
| 场景视觉、料盘及传送带结构 | `config/sites/current_factory/` |
| 维护机床组与四个抓放站点 | `config/sites/current_factory/maintenance_workcells.yaml` |
| 身份、相机 SN、工具与角度约定 | `config/robots/机器人编号/` |
| 3 kg 或 5 kg 模型及肘腕策略 | `config/robot_models/t170c_v2_3kg/`、`t170c_v2_5kg/` |
| 新机默认头与手相机理论值 | `config/robot_models/t170c_v2_common/cameras/` |
| 本机四份抓放参考 | `config/calibration/机器人编号/tasks/` |

每个普通参数只在一个来源中定义。常调字段已从原默认/场景文件迁走，不存在两份数值争夺优先级。同一个键重复定义时应用会指出两个来源，不能通过在另一个文件重复添加来覆盖。`quick_params.yaml` 纳入当前 deployment 的 layers，网页与手工编辑使用相同文件。

`config/deployment.yaml` 选择机器人 manifest，manifest 的 `robot_model` 选择 3/5 kg。作业点位由调度分配，快捷文件中的站点为维护/备用值，不将机器人与机床固定绑定。

维护模式的四个抓放站点由机床组表的 `selected_group` 选择，合成为原有 `chassis.tray_station/belt_station/out_station/tray2_station` 供单步技能调用；不要在其他层重复添加这些字段。网页切组通过统一配置事务热更新，只有机器人确认接收后才显示生效。充电、待命等其余常调站点继续位于本机快捷文件中。调度任务仍临时使用任务下发的站点，维护组不绑定生产机床。

## 文件用途

旧 `move_box_params.yaml` 已退出活动代码。程序启动或应用时自动生成 `config/effective_params.yaml`，供 C++ 加载完整配置；工程不要求人工维护它。`effective_config.yaml` 是 C++ 解析后的快照，`deployment_resolved.json` 记录来源和摘要，接受热更新后一起更新。

配置和标定保存在持久工作区。源码模式默认是仓库根目录；新版安装模式默认是运行包根目录，直接编辑包内 `config/`。显式设置 `T170_WORKSPACE` 可以继续使用外部目录。rc.3 及更早版本的默认位置是 `~/.config/t170-factory/workspace`，导入方式见 [升级说明](PACKAGE_UPGRADE.md)。运行会话及日志仍保存在 `~/.local/state/t170`，由 `T170_STATE_DIR` 和 `T170_LOG_DIR` 调整。

当前模板的料盘人工 XYZ 已清零，传送带 XYZ 已统一；实际参数与历史差异见 [偏置清理记录](OFFSET_AUDIT_20261125.md#10-本次清理结果)。本机任务标定的 XY 和像素参考继续保留。`config/robots/机器人编号/parameter_migrations.yaml` 记录已完成的一次性迁移，不属于运行参数层；保留该文件可避免后续升级覆盖现场调参。

## 统一应用命令

```bash
./deploy/apply_config.sh
```

该命令重新读取选中档案引用的 YAML、生成完整候选并调用真实 C++ 配置加载器。主程序空闲且资源允许时热更新；正在动作、标定或改动了缓存资源时保留持久修改，并提示再次应用或下次启动加载。主程序未启动时只校验。明确只校验使用 `--validate-only`。

网页参数页保存走同一链路。非法网页修改会恢复原文件；忙碌或需要重启不是保存失败，回复会区分 `saved` 与 `applied`。手工编辑的非法文件由操作者修正，程序保留上一份合法运行配置。

热点参数在硬件事务锁和统一配置锁保护下提交。相机流、设备连接、模型、手眼及 Python 引擎资源变化需要重启，不能用更新一个参数对象代替重建实例。

## 启动入口

```bash
./deploy/run_maintenance.sh
./deploy/run_web.sh
```

两条命令在不同终端执行。维护结束后停止主程序，再使用 `./deploy/run_production.sh` 接受调度。每次启动自动合成持久配置，不需手工准备会话目录。不要沿用旧终端中指向历史会话的 `T170_RUNTIME_DIR`。
