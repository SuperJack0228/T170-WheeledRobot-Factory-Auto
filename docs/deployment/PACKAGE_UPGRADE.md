# 运行包升级与配置迁移

rc.3 已交付，使用隐藏的持久工作区。V1.0.0 将默认配置位置改为运行包根目录的实际 `config/`，需在兼容机器人的 Linux 环境构建新包后生效。旧 rc.3 压缩包和标签不变。首次发布和新机完整步骤见 [V1.0.0 发布与首次部署](V1_RELEASE_AND_FIRST_DEPLOYMENT.md)。

## 新版目录

```text
t170-factory-版本-Linux-aarch64/
├── bin/
├── lib/
├── config/                 实际生效的参数和本机标定
│   ├── deployment.yaml    选择当前机器人
│   ├── robots/机器人编号/quick_params.yaml
│   ├── calibration/机器人编号/tasks/
│   ├── sites/current_factory/
│   └── robot_models/
├── deploy/
└── release.json
```

网页保存、手工修改和启动使用同一个 `config/`。首次解压的包包含模板，但未选择 No.124；新机执行建档，已有机器人导入旧配置。日志及运行快照仍在 `~/.local/state/t170/`，不要修改其中临时生成的 `effective_params.yaml`。

## No.120 从 rc.3 迁移

先结束任务和标定，停止机器人主程序和网页。systemd 部署先停止相应服务；终端启动则正常退出。保留旧包，将新版解压到独立目录，进入新版目录并激活原视觉环境。

rc.3 原来使用默认配置位置时，执行：

```bash
conda activate human_interaction_env
unset T170_WORKSPACE T170_ROOT T170_RUNTIME_DIR T170_CONFIG_DIR T170_DEPLOYMENT T170_BINARY_ROOT
./deploy/upgrade_robot.sh --from-workspace "$HOME/.config/t170-factory/workspace"
```

`--from-workspace` 指向包含旧 `config/` 和 `release.json` 的目录，不指向 `config/` 本身。若原来显式设置了外部工作区，填写那个实际目录。

升级脚本执行以下操作：

1. 锁定源、目标配置，备份旧配置到 `~/.local/state/t170/config_backups/时间戳/`。
2. 将旧机器人档案、本机快捷参数、相机 SN、标定等完整复制到新版包的 `config/`，旧目录保持原样。
3. 按原版本执行已有相机方向及机床高度表迁移。rc.3 已采用标准方向时，不再次改变图像方向或清空标定；已有机床高度表不被模板覆盖。
4. 对尚无 `20261125_task_offsets` 迁移记录的机器人，执行已确认的偏置清理：料盘参考改为 −0.623 m、料盘人工 XYZ 清零、两套传送带抓取及放置采用统一偏移；移除旧三维修正字段，修正传送带二的 `belt0` 类型配置。具体数值见下文。
5. 使用新版 C++ 配置加载器校验。失败则恢复目标目录原配置和迁移记录；成功则打印实际配置路径、来源、备份位置和 `task_offsets_migrated`。不启动机器人。

本批还补入 `maintenance_workcells.yaml` 的五组站点表及 manifest 映射，将旧快捷文件中的四个抓放站点迁出。已有机床组表和选择保留，新建表默认第 1 组。右手传送带放置 Y 已更正为 −15 mm；若已完成先前偏置迁移，额外的一次性修正仅修改两套传送带右手放置 Y，不重置其他已调参数。

这次升级会有意修改上述已确认的参数，不能再理解成所有旧参数原样保留。两套传送带抓取左 `[50,120,0]`、右 `[-30,120,0]` mm；放置左 `[-70,-15,0]`、右 `[0,-15,0]` mm。迁移完成后在本机 `parameter_migrations.yaml` 记录一次，后续升级保留新的现场调参，不会重复下移 5 mm 或再次清零。相机 SN、工具、任务标定、速度等其他参数保留。转运额外 −5 mm 已从 C++ 删除，需要使用重新编译的二进制。完整高度联动见 [偏置清理记录](OFFSET_AUDIT_20261125.md#10-本次清理结果)。

在两个终端分别启动：

```bash
./deploy/run_maintenance.sh
```

```bash
./deploy/run_web.sh
```

此时 No.120 常调参数直接位于当前包的 `config/robots/TH-T170C-0120/quick_params.yaml`。手工修改后执行 `./deploy/apply_config.sh`；网页保存写入同一份文件。完成维护验证后，停止维护主程序，再执行 `./deploy/run_production.sh`。

## 后续升级

每次将新版解压到新目录，从新目录导入上一次实际使用的旧包配置：

```bash
./deploy/upgrade_robot.sh --from-workspace /旧运行包的绝对路径
```

不要继续指向已经停用的隐藏工作区，否则会导入旧参数。已有配置的同目录校验及迁移仍可执行 `./deploy/upgrade_robot.sh`，但新版本应使用独立目录，避免直接覆盖旧二进制。

显式外部配置模式仍兼容：`--workspace /目标工作区` 决定写到哪里，`--from-workspace /旧目录` 决定从哪里读取。不指定 `--workspace` 且未设置 `T170_WORKSPACE` 时，目标就是当前运行包。

回退时停止主程序与网页，从保留的旧目录启动原版本；原 rc.3 若使用隐藏工作区，该工作区仍保留。常规升级保留原相机方向和标定；涉及方向变更的旧版迁移需要程序、配置、任务参考配套回退。

systemd 部署还需将 `/etc/t170-factory/runtime.env` 的 `T170_INSTALL_ROOT` 切换到新包目录。如果以前写了指向隐藏工作区的 `T170_WORKSPACE`，删除该项或改成新包目录，否则显式设置仍优先生效。升级脚本不修改系统服务配置。

## 构建和发布

源码修改和检查完成后，确定新的 `packaging/VERSION`、提交和标签，在匹配机器人环境的 Linux 上运行：

```bash
./packaging/build_release.sh
```

输出位于 `build_release/packages/`，包含 TGZ 和 SHA256。新提交、新标签、新包一一对应，不覆盖旧包或移动旧标签。GitHub 保存源码；Release 分发构建包，不负责自动将源码改动变成二进制。Mac 的离线检查不能生成现场使用的 Linux ARM64 二进制。

以后常调参数只需保存、应用，无需重新打包；代码、网页或依赖修复则交付新版本包，导入本机配置后使用。
