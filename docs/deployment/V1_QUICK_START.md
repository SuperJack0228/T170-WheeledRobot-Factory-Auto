# T170 V1.0.0 部署与参数操作

版本 1.0.0。程序和持久配置同目录，日志及运行快照独立保存。已交付的 rc.3 使用旧工作区布局；本次源码调整需要重新构建新版运行包后生效。`move_box_params.yaml` 已退出当前运行链路。首次发布和全新机器人部署见 [V1.0.0 发布与首次部署](V1_RELEASE_AND_FIRST_DEPLOYMENT.md)，已有机器人升级见 [运行包升级说明](PACKAGE_UPGRADE.md)，十台机床高度配置见 [信息码与高度绑定](CONVEYOR_HEIGHT_BINDINGS.md)。

## 现场开发迭代

尚在调试的机器人采用源码模式。Mac 修改并推送当前开发分支，测试机器人直接拉取、增量编译并启动，不经过 124 中转，不生成 TGZ，不修改版本号或建立标签。正式验收通过后再统一打包发布。

### No.120 一次性切换到源码模式

停止旧包主程序和网页，在 No.120 已配置 SDK、编译依赖和视觉环境的 Linux 主机执行：

```bash
git clone --branch deployment1117 \
  git@github.com:SuperJack0228/T170-WheeledRobot-Factory-Auto.git "$HOME/T170-dev"
cd "$HOME/T170-dev"
conda activate human_interaction_env
```

将下方变量改为 No.120 当前实际运行包的绝对路径，只迁入本机档案、相机信息、快捷参数和标定；共享代码与理论配置使用新源码中的版本。

```bash
T170_PREVIOUS_PACKAGE=/home/ti5robot/实际运行包目录
cp -a "$T170_PREVIOUS_PACKAGE/config/robots/TH-T170C-0120" config/robots/
cp -a "$T170_PREVIOUS_PACKAGE/config/calibration/TH-T170C-0120" config/calibration/
cp -a "$T170_PREVIOUS_PACKAGE/config/deployments/TH-T170C-0120.yaml" config/deployments/
unset T170_WORKSPACE T170_ROOT T170_RUNTIME_DIR T170_CONFIG_DIR T170_DEPLOYMENT T170_BINARY_ROOT
./deploy/setup_robot.sh TH-T170C-0120 --model 3
```

已有档案会被选择，不重新生成相机 SN 和快捷参数。旧包保留作为回退目录。手眼或模型实际变化时，仍需完成对应参考记录的兼容处理；切换更新方式不自动消除标定指纹失配，当前理论手眼修复见下文。

### 每次更新测试

Mac 提交并推送后，在 No.120 退出主程序和网页，再执行：

```bash
cd "$HOME/T170-dev"
conda activate human_interaction_env
./deploy/update_dev.sh
./deploy/run_maintenance.sh
```

另开终端，在同一源码目录和视觉环境执行 `./deploy/run_web.sh`。源码改动通过增量编译生成 `build_robot/t170_factory`，无需再次建档或迁入旧包。反馈问题时同时提供更新脚本打印的提交号和本次日志，以提交号区分测试批次。

`update_dev.sh` 仅执行当前分支的 `git pull --ff-only`、CMake 配置及增量构建，不自动切换分支、提交、清理参数或发布。Git 检测到本地修改冲突时会停止，不执行强制覆盖。No.120 本机参数继续保存在源码目录的 `config/robots/TH-T170C-0120/`，标定在 `config/calibration/TH-T170C-0120/`；共享配置随代码正常更新。

仅修改 Python、网页或 YAML 时通常无需编译 C++，重启对应服务或执行 `./deploy/apply_config.sh` 即可；不确定改动类型时统一执行开发更新入口。CMake 会复用已有构建结果，只编译受影响的 C++ 部分。

稳定后再更新版本号、冻结提交、建立标签并执行 `packaging/build_release.sh`，将经过验收的运行包交付其他机器人。

## 日常调参

维护网页顶部可选择当前调试的机床组，并显示料盘一、传送带一、传送带二、料盘二的生效站点。切换只保存和应用配置，不立即导航；例如第 2 组的“传送带二 · 抓取”导航至 AP8。选择成功后重启仍保留。动作或标定中切换会被拒绝，维持原组。

五组站点及当前选择统一位于 `config/sites/current_factory/maintenance_workcells.yaml`，原 `quick_params.yaml` 中四个抓放站点已迁出。手动任务联调同样使用当前组；生产任务仍使用调度实际下发的站点。

网页参数页对应当前机器人的 `config/robots/机器人编号/quick_params.yaml`，保存会写入持久文件。程序空闲且改动支持热更新时立即应用；忙碌、标定中或涉及模型/相机资源时，保存为下次启动配置并明确提示。

手工修改任何被选中档案引用的 YAML 后，统一执行：

```bash
./deploy/apply_config.sh
```

未启动主程序时只做完整校验，下次维护或生产启动加载。`--validate-only` 明确只校验。非法配置不替换正在执行的配置；网页保存校验失败会恢复原文件。

每个参数只有一个持久来源。网页中的 94 项常调字段归 `quick_params.yaml`；稳定控制默认值、场景视觉、身份、模型仍归对应文件。同一个键重复定义会报错并指出归属。程序自动合成 `config/effective_params.yaml`，此文件只供读取和日志快照，不手工维护。更改不在活动档案中的 YAML 不影响当前机器人。

源码模式配置工作区是仓库根目录。新版安装模式默认为解压后的运行包根目录，实际生效的持久文件直接放在包内 `config/`；这是真实目录，不是隐藏工作区的链接。日志和运行快照默认为 `~/.local/state/t170`。`T170_WORKSPACE` 仍可显式指定外部配置位置。rc.3 及更早安装的配置默认在 `~/.config/t170-factory/workspace/config/`，升级时需要导入。

## 新机器人

安装包解压后，进入包目录，使用与构建时匹配的 Python 和设备/调度 SDK 环境：

```bash
./deploy/setup_robot.sh TH-T170C-0125 --model 3 \
  --head-serial 实际头相机SN --left-serial 实际左手SN --right-serial 实际右手SN
./deploy/apply_config.sh --validate-only
./deploy/run_maintenance.sh
```

5 kg 使用 `--model 5`；底盘地址不同增加 `--chassis-host 实际IP`。首次建立新档案，不覆盖已有标定；再次运行选择已有档案，不静默重写相机信息。已存在机器人的配置可从备份恢复到持久工作区。

当前建档从模板复制快捷参数及兜底像素。模板中的料盘人工 XYZ 已清零，两套传送带改用统一偏移；旧档案升级时执行一次参数迁移，之后现场调参继续保留。本机四份任务标定另行建立。数值与生效条件见 [偏置清理记录](OFFSET_AUDIT_20261125.md#10-本次清理结果)。

另开终端在同一包目录执行：

```bash
./deploy/run_web.sh
```

新档案默认使用头、左右手相机的理论值，来源在 `config/robot_models/t170c_v2_common/cameras/`。左右手理论矩阵以已验证 No.124 的旧法兰链路为兼容基准，声明 `parent_frame: legacy_t7_flange`，生成运行配置时转换到所选 SDK 法兰；文件中的 `parent_link` 只标识物理安装位置，不代表当前 3/5 kg URDF 的同名腕部坐标基底。标准图像方向为左手原图、右手旋转 180°；本机 `cameras/realsense_cameras.yaml` 的 `image_rotation_deg` 为统一来源。No.124 装反，明确保留左 180°／右 0° 的独立配置。新机器人仍需建立自己的左/右抓取、左/右放置四份图像参考。

### 已部署默认理论值的坐标适配修复

2026-11-25 的 No.120 抓取参考暴露出默认手眼父坐标声明错误。原 `urdf_wrist` 路径在该参考姿态下将光轴算成朝上，参考距离为 −0.595 m，持续触发 `reference_depth_outside_range`。改用旧法兰兼容路径后为 +0.116 m。修复保留理论矩阵数值和图像方向，不复制 No.124 的实测误差。该结果已通过离线几何和 C++ 映射回归，实际抓放精度需要真机复测。

已部署机器只需更新下面两份持久源文件，随后重新启动；运行逻辑未改变，无需为此重新编译 C++：

```text
config/robot_models/t170c_v2_common/cameras/left_nominal.yaml
config/robot_models/t170c_v2_common/cameras/right_nominal.yaml
```

若通过 `upgrade_robot.sh --from-workspace` 导入旧包，旧包的整个 `config/` 会被保留；导入后也要将这两份文件更新为修复版，不能只依赖新包初始模板。

手眼变化会使原任务参考的 `eye_fingerprint` 不匹配。对使用旧默认理论值完成过示教的 No.120，停止服务后，在实际持久配置所在目录执行一次：

```bash
mv config/calibration/TH-T170C-0120/tasks \
   "config/calibration/TH-T170C-0120/tasks_before_eye_frame_$(date +%Y%m%d_%H%M%S)"
mkdir -p config/calibration/TH-T170C-0120/tasks/grasp \
         config/calibration/TH-T170C-0120/tasks/placement
./deploy/run_maintenance.sh
```

随后启动网页，通过标定向导重新建立左/右抓取、左/右放置参考。备份目录保留原始数据供核对；不手工改写指纹。新建机器人无需此迁移。No.124 的档案仍引用自身实测手眼文件，不受这两份默认值修复影响，也不需要执行上述备份命令。

完成维护验证后，停止维护服务：

```bash
./deploy/run_production.sh
```

生产入口听调度分配任务。更换机器人、3/5 kg、相机、手眼或模型后重新启动；无需手工生成临时目录。

## 运行包构建

发布代码冻结并建立匹配版本标签后，在匹配机器人的 Linux 环境执行：

```bash
./packaging/build_release.sh
```

生成目录为 `build_release/packages`，包含 TGZ 和 SHA256。构建脚本检查安装目录中的 Linux 动态库，缺失依赖或仍引用源码目录会停止打包。发布包不包含 C++ 源码、历史日志、旧权重及单机任务标定；Python、网页和在用权重作为运行资源安装。`release.json` 和 `environment.lock.json` 记录版本、构建环境及动态依赖解析结果。

Mac 离线测试不替代 Linux 链接和真机检查。第二台机器人部署使用已验证的二进制包及匹配环境，不需要再编译 C++。

systemd 模板读取 `/etc/t170-factory/runtime.env`，至少指定 `T170_INSTALL_ROOT`；自定义配置位置同时设置 `T170_WORKSPACE`，Python 环境不同设置 `T170_PYTHON`。升级前停止主程序，保留配置备份，再切换版本。`Restart=no` 保留，任务中断不会自动重新启动并 Home。

## 维护机床组站点表

| 组别 | 料盘一 | 传送带一 | 传送带二 | 料盘二 |
|---|---|---|---|---|
| 1 | AP1 | AP2 | AP3 | AP4 |
| 2 | AP6 | AP9 | AP8 | AP10 |
| 3 | AP407 | AP408 | AP409 | AP410 |
| 4 | AP413 | AP414 | AP415 | AP416 |
| 5 | AP418 | AP419 | AP420 | AP421 |

网页维护组选择与本表对应。直接编辑 YAML 时，修改 `selected_group` 或各组站点后执行 `./deploy/apply_config.sh`。二维码身份与机床高度仍由独立的 `conveyor_heights.yaml` 管理，选择维护组不修改二维码或高度绑定。
