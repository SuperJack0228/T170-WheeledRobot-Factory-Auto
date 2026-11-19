# T170 同环境新机快速部署与调参

适用：新机与 No.124 使用相同运行环境、工厂场景和主要参数，只替换型号、编号、相机与标定等单机信息。继续使用现有轮询与推理锁。

## 1. 新机建档：只改编号和 3/5 kg 选择

在仓库根目录、原有可运行的 Python 环境中执行。下面以 `TH-T170C-0125` 为例，换成真实编号；3 kg 用 `t170c_v2_3kg`，5 kg 用 `t170c_v2_5kg`。

```bash
export ROBOT_ID="TH-T170C-0125"
export ROBOT_MODEL="t170c_v2_3kg"
python - <<'PY'
import os, shutil
from pathlib import Path
import yaml

root = Path.cwd()
robot = os.environ['ROBOT_ID']
model = os.environ['ROBOT_MODEL']
assert robot != 'TH-T170C-0124'
assert model in ('t170c_v2_3kg', 't170c_v2_5kg')
robot_dir = root / 'config/robots' / robot
calib_dir = root / 'config/calibration' / robot
profile = root / 'config/deployments' / (robot + '.yaml')
assert not robot_dir.exists() and not calib_dir.exists() and not profile.exists(), '档案已存在，直接编辑即可，不要重复建档'
shutil.copytree(root / 'config/robots/TH-T170C-0124', robot_dir)
for folder in ('head', 'hand_eye'):
    shutil.copytree(root / 'config/calibration/TH-T170C-0124' / folder, calib_dir / folder)
for folder in ('tasks/grasp', 'tasks/placement'):
    (calib_dir / folder).mkdir(parents=True, exist_ok=True)
identity = yaml.safe_load((robot_dir / 'identity.yaml').read_text())
identity['chassis']['sequence_number'] = robot
(robot_dir / 'identity.yaml').write_text(yaml.safe_dump(identity, sort_keys=False))
text = (root / 'config/deployments/TH-T170C-0124.yaml').read_text().replace('TH-T170C-0124', robot)
doc = yaml.safe_load(text)
doc['robot_model'] = model
profile.write_text(yaml.safe_dump(doc, sort_keys=False))
(root / 'config/deployment.yaml').write_text(yaml.safe_dump({'manifest': 'deployments/' + robot + '.yaml'}))
print('已建立并选中:', profile)
PY
```

这一步复用 No.124 的参数结构与初值，**没有复制四份抓放任务标定**。下面把新机标定覆盖进去。场景参数继续共用 `current_factory`，不另建场景、不重装环境。

以后修改或切换型号，直接编辑：

```text
config/deployments/TH-T170C-0125.yaml
```

其中：

```yaml
robot_id: TH-T170C-0125
robot_model: t170c_v2_3kg   # 5 kg 改为 t170c_v2_5kg
```

机器人编号同时保存在 `config/robots/TH-T170C-0125/identity.yaml` 的 `chassis.sequence_number`，与 `robot_id` 一致；上面的建档脚本已一起填好。该文件里的 `chassis.host` 是机器人自身底盘 IP，不同才改。

默认选择入口是 `config/deployment.yaml`，脚本也已切换为新机：

```yaml
manifest: deployments/TH-T170C-0125.yaml
```

## 2. 导入标定、替换相机 SN

直接用新机文件覆盖以下位置；不需要修改 C++ 路径：

| 文件内容 | 放置位置（示例编号 0125） |
|---|---|
| 头相机 → 头部电机末端的标定矩阵 | `config/calibration/TH-T170C-0125/head/extrinsics.yaml` |
| 左手相机 → 左手法兰 | `config/calibration/TH-T170C-0125/hand_eye/left.yaml` |
| 右手相机 → 右手法兰 | `config/calibration/TH-T170C-0125/hand_eye/right.yaml` |
| 头相机 → 机器人基座的静态备用矩阵 | `config/calibration/TH-T170C-0125/hand_eye/head_fallback.yaml` |

正常头相机定位优先用 `head/extrinsics.yaml` 做动态头变换；如果拿到的是直接到基座的矩阵，应放备用文件，不能把它当成“到头部末端”的矩阵。沿用现有标定文件格式，矩阵平移单位为 mm。

手眼如果已经按新 SDK 法兰标定，把单机 `arm_binding.yaml` 的 `hand_eye_coordinates` 改成 `sdk_flange`；旧法兰约定则保持 `legacy_t7_flange`，生成器会转换。

相机换了，只改：

```text
config/robots/TH-T170C-0125/cameras/realsense_cameras.yaml
```

填好 `head.serial`、`left_hand.serial`、`right_hand.serial`（位于 `realsense` 下）。工具、准备位和电机约定相同就沿用；不同才改同目录的 `arm_tools.yaml`、`taught_poses.yaml`、`arm_binding.yaml`。

## 3. 安装 SDK、编译并启动机器人服务

新机首次安装 SDK（沿用 No.124 已配置的公司 APT 源）：

```bash
sudo apt update
sudo apt install ti5-device-sdk ti5-jindi-faactory-robot-client
```

`ti5-device-sdk` 的交付说明列出 CAN、串口、电机/夹爪编解码和仙工底盘包为依赖，由 APT 一并处理。调度客户端按当前工程/交接资料名称写为 `ti5-jindi-faactory-robot-client`（末尾是 `client`，`faactory` 保留原拼写）；私有源的实际包名与发布版本由公司提供，当前电脑未连接该源核验。若提示找不到包，用 `apt-cache search ti5` 查看该机源中的实际名称。

已经安装、只更新调度库时使用：

```bash
sudo apt update
sudo apt install --only-upgrade ti5-jindi-faactory-robot-client
```

首次安装不要加 `--only-upgrade`，它只升级已安装的包。`apt update` 是更新软件索引，不是更新 GitHub 代码。[APT 官方说明](https://manpages.debian.org/bookworm/apt/apt-get.8.en.html)。升级 SDK 前停止机器人服务，升级后重新编译并启动。

本项目的新运动学库 `libti5_robot.so` 已在 `dependencies/sdk/arm/` 随仓库提供，由 CMake 选择对应架构；腰腿 SDK 源码也已入库，不需要猜测另一个 APT 包名安装它们。

代码更新后编译一次；只改 YAML 不需要重新编译：

```bash
cmake -S . -B build_robot
cmake --build build_robot -j4
```

在仓库根目录，使用原来的视觉环境，启动维护服务：

```bash
export T170_PYTHON="$(command -v python)"
unset T170_ROOT T170_CONFIG_DIR T170_RUNTIME_DIR T170_DEPLOYMENT
./deploy/run_maintenance.sh 2>&1 | tee startup.log
```

脚本会自动读取 `config/deployment.yaml`，合成新运行目录并启动 `t170_factory`。维护模式初始化设备，等待网页/人工指令；不会自动接调度任务。

启动时会打印类似：

```text
[deployment] T170_RUNTIME_DIR=/home/ti5robot/.local/state/t170/session.XXXXXXXX/runtime
```

网页启动脚本会向主程序自动查询这个目录，不需要复制。新仓库使用部署启动脚本，不再在源码根目录直接运行旧的 `t170c_debug`。

## 4. 启动网页调试端

另开终端，进入同一个仓库，使用同一个 Python 环境，直接执行：

```bash
./deploy/run_web.sh
```

浏览器打开网页输出提示的地址，远程访问时将主机换为机器人 IP，保留 token：

```text
http://机器人IP:8080/?token=网页启动时打印的token
```

主程序默认本机命令端口为 8099，网页为 8080，Web 默认监听 0.0.0.0，这些不需要重复输入。脚本只查询已运行主程序的目录、启动网页，不会另外生成一套配置或启动第二份机器人程序。

代码更新后首次需重新编译并重启主程序，使 ping 返回当前目录；之后用上面的一行即可。显式设置 `T170_RUNTIME_DIR=...` 的旧用法仍保留。如果当前终端留有旧的显式运行目录，先取消该设置。自定义主程序命令端口时，例如 `./deploy/run_web.sh --robot-port 8123`。

新机的四份料盘抓/放参考通过网页标定向导分别保存，位置为：

```text
config/calibration/TH-T170C-0125/tasks/grasp/left.yaml
config/calibration/TH-T170C-0125/tasks/grasp/right.yaml
config/calibration/TH-T170C-0125/tasks/placement/left.yaml
config/calibration/TH-T170C-0125/tasks/placement/right.yaml
```

向导操作见 [料盘抓放标定](../calibration/PLACEMENT_WIZARD.md)。这些是抓放参考，与前面导入的完整手眼文件分别管理。

## 5. 抓取、放置高度偏置在哪里改

**料盘抓放：** `config/sites/current_factory/tray_and_navigation.yaml`

| 操作 | 参数键 | 当前值（m） |
|---|---|---:|
| 料盘左手抓取高度偏置 | `head_grasp.grasp_z_offset_m` | -0.001 |
| 料盘右手抓取高度偏置 | `head_grasp.right_grasp_z_offset_m` | -0.005 |
| 料盘左手放置高度偏置 | `tray2_place.offset.left.z` | -0.008 |
| 料盘右手放置高度偏置 | `tray2_place.offset.right.z` | -0.008 |

单位是米，`0.001 = 1 mm`。料盘这几个偏置加在基座 Z 上，**数值调小就更低，调大就更高**。例如放置由 `-0.008` 改成 `-0.010`，目标降低 2 mm。要改整盘高度基准，则改同文件的 `tray.z_ref_m`；平常微调抓放高度先改上表。

**传送带抓放：** `config/sites/current_factory/workcell_profiles.yaml`

| 操作 | 第一传送带 | 第二传送带 |
|---|---|---|
| 左手抓取 | `conveyor.grasp_offset.left.z` | `conveyor2.grasp_offset.left.z` |
| 右手抓取 | `conveyor.grasp_offset.right.z` | `conveyor2.grasp_offset.right.z` |
| 左手放置 | `conveyor.offset.left.z` | `conveyor2.offset.left.z` |
| 右手放置 | `conveyor.offset.right.z` | `conveyor2.offset.right.z` |

传送带这些偏移沿识别出的码坐标轴，其中 z 沿码的 Z 轴；料盘与传送带的偏置坐标约定不同。另外，自动转运放置仍在 `src/tasks/transfer.cpp` 额外加了固定的基座 Z `-0.005 m`，调整第二传送带放置时要算上这一项。

修改源 YAML 后，**停掉主程序和网页，重新执行第 3、4 步**，新参数就会进入新运行目录。只按“重载”不会重新合成这些源文件。

这两份场景文件在同一仓库内由各机器人档案共享；每台机器人电脑上的独立仓库副本互不自动同步。

首版封装时计划增加一份 `config/quick_params.yaml`，只集中放常改的高度偏置、动作速度和少量操作参数；型号/单机标定仍独立，稳定默认值随版本封装。当前尚未接入这份快捷覆盖文件，现阶段仍按上表修改，不会因为手工新建同名 YAML 就自动生效。

## 6. 接入自动调度运行

维护调试完成后，停止维护主程序；同样从仓库根目录启动：

```bash
unset T170_ROOT T170_CONFIG_DIR T170_RUNTIME_DIR T170_DEPLOYMENT
./deploy/run_production.sh 2>&1 | tee startup.log
```

生产入口使用刚才选中的新机档案，初始化后自动进入调度，会执行当前开机准备流程。需要网页时重新执行第 4 步的一行命令，自动使用本次主程序的运行目录。

以后换机器人，切换 `config/deployment.yaml` 的 manifest；换 3/5 kg，改对应 deployment 的 `robot_model`；换标定，覆盖该机的标定文件。上述改动都通过重启生效。

Home 已改为固定关节返回。对应机型的 Home 和肘部偏好放在 `config/robot_models/t170c_v2_3kg/motion_policy.yaml` 或 `t170c_v2_5kg/motion_policy.yaml`，修改后重新启动生成运行目录；旧 `home_tcp` 不再控制 Home 指令。启动自动只读驱动限位，日志快照为当前会话 `logs/arm_limits.yaml`。详见 [双臂运动策略](ARM_MOTION_POLICY.md)。
