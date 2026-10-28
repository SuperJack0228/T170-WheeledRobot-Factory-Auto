# 手相机伺服第二版：连续交接、单侧 Z 边界与抓取容差

基线：`codex/wrist-servo-v1` 的 `9ff8ab3`。日期：2026-10-29。本次实现针对料盘抓取；传送带放置仍默认不开视觉闭环。以下行为已经实现，离线测试不代表真机验证完成。

## 执行链路

1. 沿用启动阶段已有的相机初始化和双视觉引擎预热。接近过程中不进行手相机推理。
2. SDK 仍规划向引导点 B 的接近轨迹。播放到 B 附近的减速段，若距离、姿态和关节速度满足交接要求，提前交给局部控制器；不要求精确到 B，不再固定等待 B 点停稳。局部 XY/Z 速度按加速度约束连续变化，形成过渡。
3. 提前交接条件不满足时，播放完 SDK 接近轨迹后交接。反馈明显落后时，参考推进调速/保持；持续异常才退出。这里保留异常兜底，不承诺所有姿态和 SDK 轨迹都能无减速通过。
4. 进入局部控制后，实际姿态及目标区域适合观察时才取图推理。第一帧未就绪前仅允许有限下降；没有有效视觉证据不会合爪。
5. 使用拍照时的实际位姿映射图像，扣除拍照后已完成的运动。PD 控制 XY，同时依据“预计还需多少时间对齐”和“距对齐保护高度还有多少行程”协调 Z 速度。
6. 优先在 2 mm 模型误差内精对齐，采用 3 mm 退出迟滞减少反复清零。进入 C 上方 30 mm 区域且已有足够观测证据时，可提前锁定 XY 并连续进入最后接近。
7. 未完成精对齐时，Z 参考最多推进到 C 上方 8 mm。若抓取残差已在 6 mm 内且观测有效，在该区域等待至少 0.35 s 后可按抓取容差锁定，不因几毫米残差无限等待。较大误差、缺失观测或异常仍不放行。放置没有此宽松授权，仍使用严格 XY 容差。
8. 锁定后不再接受新图像改变目标，但继续保持已锁定的 XY。提前锁定不会立即强制整段低速，接近末段时才依据制动距离平滑切到最后接近速度。参考下降到 C 上方 2 mm，速度归零、反馈满足单侧高度及抓取容差并连续稳定 0.10 s 后，才标记完成。
9. 双手均完成后进行一次新鲜位置复核；视觉抓取跳过旧的重复电机到位等待、最终点反复复核和固定 0.15 s 等待。记录合爪开始、结束和反馈。

双手仍由同一推理线程公平轮流处理。夹爪仍使用既有串行接口，未假设其线程安全性；新日志可量化串行合爪耗时。

## Z 方向与约束

本项目基座 Z 向上为正。因此“宁可高、不要低”的数学表达是：**Z ≥ 原始 C 的 Z**，不是数值上的 Z≤C。

- 原始 C 仍是视觉映射中的特征高度基准，手相机不重新估计或修改抓取终点高度。
- 抓取运动参考停在 `C.z + grasp_height_margin_m`，默认上抬 2 mm。
- IK 候选还经过 FK 高度检查；考虑求解允许残差后，仍必须高于原始 C。拒绝下发越过该边界的候选。
- 抓取完成时，反馈高度必须在 `[C.z, C.z + 6 mm]`；不能用对称的 ±Z 容差放行低于 C 的反馈。
- 上述边界属于编码器/运动学模型和指令参考层；物理安装、模型误差及驱动动态仍需真机核对，不构成实际末端绝不触碰料盘的保证。

XY 参考也受到目标与领先包络约束，避免实际误差尚未消除时持续累加参考。`done` 时直接保持最后合法关节参考，不再执行额外 IK 或残留下降步。

## 本次默认参数

均在 `config/move_box_params.yaml` 的 `visual_servo` 下，单位为米、秒、弧度。

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `xy_tolerance_m` | 0.002 | 优先精对齐目标 |
| `grasp_xy_accept_m` | 0.006 | 抓取允许的平面残差范数，不是每轴各6mm |
| `grasp_relax_after_s` | 0.35 | 对齐保护区内，使用抓取容差前的等待时间 |
| `commit_height_m` | 0.008 | 未对齐时的 Z 参考推进边界 |
| `freeze_height_m` | 0.030 | 已精对齐时可提前锁定目标的高度 |
| `grasp_height_margin_m` | 0.002 | 抓取参考相对原始 C 的向上余量 |
| `grasp_z_accept_above_m` | 0.006 | 合爪允许高于原始 C 的范围 |
| `finish_speed_m_s` | 0.008 | 抓取完成时的反馈速度上限 |
| `finish_stable_s` | 0.10 | 完成条件持续时间 |
| `acquire_descent_m` | 0.020 | 首帧未就绪时允许的有限下降行程 |
| `handoff_distance_m` | 0.015 | 接近轨迹提前交接区域 |
| `handoff_speed_m_s` | 0.030 | 提前交接的笛卡尔速度上限，另受关节速度和加速度条件限制 |
| `max_reference_lead_m` | 0.012 | 开始调低参考推进速度的领先量 |
| `hard_reference_lead_m` | 0.025 | 限制参考继续前推的包络 |
| `following_timeout_s` | 0.50 | 跟随/姿态异常持续此时间才失败 |
| `commit_timeout_s` | 5.0 | 锁定后的最后接近和稳定等待总上限 |

下降上限仍为 `head_grasp.descend_vel_m_s=0.03`，最后接近上限仍为 `commit_speed_m_s=0.01`。本次先消除错误推进与重复等待，没有提高速度上限。

旧 `slow_xy_error_m`、`stop_xy_error_m` 已移除；不再按这两个固定误差门限分段缩放下降速度。

## 关节限位的明确边界

远程现场基线已经把 SDK 限位检查硬编码旁路。本次将其改为显式参数：

- `enforce_sdk_joint_limits: false`：延续现场基线行为，不擅自恢复存在模型冲突的检查。核对模型角、配置与真实允许范围后再设为 `true`。
- `max_joint_excursion_rad: 0.70`：每个局部候选关节角相对 SDK 交接时关节角的偏离不超过约 40°；同时保留每步速度、加速度和跟随异常约束。

相对偏离约束用于防止局部冗余关节持续漂移，**不替代真实绝对关节限位或完整碰撞检测**。当前双手间距检查也不代表全连杆碰撞检测已完成。

## 日志与版本定位

仍写入 `logs/visual_servo/servo_*.csv`，保留旧列并新增：

- `reference_floor_z_m`、`nominal_final_z_m`：参考推进边界和原始 C。
- `alignment_time_s`、`available_time_s`：预计对齐时间和当前速度对应的剩余时间。
- `finish_xy_tolerance_m`、`relaxed_grasp`：实际使用的抓取授权容差。
- `handoff`：提前交接或 SDK 终点兜底，并记录交接指令、参考、反馈和传入速度。
- `joint_policy`：SDK 限位是否旁路、相对偏离约束启用情况。
- `commit` 原因区分 `target_frozen_precise` 和 `target_frozen_grasp_tolerance`。
- `settling` / `done`：保持合法参考和完成，完成原因区分精对齐/抓取容差。
- `both_ready`、`gripper_gate`、`gripper_start`、`gripper_end`、原有 `gripper_check`。

每个会话生成 schema 2 参数快照，记录构建完整 Git SHA、dirty 状态、差异摘要和自有源码/配置摘要；构建时自动刷新这些信息。另保存 `*.arm_model_on_disk.yaml`，名称明确表示当时磁盘文件，不声称证明 SDK 内存配置。资源 SHA256 可通过预检工具记录。

外层双手单次命令仍可能返回 `-34`，现在同时打印 `root_right` / `root_left`；未进入伺服的任务级失败可能没有伺服根因，要结合任务输出判断。

## 机器人上的编译与验证

在机器人已有完整 SDK、模型和 Python 环境的工程根目录执行；不要用个人电脑里的空库占位文件覆盖机器人的真实库。

```bash
python tools/servo_preflight.py 2>&1 | tee servo_preflight.log
cmake -S . -B build_robot
cmake --build build_robot -j4
./build_robot/t170c_debug 2>&1 | tee startup.log
```

在程序中继续使用 `servo_dual_once` 做一次双手验证。确认现场保留的相机、像素、手眼参数和实际 SDK 文件；不需要重新调整原始 C 来抵消本次新增的 Z 参考逻辑。

结束后汇总：

```bash
python tools/analyze_visual_servo.py logs/visual_servo --json servo_summary.json
```

反馈时保留 `startup.log`、本次 CSV、对应 `*.params.yaml` / `*.arm_model_on_disk.yaml` 和预检输出。优先看是否提前交接、首次有效观测时间、在哪个高度锁定、最小参考高度、完成时是否零运动指令，以及合爪前后时间。

离线回归：

```bash
cmake -S . -B build_servo_tests -DT170_SERVO_TESTS_ONLY=ON
cmake --build build_servo_tests -j4
ctest --test-dir build_servo_tests --output-on-failure
```

覆盖双手延迟观测与 300 ms 位置跟随、单侧 Z 约束、XY 参考不向目标外持续累积、4 mm 可接受抓取/9 mm 不放行/严格放置不放宽、无视觉超时、配置快照回读及 CSV 格式。检测到 yaml-cpp 时额外构建配置测试。

Mac 无法链接和运行真机 Linux SDK；机器人完整构建、实际交接平顺程度、真实抓取余量和成功率仍待现场验证。
