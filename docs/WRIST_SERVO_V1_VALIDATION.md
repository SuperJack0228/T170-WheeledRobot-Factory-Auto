# 手相机视觉伺服 V1：改动、双手验证与日志

本文件描述本轮实际实现。原有 README、旧手相机方案和 `hand_shadow.csv` 不是本版行为的依据。

## 1. 实现范围

- 同时使用 u/v，取消“下降 35%/70% 各修一次”的逻辑。
- 支持高度相关的期望像素及 2×2 图像雅可比；支持几何估计与实测表两种明确模式。
- RGB 帧记录帧号、采集时刻、时钟来源和不确定度；保存实测关节/FK 历史，用采集时状态建立局部目标，再以当前位置计算剩余修正。
- P 加实际速度 D 阻尼，输出平面速度；I 未启用，避免把错误映射和延迟积累成持续运动。
- 提前制动、限速度/加速度，根据 XY 误差和剩余高度调节下降。实现为保守的制动距离近似，不是完整 MPC，也没有宣称硬实时或无条件稳定性证明。
- 视觉线程只提交观测；每臂运动执行者独占下发，局部 6D 有限差分 DLS 保持姿态；只使用原 SDK 的 FK 和关节限位检查，不改变 SDK 二进制。
- 修改后的目标需要通过实际残差、速度、多个新帧确认后，才能进入最后短行程和夹爪操作。持续视觉/求解故障返回失败。
- 双手使用公平交替推理，取得执行机会后才取最新帧；新伺服开启时关闭争抢 pipeline 的旧网页实时预览线程。
- 保留原有头相机粗定位、调度和成熟的接近/收手流程。接近轨迹在 B 点按 SDK 规划自然减速，随后交接到局部下降；没有插入固定的停拍等待。不能把它描述成整段始终不减速。

主要文件：`include/servo/`、`src/servo/`、`src/hand_shadow.cpp`、`src/function.cpp`。配置位于 `config/move_box_params.yaml` 的 `visual_servo`。

## 2. 当前参数中的已知假设

默认 `mode: geometry` 是使用现有手眼、相机内参与水平特征面的初始估计，不是已完成自动标定。没有伪造任何实测矩阵。

四个 profile 分别为 left/right grasp/place。初始参考像素继承 cameras 中的旧版最终抓准像素；放置参考尚需单独验证。需要动作专用参考时，在对应 profile 添加 `reference_px` 显式覆盖；快照记录最终采用的值。`reference_height_m: 0` 表示这些像素对应最终 TCP 高度。

`feature_z_offset_m` 是可见工件顶面/孔口的 Z 减去最终 TCP Z。当前 0 是初始假设，不能理解为“已经知道顶面与 TCP 一样高”。它影响几何换算与期望像素的高度关系。日志中的 depth、A、reference_px 与误差可用于检查这一假设。

实测表的 `height_m` 定义为“当前 TCP Z 减最终 TCP Z”，单位 m；矩阵为 d(pixel)/d(base XY)，单位 px/m。表外高度、奇异矩阵或非法深度明确拒绝，不用固定 8cm 替代，也不把近距离硬夹成 4cm。

初始配置包括：P=1.5/s、D=0.2、XY 最大速度 20mm/s、模型内平面容差 2mm、名义内环 10ms。它们是首次验证参数，不代表 T170 已实现这些物理指标。实际周期由硬件读取、SDK 求解与通信决定，日志会记录。

最后可靠可见区默认设在最终高度以上约 8mm，且需要多个新帧确认和较低的横向速度。确认后保持目标 XY，限速完成短 Z 行程，允许这一段出现预期遮挡；长度应按现场可见性复核。此前视觉失效会暂停并最终失败，不允许直接盲降到底。

本版只实现部署 profile 的加载与校验，尚未实现自动运动辨识向导和接近姿态族的自动路由。实测表需要用于其标定时的接近姿态和有效范围，不能任意外推。拿到首轮数据后可填入实测表；无需逐孔加偏置。

## 3. 哪些动作启用

- 双手料盘抓取：本次首测重点。
- 料盘放置与传送带抓取：复用同一后端；对应高度、姿态和目标特征需要分别验收，不能由料盘抓取通过推定全部通过。
- 传送带放置：`belt_place_enabled: false`，默认沿用既有开环放置。代码目前没有已确认的传送带放置对准特征，不能强行把该动作当成寻找料盘空孔。启用之前需要定义可观测特征和对应 profile。
- 旧 `vision_grasp_track` 已禁用，避免 RGB 的 0.8m 占位三维点进入真实运动目标。`tray_servo_test` 是独立运动测试，不是新手相机闭环的验收入口。

新闭环开关为 `cameras.hand_descent_correct: true`；旧 `head_grasp.use_hand_camera` 应保持 false。同时启用会在加载配置时报错。

## 4. 同步到机器人前

本次修改位于本地 `T170-WheeledRobot-Factory-Auto-20261028`，分支 `codex/wrist-servo-v1`。原来的 main 目录未覆盖。代码不自动推送；确认带上新 `include/servo`、`src/servo`、测试及工具文件，不能只复制修改过的旧 cpp。

在本地审查后提交并推送工作分支；机器人检出同一提交。无需强制覆盖远端分支。例如在本次修改目录：

```bash
git status --short
git diff --check
git add CMakeLists.txt .gitignore config/move_box_params.yaml include src orchestrator/cpp tools tests docs/WRIST_SERVO_V1_VALIDATION.md web_debug/server.py
git commit -m "Implement continuous dual-arm wrist visual servo"
git push -u origin codex/wrist-servo-v1
```

模型 `feeding_cylindrical_parts_alg/models/best1027.pt` 仍未包含在 Git 中。机器人若已有对应权重，应保留并核对；新部署目录需要另行提供正确版本。程序不会静默换另一份模型。

## 5. 机器人编译与只读预检

以下命令在机器人的实际项目根目录执行。Python 路径使用 CMake 中现有的环境；如果现场实际路径不同，使用对应环境。

```bash
/home/ti5robot/anaconda3/envs/human_interaction_env/bin/python tools/servo_preflight.py > servo_preflight.txt 2>&1
cat servo_preflight.txt
```

必须查看预检退出状态与输出；它检查资源和配置，不连接硬件，也不证明设备正常或物理精度达标。它会输出参数、手眼、SDK 和模型的 SHA256。

```bash
cmake -S . -B build_robot -DCMAKE_BUILD_TYPE=Release
cmake --build build_robot --target t170c_debug -j2
```

编译成功后，启动方式保持不变：

```bash
./build_robot/t170c_debug 2>&1 | tee startup.log
```

不能直接运行旧二进制来验证新源码。参数快照包含构建时的 Git revision 与编译时间，dirty 表示构建时存在未提交修改。

## 6. 直接双手验证

程序正常准备好后，在第二个终端执行：

```bash
python3 tools/debug_client.py servo_dual_once
```

对应 `vision_servo_dual_once`。该入口沿用既有料盘准备、识别与抓取流程，可能包含底盘到料盘工位及腰头准备；不是单纯的手臂点动命令。

它要求两侧都有有效目标，并满足原有列间距/避让条件；不会为完成测试强行解除限制，也不会自动降级为单手。只进行一次双手尝试，不自动补抓；抓后按原流程收手，保持夹爪，不自动转运到下一工位。

停止命令仍为：

```bash
python3 tools/debug_client.py abort
```

若双侧目标或约束不满足，会返回失败并打印原因。`-30` 通常为启动/配置前置条件，`-31` 为局部闭环失败，`-32` 为执行异常，`-34` 为双手单次验证未完成，`-35` 表示另一侧在夹爪操作前失败；具体以日志中的 reason 为准。

## 7. 专门日志

新文件位于：

```text
logs/visual_servo/
  servo_<启动时间微秒>_<进程号>.csv
  servo_<启动时间微秒>_<进程号>.csv.session_<编号>.params.yaml
```

hand=0 左手，hand=1 右手；session 按每臂每次动作分配，不能假设左右永远是 1/2。两个手的轨迹、观测与夹爪结果由 session 关联。

每次 session 保存实际采用的控制参数和四组 profile 快照。控制/观测日志由后台线程写盘，运动线程不做同步 CSV 写入。队列有界，记录 dropped_rows；日志写入故障会进入失败处理，控制台仍有原因。

重点字段：

| 字段 | 含义 |
|---|---|
| task_tag、target_class、map_mode | 动作、检测类别、实际映射模式 |
| frame、capture_s、completed_s、timestamp_source | 图像身份与时序；source=1系统、2全局、3带不确定度的硬件时钟估计 |
| clock_uncertainty_ms | 包含相机时钟与状态读取预算，不等于实测机械延迟 |
| fx/fy/cx/cy、u/v、ref_u/ref_v、du/dv | 原图坐标约定、误差与高度相关参考 |
| depth_m、a00..a11、capture_dx/dy | 几何估计深度、图像雅可比及采集时修正量 |
| goal、actual、remaining | 目标估计、实测 FK 和补偿后的模型误差 |
| requested_v、command_v、applied_scale | 控制器请求与后端实际采用的参考，方便识别限幅 |
| phase、descent_scale、confirmed_frames | 下降状态和真正的新图确认次数 |
| dt/read/solve_ms | 采样到的周期、关节读取与局部求解开销 |
| solver_reason、solver_xyz、solver_rotation | 求解结果和残差 |
| actual_rotation_error、reference_lead、joint_lead | 实际姿态偏差与参考跟随差 |
| actual_q0..6、command_q0..6 | 实际与指令关节角，单位 rad |

`observation` 行对应该帧的采集时状态；`control/end` 行对应当前实测状态，携带最后被控制器接受的观测。COMMIT 以后停止索取新的视觉更新，此时 observation_age 增长是预期行为，短行程仍受长度、时间和执行状态约束。

`gripper_check` 会记录夹爪位置、力矩及 `empty_grasp` / `holding_candidate` / `unknown_gripper_feedback`。holding_candidate 是现有夹爪反馈下的持件候选，不是独立的抓牢验证；仍需结合实际抬起和现场结果。

## 8. 汇总与反馈

```bash
python3 tools/analyze_visual_servo.py logs/visual_servo --json servo_summary.json
```

该工具只需要 Python 标准库。输出各侧有效帧数、观测到采用的延迟、采样周期与求解开销、拒绝原因、最终模型残差、夹爪检查和结束状态。它不会把模型残差当成真实物理精度。

反馈时保留同一次运行的：

1. `startup.log`。
2. 对应新 CSV 和同前缀所有 `.params.yaml`。
3. `servo_preflight.txt`、`servo_summary.json`。
4. 实际现象说明：哪只手、哪个目标、修正方向、是否往返、到最后是否偏、是否夹住/滑落。若有现场视频，可用来对应日志，但不是运行工具的前置条件。

不要用旧 `hand_shadow.csv` 代替新日志，也不要只截最后几行：需要完整看到首次误差、修正、制动和末段动作。

## 9. 本地检查与边界

可在无机器人 SDK 的环境运行控制与运行层模拟回归：

```bash
cmake -S . -B build/servo-tests -DT170_SERVO_TESTS_ONLY=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build/servo-tests -j2
ctest --test-dir build/servo-tests --output-on-failure
```

这些目标不连接 CAN、相机，不调用 T170 SDK。包括二维映射、高度参考、倾斜相机、延迟补偿、状态插值、重复帧/异常拒绝、制动、视觉丢失、多个新帧确认、局部姿态求解，以及带延迟的双手控制与运行层集成。运行层集成使用明确的合成 SDK/电机适配器，不能证明真实 SDK 的周期和跟随性能。

另有 `tests/visual_servo_config_test.cpp` 检查实际 YAML、参数快照往返与非法输入，需链接 yaml-cpp。修改过的可移植 C++ 源可在 Mac 做语法检查；完整硬件目标依赖 Linux ARM 二进制和现场 `/opt/ti5-*`，必须在机器人上完成编译链接和真实双手验收。

本版尚未实现：自动标定向导、受限积分、完整 jerk 约束生成器、全工作空间碰撞规划、独立的抬起后视觉抓牢验证。现有双臂避让与新增局部间距门禁继续生效；不能把局部间距检查当作完整碰撞检测。
