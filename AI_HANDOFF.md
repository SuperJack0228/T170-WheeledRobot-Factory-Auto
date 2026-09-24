# T170C 交接文档（给下一任 AI）

更新：2026-09-23 傍晚  
权威目录（机器人本机）：`/home/ti5robot/T170Clean`  
用户：`ti5robot`，主机 Ubuntu aarch64（Jetson/Tegra），无 ROS2。  
机器人：钛虎 T170C，双 7 轴 + 腰腿 LowerBody 5 轴 + 头 3 电机 + DEX1 夹爪 + 仙工底盘。

**2026-09-21 及更早的交接正文已过时。** 当时的主问题是料盘左右分列、传送带未接入、底盘未接。那些都已经做完。以本文和 `config/move_box_params.yaml` 为准。yaml 里 `conveyor:` / `conveyor2:` 上方仍有「第一传送带 AP6、第二传送带 AP5」的旧注释，**不要信**。站点以 `chassis:` 和 `vision_grasp_then_belt()` 的日志为准。

---

## 0. 下一任立刻要遵守的硬约束

1. **不要用 ROS2。** 没有 rclcpp / topic。
2. **右臂已解锁：`kLockRightArmMotors = false`**（`src/Ti5_socketcan.cpp`）。旧机右臂曾撞断，开关保留；不要删右臂代码。再锁定只改这一处为 `true`。
3. **右 DEX1 经常扫不到。** 启动允许少一个夹爪；缺侧 `open/grasp` 直接跳过，手臂仍走。不要把「没夹爪」当成启动失败。缺右爪时右手仍可能计一次成功。
4. **YOLO 只给类别，不参与定位。** 料盘孔位 XY 来自 ArUco 盘系 → 基座。码不够禁止回退 YOLO 抓取。传送带抓取的 YOLO 门禁默认关（`grasp_yolo_enable: false`）。
5. **不要把盘心再加进孔位 XY。** `t_avg` 已经在 `xyz_level` 里。再加一次就是双重平移。
6. **C++ 改完必须重编 `build_robot/t170c_debug` 并重启进程。** yaml 数值（含 offset、固定 Z）可 `reload`。新命令、Python 视觉引擎、头/腰运动逻辑不行。不要擅自启动 `t170c_debug`，除非用户要求。
7. **不要 git commit，除非用户明确要求。**
8. 软件 `abort` **不能替代实体急停**。动腰/臂时旁边要有人。
9. 头俯仰现场大约最多 **47°**。`far_pitch_deg: 47`。不要再写 65°。
10. **放置不要再加 J2 锁，也不要加额外的 XY 回退。** 用户试过，结论是越改越糟。放置松爪后只抬约 3 cm，然后一条直线回准备。
11. **不要用每天改 offset 去吞 2–3 cm 的随机高度差。** 见 §3。offset 只改下发目标，改不了跟踪没走完。

坐标系（全工程统一）：**基座 +X 前、+Y 左、+Z 上**。料盘 / 皮带 ArUco 系同样前左上。

---

## 1. 当前进度（2026-09-23）

整圈已经打通，执行起来没有大的流程问题。用户明确的下一步是：

1. **抓取 / 放置精度**（先把误差变小、变稳，再谈微调）。
2. **手臂动作平滑。**
3. **效率**（动作连贯，少停、少分段）。
4. **和机床信号配合**（还没做，不要先发明协议）。

### 已跑通的闭环

`cycle` → `vision_grasp_then_belt()`，一直循环到 abort 或某步失败：

```
AP7 料盘抓毛坯
  → home（站起 + home_tcp，保持合爪）
  → AP5 第一传送带放置（conveyor）
  → belt_grasp（原地，不转腰）
  → AP6 第二传送带放置（conveyor2）
  → belt2_grasp（原地，不转腰）
  → AP9 空孔放置（料盘2，YOLO class 3）
  → 站起 + home_tcp
  → 底盘回 AP7
  → 下一圈
```

底盘是仙工 `GoToStation`，`chassis.host: 192.168.192.5`。曾经出现的 `code=-6` 是导航「control is preempted」，不是流程逻辑错。

### 站点

| 名字 | yaml | 工位 | 参数 |
|------|------|------|------|
| 料盘抓取 | `chassis.tray_station` | **AP7** | `head_grasp` + 料盘 5×5 |
| 第一传送带 | `chassis.belt_station` | **AP5** | `conveyor` |
| 第二传送带 | `chassis.out_station` | **AP6** | `conveyor2` |
| 空盘放置 | `chassis.tray2_station` | **AP9** | `tray2_place`，识别空孔 |

传送带腰高都是 `waist_z: 0.63`。料盘抓取腰高仍是 `ready_z_m: 0.36`，home 腰高 `layer3_home.z: 0.63`。

---

## 2. 各段在做什么

### 2.1 料盘抓取（AP7，`grasp`）

- 只抓 YOLO **class 0**（毛坯）。左右分列用 **ArUco 列号**：1–3 右、4–6 左。盘转 180° 时按列的基座 Y 判断对调，不要改回用 `Y=0` 当盘心。
- 远近用 `from_robot`（1 最远）。近三排 4–6 在 ready 起始 x；远三排 1–3 腰前伸 `far_row_forward_m: 0.18`，头 `far_pitch_deg: 47`，live `free_calib` 后重拍。
- 满排同时抓的列对是 3–6 / 2–5 / 1–4。姿态按离机器人远近，不按 ArUco 行号。`ready1/2/3/6` 只摆这些姿态。
- Z：物体顶面 = `tray_z_ref_now() + part_above_tray_m`。相机顶面与参考差超过 `z_refine_max_m`（3 mm）就用参考。最终 Z = 顶面 + 该手 `grasp_z_offset`。`nearest_row_grasp_z_offset_m` 只叠在 `from_robot=6`。
- 左右 offset **不共用、不取反**。当前左手 z=+0.02、右手 z=+0.04，所以同一参考下右手目标高 2 cm。
- 盘面参考公式仍是 `z_now = z_ref_m − (当前腰z − z_ref_waist_z_m)`，标定锚点是腰 **0.55**（`z_ref_waist_z_m`）。home 腰后来改成 0.63，不要把 `z_ref_m` 无测量地改掉。
- 手相机关：`use_hand_camera: false`。轨迹：`use_bezier_grasp: false`，直线下压。Bezier 代码留着。
- 料盘抓取在笛卡尔检查前会 `wait_arms_motors_stopped`。传送带直线 **不会**。

### 2.2 第一传送带（AP5，`conveyor`）

放置和抓取都先到观察姿态，然后停稳 **1.5 s** 再拍照。底盘刚到之后不再另等 1 s / 0.5 s。抓取点仍停 **0.5 s** 才合爪。传送带直线在读正运动学前会 `wait_arms_motors_stopped`，编码器没停稳不算到位。

- 放置 XYZ = 码原点 + `R_识别 × offset`。RPY 用 `place_tcp`，不是 `tcp`。
- 抓取点 = 码原点 + `R_识别 × grasp_offset`。RPY 用 `grasp_rpy_deg`，不是 tcp 的 RPY。
- 左右手都可以过 Y=0。放置右手先放，抓取左手先抓。先动的那只手夹爪一完成，另一只手立刻过来，先动的手同时抬起撤回，不等整段结束。不预开爪。
- 准备姿态夹爪先收到 `grasp_ready_close_ratio`（现 0.20），避免下探磕皮带边。
- **码平面 Z 已固定，不用相机高度。** `use_fixed_origin_z: true`，`fixed_origin_z_m: -0.29`。这是腰 0.63 m 时卷尺：手臂基座到传送带（码平面）29 cm，Z 向上，传送带在下方。XY 和板朝向仍用二维码。offset 和 hover 仍叠在 −0.29 上。左右手因此共用同一个码平面 Z；若两边 `grasp_offset` 相同，下发的抓取 XYZ 也相同。
- 调试客户端里的 `grasp_z` / `belt_z` 在抓取时是 **左手抓取点**，不是码原点，也不是右手。

只摆手腕、不拍照不夹取：`belt_grasp_rpy`（tcp 的 XYZ + `grasp_rpy_deg`）。准备姿态本身是 `belt_ready`。`belt_place` 是放置末端预览，不是抓取前的准备位。

### 2.3 第二传送带（AP6，`conveyor2`）

流程和第一套相同，参数独立。底盘到 AP6 后同样是准备姿态到位，再停稳 **1.5 s** 拍照。

**仍用相机读出的码平面 Z**（`use_fixed_origin_z: false`）。固定 −0.29 先只开在第一套上，方便对比。用户确认第一套之后，再同样接到第二套。

当前抓取 offset 左右不完全相同：左 `{−0.04, 0.12, 0.05}`，右 `{−0.04, 0.105, 0.05}`。RPY 是 pitch 45°、rz ∓40°，和第一套的 40° / ∓60° 不同。

### 2.4 料盘2 空孔放置（AP9，`tray2` / `tray2_place`）

复用料盘抓取的孔序和腰/头，但 YOLO 要 **class 3 空孔**，不是 class 0。四组 RPY 在 `tray2_place`，**不要复用** `ready1/2/3/6`。调试命令 `tray2ready1/2/3/6`。XYZ offset 在 `tray2_place.offset`，左右手分开，还有 `nearest_row_z_offset_m`。改 `head_grasp` 的 offset 不再带动料盘2。放置是张开夹爪，不是合爪。放完站起，手臂到和第一次抓完一样的 `home_tcp`。

---

## 3. 精度：已经查清的原因（先看这个再改）

用户要的是误差小且稳定，不要求 100% 精准。每天改一个 offset 吃掉随机厘米级偏差，没有意义。

### 3.1 日志里的「到位误差」是什么

下发的笛卡尔目标，对上 **当前编码器关节的正运动学**。两边用同一套模型。它不是卷尺量到的夹爪在世界上的位置。

2026-09-23 起，传送带 `move_one_arm_line_to` 先 `wait_arms_motors_stopped`，停稳后再读 FK。在这之前它是最后一个关节设定点刚发出去就读，所以下降会看到 2–3 cm。料盘抓取本来就会等电机停稳再查。

2026-09-23 16:36 第一传送带抓取（固定 Z 已生效），左右下发 Z 都是 **−0.2493 m**：

| | 下发 Z | 发完瞬间的 FK Z | 还差 |
|--|--------|-----------------|------|
| 左手 | −0.2493 | −0.2236 | 高 25 mm |
| 右手 | −0.2493 | −0.2244 | 高 25 mm |

两手之间只差 0.8 mm。悬停（同一姿态、还没下降）Z 误差约 1 mm。下降短一截、抬升也短一截，方向跟着运动走。这组数是改「停稳再读」之前的。之后传送带会等编码器到位再打印误差。

所以「左右实际高度不一致」若是看这个日志，两手命令相同、瞬间 FK 也几乎相同，共同的问题是下降没走完就被当成到位。

### 3.2 怎么把一次误差拆开

同一条日志、等停稳之后：

| 比较 | 说明 | 该怎么处理 |
|------|------|------------|
| FK(最后下发的 q) 对目标 | 逆解 / 限位把目标投影丢了 | offset 推不过去。要拒掉 FK 离目标太远的 `Line_Trajectory` |
| FK(编码器) 对最后下发的 q | 跟踪没到 | 等编码器；最后 2 cm 放慢。只有重复出现的几毫米才进 offset |
| 编码器跟上了、日志误差很小，零件仍偏 | 腰沉、视觉、工具长度 | 不要改手臂 offset |

### 3.3 已经排除或次要的

- **工具长度** `[0.23, 0, 0]` 沿法兰 X。长度错会在 45° 俯仰下同时出现在 X 和 Z，解释不了「XY 约 1 mm、纯 Z 差 25 mm」。
- **腰下沉 / 连杆变形** 不进手臂基座 FK。日志写 2 mm 但零件没抓到，去看腰和视觉。
- **编码器量化** 可忽略。`grasp_valid.z_min=-0.65` 不会夹到传送带大约 −0.25 的目标。
- **J2/J6 的 yaml 限位只约束求解器，不挡住电机。** 曾看到左手传送带抓取模型 J6 约 −53°，yaml 是 ±40°，手臂已经在公布限位外面。冗余 J2 在抓取直线上不锁，种子不同会留几毫米。
- `Line_Trajectory`（闭源 `Ti5_Arm`）路点 IK 返回 0 就算成功，**不像** `solve_bezier_ik` 那样用 FK 复核 1 mm / 0.5°。限位投影出的「最近姿态」也会报成功。
- 料盘等待里，稳定误差到约 2500 计数（≈3.4°）会当已到位。0.25 m 连杆上 1°≈4 mm，3.4°≈15 mm。
- 第一传送带码平面相机 Z 曾经大约 −0.32，剔除一个 −0.317 的离群后散布约 4 mm。这是改成固定 −0.29 的原因。固定的是码平面，不是最终 TCP。

### 3.4 精度上先做的事（用户还没要求改代码）

1. 传送带直线已经等编码器停稳再读 FK。还没做的是：`Line_Trajectory` 成功但 FK 离目标超过几毫米就当失败。
2. 不要把「平台期还差 3°」当成功。
3. 做完上面两件，只把重复、稳定的几毫米写进 offset。
4. 第一传送带固定 Z 对比满意后，再给 `conveyor2` 打开 `use_fixed_origin_z`。

---

## 4. 平滑和效率（还没改，先知道停在哪）

现在为了认码和松爪，故意插了多段等待，动作是一段一段的：

- 两条传送带都是准备姿态到位后再等 1.5 s 才拍照。
- 抓取点再 0.5 s 才合爪。
- 右手放置回准备后间隔 0.5 s，左手才开始放。
- 传送带双手是依次的，不是同时。
- 放置下降用直线，速度走 `head_grasp` 的接近速度，最后一段没有单独放慢。

提效率时先减这些确定的空等和衔接，不要先改几何。平滑和「等电机真正到位」是一件事的两面：现在发完就报到位，看起来快，高度却是虚的。

---

## 5. 机床信号

未接入。闭环目前只到空盘放回 AP9 再回 AP7。不要在用户给出信号定义之前加 Modbus / IO 占位流程。

---

## 6. 架构与启动

常驻进程是 **C++** `./build_robot/t170c_debug`（CMake 在 `build_robot/`）。

- 监听 **仅** `127.0.0.1:8099`，一行 JSON 一命令。
- `tools/debug_client.py` 和 `web_debug/` 只是遥控器。流程在 `orchestrator/cpp/robot_runtime.cpp`。
- Python 通过 pybind11 嵌在 C++ 里做视觉。料盘 `board_yf100_aruco/`（5×5），传送带 `board_belt_aruco/`（6×6），两套互不覆盖。
- 底盘已接入，不是 `chassis=disabled`。

```bash
cd ~/T170Clean
sudo -v
./build_robot/t170c_debug 2>&1 | tee startup.log
```

```bash
cmake --build /home/ti5robot/T170Clean/build_robot -j2 --target t170c_debug
python3 tools/debug_client.py reload    # 只重载 yaml 数值
python3 tools/debug_client.py cycle
python3 tools/debug_client.py abort
```

Conda 视觉环境：`/home/ti5robot/anaconda3/envs/human_interaction_env`。直接跑二进制即可。`startup.log` 由 tee 写出，有时缓冲，查高度以进程终端和该文件里带时间的那一段为准。截图在 `picture_debug/head/`。

---

## 7. 调试命令

| CLI | JSON `cmd` | 作用 |
|-----|------------|------|
| ping / status / reload / abort | 同名或 snapshot / reload_config | 探活、TCP、重载 yaml、软件中止 |
| home | home | 头标定角 → `home_tcp` → 腰到 `layer3_home` |
| ready1/2/3/6 | grasp_readyN | 料盘抓取腰高 + 该组 RPY |
| tray2ready1/2/3/6 | tray2readyN | 料盘2 放置腰高 + **独立** RPY |
| tray2 / tray2_place | tray2 / tray2_place | AP9 → 下蹲 → 拍空孔 → 放置 → 站起 home_tcp |
| grasp | vision_grasp | 只做 AP7 料盘抓取 |
| belt_ready | belt_ready | 第一传送带腰/头 + `tcp`，不动底盘 |
| belt_place | belt_place | 预览 `place_tcp` 整段 6D |
| belt_grasp_rpy | belt_grasp_rpy | `tcp` XYZ + `grasp_rpy_deg`，不拍照不夹取 |
| belt | belt | 底盘 AP5 → 准备 → 认码 → 左右放置 |
| belt_grasp | belt_grasp | 底盘 AP5 → 认码 → 抓取 |
| belt2_ready / belt2_place / belt2_grasp_rpy / belt2 / belt2_grasp | 对应 belt2* | 第二传送带，站点 AP6，参数 `conveyor2` |
| cycle | grasp_belt | §1 的整圈闭环 |
| cycle1 | grasp_belt1 | 只在 AP7 抓和 AP5 放置之间循环。不抓传送带，不去 AP6 / AP9 |
| waist1 / waist2 / waist_jog | 同名 | 腰回 home x / 远排前伸 / 5 轴点动 |
| qr / tray | aruco_detect / detect_tray_holes | 只看码或料盘，不动臂 |

腰 jog：1 脚踝、2 膝盖、3 髋、4 侧倾、5 回转。CAN 4/3/2/5/1。零位站立约 z=0.67，`z_max: 0.67`。

---

## 8. 关键文件

```
orchestrator/cpp/orch_hw_main.cpp     命令分发
orchestrator/cpp/robot_runtime.cpp    cycle、传送带放置/抓取、固定 Z、料盘2
src/grasp_to_standby.cpp              料盘一轮；空孔放置复用，张开而不是合爪
src/move_box_runtime.cpp              分列、A-B-C、传送带直线与到位误差打印
src/move_box_config.cpp               yaml。conveyor2 先整份拷贝 conveyor 再覆盖
include/move_box_config.h
include/seg_pose_bridge.h             class 0 抓取，class 3 空孔
board_belt_aruco/belt_detect_api.py   传送带抓取点 = 原点 + R×offset
board_yf100_aruco/                    料盘 5×5
config/move_box_params.yaml           现场唯一调参入口
config/Robot_Arm_Model.yaml           工具 0.230；J2/J6 限位只进求解器
tools/debug_client.py
web_debug/                            同一套 JSON
```

头相机 SN `261722072173`。头 CAN：30 偏航 / 32 俯仰 / 31 横滚。  
`load_conveyor_station` 之后有 `cfg.conveyor2 = cfg.conveyor`，所以第一套新开关会先抄到第二套，再被 `conveyor2:` 里写明的键盖掉。第二套要保持关闭的项必须在 yaml 里写出来。

---

## 9. 不要再做的事

- 用基座 `Y=0` 给料盘分列，或把盘心 4 cm 再加进抓取 XY。
- 传送带放置加 J2 锁、加一段 XY 撤出。
- 把 2–3 cm 的下降跟踪误差写进 `grasp_offset.z` / `offset.z`。
- 让料盘2 直接引用 `ready1/2/3/6` 的 RPY。
- 在用户点头前把第二传送带改成固定 Z，或改机床信号。
- 删右臂代码、打开 YOLO 定位回退、把皮带参数写进 `tray.*`。

先前会话：[料盘抓取与皮带起点](e7664eb9-388a-499a-8efc-1835dd777a93)、[T170C 抓取分列](fd4f66e4-07f1-48c0-9b5c-6e3b4579aee1)、本轮闭环与固定高度 [传送带放置与精度](a5b4797e-df10-4e29-b342-e25c3524a140)。
