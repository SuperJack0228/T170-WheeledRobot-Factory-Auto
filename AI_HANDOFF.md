# T170C 交接文档（给下一任 AI）

更新：2026-09-29  
权威目录（机器人本机）：`/home/ti5robot/T170Clean`  
用户：`ti5robot`，主机 Ubuntu aarch64（Jetson/Tegra），无 ROS2。  
机器人：钛虎 T170C，双 7 轴 + 腰腿 LowerBody 5 轴 + 头 3 电机 + DEX1 夹爪 + 仙工底盘。

**以本文和当前代码为准。** `config/move_box_params.yaml` 里 `conveyor:` / `conveyor2:` 上方仍有「第一传送带 AP6、第二传送带 AP5」「抓完回 grasp_tcp」「料盘二放置仍走直线」等旧注释，**不要信注释，信 `chassis:` 和 `robot_runtime.cpp`。**

本轮会话：[调度与动作衔接](11893e6d-7422-405d-adb8-aceae908e7c8)。更早：[料盘抓取与皮带起点](e7664eb9-388a-499a-8efc-1835dd777a93)、[T170C 抓取分列](fd4f66e4-07f1-48c0-9b5c-6e3b4579aee1)、[传送带放置与精度](a5b4797e-df10-4e29-b342-e25c3524a140)。

---

## 0. 立刻要遵守的硬约束

1. **不要用 ROS2。** 没有 rclcpp / topic。
2. **右臂已解锁：`kLockRightArmMotors = false`**（`src/Ti5_socketcan.cpp`）。旧机右臂曾撞断，开关保留。再锁定只改这一处为 `true`。
3. **右 DEX1 经常扫不到。** 启动允许少一个夹爪；缺侧 `open/grasp` 直接跳过，手臂仍走。不要把「没夹爪」当成启动失败。
4. **YOLO 只给类别，不参与定位。** 料盘孔位 XY 来自 ArUco。码不够禁止回退 YOLO 抓取。传送带抓取 YOLO 门禁默认关。
5. **不要把盘心再加进孔位 XY。** `t_avg` 已经在 `xyz_level` 里。
6. **C++ 改完必须重编并重启。** `cmake --build /home/ti5robot/T170Clean/build_robot -j2 --target t170c_debug`。yaml 数值可 `reload`。新命令、新分支逻辑不行。**不要擅自启动 `t170c_debug`，除非用户要求。**
7. **不要 git commit / push，除非用户明确要求。** `origin` 是 jihulab，不要推那里。GitHub 远程若存在，也只在用户点名时推。不要在文档或回复里重复任何 token。
8. 软件 `abort` **不能替代实体急停**，也**杀不掉**正在跑的服务端命令。客户端 Ctrl+C 只断自己。要停进程：`pkill -f build_robot/t170c_debug`，然后再启动才能发下一条。
9. 头远排俯仰是 **`far_pitch_deg: 47`**。不要写成 65°。标定俯仰约 40°，传送带拍照俯仰 38°。
10. **抹角失败不要退回两段停车直线。** 收手衔接失败才允许退回「先抬升、停住、再走直线」。
11. **抹角长度是 8 cm**（`kCornerBlendM = 0.08`），不是 8 mm。不要改成几毫米。
12. **不要在手臂 `set_Position` 的同时再开一条腰/头的 CAN 线程。** 曾经因此 `std::bad_alloc`，进程死掉，客户端只看到 `Expecting value: line 1 column 1`。电机 SDK 已有 `g_motor_sdk_mu`（`sdk_try` / 设位置）。底盘是仙工 TCP，可以单独线程和手臂重叠。去料盘二时腰和底盘一起动，是刻意加的，腰走这条互斥锁。
13. **不要用每天改 offset 去吞 2–3 cm 的随机高度差。** offset 只改下发目标。
14. 坐标系：**基座 +X 前、+Y 左、+Z 上**。料盘 / 皮带 ArUco 同样前左上。

---

## 1. 当前进度（2026-09-29）

整圈、料盘↔传送带一、传送带二↔料盘二、以及调度三任务都已接上。用户在收尾效率和腰的安全顺序，不是在重写识别。

未提交。改完要用户自己重启 `t170c_debug` 才生效。

### 1.1 `cycle`（`grasp_belt`）现在的顺序

一直循环到 abort 或某步失败：

```
AP7 抓毛坯（抹角 0.16 m/s）
  收手不停，只把抓到的手送到传送带准备 tcp
  收手一结束：底盘去 AP5，同时腰到拍照位（见 §2.5，远端不能在 x=0.18 上直接站）
→ AP5 放置（抹角/收手都 0.15 m/s，松爪到 0.2，收手回准备 tcp）
  拍照时钟从导航完成 task_status==4 起算，再补满 3 s
→ 原地抓半成品（0.15 m/s）
  右手一合爪，等 2 s，底盘去 AP6，不等两只手都收到位
  收手平移回传送带准备 tcp，腕角先保持，到位再 40°/s 转到准备姿态
→ AP6 放置
→ 原地抓成品
  收手直接去料盘放置准备 standby（不是 grasp_tcp / home_tcp）
  右手一合爪，等 2 s：底盘去 AP9，同时腰开始到料盘放置高度
→ AP9 空孔放置
  放完升腰并回 AP7，再下一圈
```

`cycle1`（`grasp_belt1`）：只在 AP7 抓和 AP5 放之间循环，放完回 AP7 再抓。回程时**站名变成 AP7 就提前摆头、手臂和腰**，导航完成才算到站，拍照仍在停稳之后。

`cycle3`（`cycle3`）：AP6 抓成品 → AP9 放置 → **停在原地结束**。不升腰，不回 AP7。右手合爪后同样 2 s 出发，腰和底盘一起动。

### 1.2 站点

| 名字 | yaml | 工位 | 参数 |
|------|------|------|------|
| 料盘抓取 | `tray_station` | **AP7** | `head_grasp` + 料盘 5×5 |
| 第一传送带 | `belt_station` | **AP5** | `conveyor` |
| 第二传送带 | `out_station` | **AP6** | `conveyor2` |
| 空盘放置 | `tray2_station` | **AP9** | `tray2_place`，class 3 |

传送带腰：`waist_x: 0.15`，`waist_z: 0.63`，头俯仰 38°。料盘抓取/放置腰高 `ready_z_m: 0.36`。home 腰 `layer3_home.z: 0.63`。底盘 `192.168.192.5`。导航完成才算到站：仙工 `task_status==4`。站名提前变成目标站**不算**到站（那会在还在滑的时候拍照）。

---

## 2. 动作（和代码一致，不要按 yaml 旧注释）

### 2.1 料盘抓取（AP7）

- class 0 毛坯。列 1–3 右、4–6 左。远近用 `from_robot`（1 最远）。远三排腰前伸 `far_row_forward_m: 0.18`，头 47°，按编码器重算外参再拍。
- `use_hand_camera: false`。`use_bezier_grasp: true` 且 `approach_nostop: false`：**伸手是抹角**，巡航 **0.16 m/s**，B 前约 8 cm 抹到 C，不在 B 停车，失败不退回分段直线。
- 收手：竖直抬 `lift_after_grasp_z`（0.08 m）后**不停**，直线去 **传送带准备 `conveyor.tcp`**，不是 `home_tcp`。速度 `lift_vel` / `return_vel`，yaml 里都是 **0.16 m/s**。只收回抓到的那只手。
- 合爪的同时预规划这段收手。衔接失败才退回先抬、停、再走直线。
- 收手全部结束后，底盘线程立刻去 AP5；主线程同时把头和腰摆到传送带拍照位。不要再改成「等腰完再开底盘」，也不要在收手还没完时动腰。
- `approach_vel_m_s: 0.12` 只留给仍走直线的旧路径，**不是**传送带抓取速度。

### 2.2 两条传送带放置

伸手抹角、收手两段直线，都是 **0.15 m/s**。松爪到闭合比例 **0.2**（0 全开，1 全合），不是张到最大。到位后仍停 0.5 s 再松。收手抬 **3 cm** 后不停，回到该传送带的 `tcp`。

拍照：`go_belt_station` 从 **导航完成** 起算满 **3 s**。准备动作如果已经花掉这 3 s，就立刻拍。不要再把「站名已经是 AP5」当成到达。

### 2.3 传送带抓取

伸手抹角、收手平移都是 **0.15 m/s**。第一只手拍照前仍停 **1.5 s**；第二只手底盘没动，**不再多停 1.5 s**，但仍重新拍照。抓取点停 0.5 s 再合爪。准备时夹爪先收到 0.2。

收手平移时**保持抓取腕角**。接到目标位置后，再以 **40°/s** 转到目标姿态。两段接缝的单个关节上限是 **25°/s**（`stitch_cruise_lines`）。以前是 8°/5 ms（约 1600°/s），右手从皮带中间收回时会抽一下。不要把这个上限改回。

收手目标：

| 从哪抓 | 收到哪 |
|--------|--------|
| 传送带一（AP5） | `conveyor.tcp`（和抓取准备同一套） |
| 传送带二（AP6） | yaml `standby`（料盘放置前的准备，不是 `grasp_tcp`，也不是 `home_tcp`） |

`grasp_tcp`（姿态 0,0,0）还在 yaml 里，**这两段抓取收手已经不用它。**

联动出发（`start_belt_depart_after_right`）：右手合爪成功后 **等 2 s**，底盘出发，收手继续。单独的 `belt_grasp` / `belt2_grasp` **不会**因此开走。

- 传送带一之后要去传送带二（`cycle`、调度转运）：2 s 后去 **AP6**。这时不要同时动腰。
- 传送带二之后要去料盘二（`cycle`、`cycle3`、调度下料）：2 s 后去 **AP9**，并且腰同时收到料盘放置高度（`ensure_waist_ready_start`）。不要等导航完成才动腰。

左手合爪后就可以开始申请右手，和左手收手重叠。`PICK_UP_DONE` 仍是合爪后至少 2 s 才发。

### 2.4 料盘二放置（AP9）

伸手同样是抹角 **0.16 m/s**（`tray_place_uses_corner`）。收手两段直线不停，目标是 `home_tcp`，张爪时把这段规划好。放置是张爪，不是合爪。

- **`tray2` / `tray2_place`**：放完 **不升腰、不回 AP7**，停在原地。
- **`cycle`**：放完仍升腰并回 AP7，以便下一圈抓毛坯。`vision_place_tray2(..., return_to_tray=true)`。
- **`cycle3`**：AP6 抓 → AP9 放 → 停在原地结束。
- **调度下料**：放完停在 AP9 等下一条任务，不回 AP7。

### 2.5 远端抓完，腰怎么去拍照位（2026-09-29 刚改）

远排抓完腰在 x≈0.18 m、z≈0.36 m（深蹲）。**禁止**锁着 x=0.18 从 0.36 直接升到 0.63。2026-09-29 11:15 那次就是这样：规划终点 (0.180, 0.630) 能解，实际俯仰甩到约 −11°，x 缩到约 0.07 m，z 冲到 0.646 m；接着在这个偏高的位置把 x 推到 0.15 m，`Move_Limit`，`code=-2`，日志 `grasp_depart 腰平移 X 失败`。底盘当时已经到了 AP5，失败的是腰。

现在 `conveyor_waist_and_head`：若当前 x 比拍照 x 大过 2 cm，**先在当前高度收到拍照 x=0.15，再上升**。近处仍是先站直再前伸。上升后 |dx| 或 |dz| > 2 cm 不算到位，先收到 `waist_ready_x()` 再站直，然后再伸到 0.15。

---

## 3. 调度（已接入，不要再说「机床还没做」）

机器人**不连机床**。只连调度 `192.168.122.120:8080`（`RobotClient`，库在 `/opt/ti5-jindi-faactory-robot-client`）。机床允许放/抓，由调度回 `PLACE` / `PICK_HALF` / `PICK_WELL`。

启动先回 AP7 待命，报 `STANDBY`，卡住等 `apply_mission()`。任务只有 `TASK_LOAD` / `TASK_TRANSFER` / `TASK_UNLOAD`。`CHARGE` 能被库返回，**没有实现**。

| 任务 | 动作 |
|------|------|
| 上料 | AP7 抓毛坯 → AP5 申请放置 → 放下 |
| 转运 | AP5 左右手各申请抓半成品 → AP6 申请放置 |
| 下料 | AP6 按空孔抓 1 或 2 只手 → AP9 放入空孔 |

申请超时 5 s。抓的超时跳过该手；放的超时不放。**手里还有件时不发 `DONE`，也不回 AP7 再抓**（`kDispatchHoldStay`）。手里没件的失败仍回 AP7。

回报都是先满 2 s 再发：`PLACE_DONE`、`PICK_UP_DONE`、`DONE`。上料把开抓前数到的零件拿光，另发 `RAW_MATERIAL_EMPTY`。下料放下后没有空孔，另发 `WELL_DONE_MATERIAL_FULL`。

上料、转运完成后原地等 **5 s**。这 5 s 内来任务就在原地做；没有任务才回 AP7，到站再停 2 s。下料放到 AP9 之后**一直等**，没有这 5 s 回 AP7。

人已经在对应传送带、双手都在该站准备 `tcp` 附近（12 mm、5°）时，转运/下料**不再 `go_home`**，直接申请并抓。否则路上直接摆拍照准备，不先回 home。

转运抓取在传送带 +X 再加 5 mm；转运放置松手 Z 再低 5 mm。只影响这两段。

---

## 4. 已经踩过的坑

- **`std::bad_alloc` / 客户端 `Expecting value`**：手臂轨迹和腰/头同时打 CAN，堆被打坏，未捕获异常把进程杀掉。SDK 已加锁。不要再无锁并发 CAN。进程死了必须重启，`abort` 没用。
- **收手抽一下**：两段直线接缝用关节空间抄近路，并且曾允许单关节 8°/5 ms。传送带抓取已改成平移时保持腕角、接缝 ≤25°/s、到位再慢转腕。
- **拍照太早**：把行进中的站名匹配当成到达。现在只在 `task_status==4`（或发导航前已经在站上）打到达时间。
- **远端站起抖一下然后腰失败**：见 §2.5。不要改回「锁 x=0.18 直接升高」。
- **`home` 的混合逆解预览 `code=-3`**：腕部奇异，仍会下发，不是抹角失败，也不是贝塞尔。`use_bezier_grasp: true` 在 `approach_nostop: false` 时表示抹角，不是二次贝塞尔。
- **传送带「未检测到 6x6 码」**：识别失败，命令可以正常返回，不是崩溃。手臂停在准备位。
- **放置申请超时仍报完成**：旧行为已改掉。现在手里有件就不报完成、不回料盘加抓。

---

## 5. 精度（仍有效，先看再改 offset）

到位误差是下发目标对上**编码器 FK**，不是卷尺。传送带直线会等电机停稳再读。FK 已经跟上、零件仍偏，去查腰和视觉，不要把厘米级跟踪误差写进 offset。

`Line_Trajectory`（闭源）路点 IK 返回 0 就算成功，不按 1 mm 复核。料盘抓取会在 C 点用编码器复核，超差不合爪/不张爪。

第一传送带码平面 Z 固定 **−0.29 m**（腰 0.63 m 时基座到皮带 29 cm）。第二传送带仍是相机高度（`use_fixed_origin_z: false`）。用户没说打开之前不要改。

盘面参考锚点仍是腰 **0.55**（`z_ref_waist_z_m`）。不要因为 home 腰是 0.63 就改 `z_ref_m`。

---

## 6. 启动

```bash
cd ~/T170Clean
sudo -v
./build_robot/t170c_debug 2>&1 | tee startup.log
```

```bash
cmake --build /home/ti5robot/T170Clean/build_robot -j2 --target t170c_debug
python3 tools/debug_client.py reload
python3 tools/debug_client.py cycle
python3 tools/debug_client.py abort
```

监听 **仅** `127.0.0.1:8099`。流程在 `orchestrator/cpp/robot_runtime.cpp`。`startup.log` 经 tee 时可能缓冲，崩溃现场以终端为准。截图在 `picture_debug/head/`。

Conda：`/home/ti5robot/anaconda3/envs/human_interaction_env`。直接跑二进制即可。

---

## 7. 调试命令

| CLI | 作用 |
|-----|------|
| ping / status / reload / abort | 探活、快照、重载 yaml、软件中止（不杀进程） |
| home | 头标定 → `home_tcp` → 腰回 home。不动底盘，不开爪 |
| ready1/2/3/6 | 只摆该排抓取准备，不动底盘 |
| grasp | AP7 抓。抓到后底盘去 AP5，腰按 §2.5 去拍照位 |
| belt_ready / belt_place / belt_grasp_rpy | 只动上半身，人要已在 AP5 附近 |
| belt | 底盘去 AP5，停稳后放置 |
| belt_grasp | 底盘去 AP5 抓取。**不会**接着去 AP6 |
| belt2_ready | 底盘去 AP6，只摆准备 |
| belt2 / belt2_place | 底盘去 AP6 并放置 |
| belt2_grasp_rpy | 只摆手腕，不动底盘 |
| belt2_grasp | 去 AP6 抓取，收手到料盘放置准备。**不会**接着去 AP9 |
| tray2ready1/2/3/6 | 料盘二放置准备，独立 RPY，不动底盘 |
| tray2 / tray2_place | AP9 放空孔。**放完不升腰、不回 AP7** |
| tray2test | AP9 精度测试：合爪到位，不松、不起身 |
| cycle | §1.1 整圈，放完 AP9 **会**回 AP7 |
| cycle1 | 只在 AP7 和 AP5 之间循环 |
| cycle3 | AP6 抓 → AP9 放 → 原地结束 |
| dispatch | §3。不要和桌面调度测试脚本同时开 |
| qr / tray | 只看码或料盘孔，不动臂 |
| waist1 / waist2 / waist_jog | 腰回 home x / 远排前伸 / 点动 |

腰 jog：1 脚踝、2 膝盖、3 髋、4 侧倾、5 回转。

---

## 8. 关键文件

```
orchestrator/cpp/orch_hw_main.cpp     命令分发
orchestrator/cpp/robot_runtime.cpp    cycle、传送带、底盘、调度、出发时机
src/grasp_to_standby.cpp              料盘一轮；空孔放置复用
src/move_box_runtime.cpp              抹角/直线选择、收手预规划
src/function.cpp                      抹角、两段直线接缝（25°/s）、慢转腕
src/Ti5_socketcan.cpp                 电机 SDK 互斥、右臂锁
src/waist.cpp                         腰笛卡尔。code=-2 是目标不可达
config/move_box_params.yaml           现场调参。注释可能过时
tools/debug_client.py
```

头相机 SN `261722072173`。头 CAN：30 偏航 / 32 俯仰 / 31 横滚。  
`load_conveyor_station` 之后有 `cfg.conveyor2 = cfg.conveyor`，第二套要保持不同的项必须在 yaml 里写出来。

调度库头文件：`/opt/ti5-jindi-faactory-robot-client/include/robot_client.hpp`。

---

## 9. 不要再做的事

- 用基座 `Y=0` 分列，或把盘心再加进抓取 XY。
- 传送带放置加 J2 锁，或加一段 XY 撤出。
- 把厘米级跟踪误差写进 offset；未让用户确认就给 `conveyor2` 开固定 Z。
- 料盘二直接引用 `ready1/2/3/6` 的 RPY。
- 删右臂代码，或打开 YOLO 定位回退。
- 抹角失败时静默退回两段停车直线；把 8 cm 抹角改成几毫米。
- 把行进中的站名当成导航完成。
- 远排锁着 x≈0.18 m 从深蹲直接升到 0.63 m。
- 收手接缝再用 8°/5 ms 的关节步长。
- 手臂还在 `set_Position` 时再无锁开一条腰/头线程。
- 手里还有件时发 `DONE` 或回 AP7 再抓。
- 未让用户要求就 commit、push，或启动 `t170c_debug`。
