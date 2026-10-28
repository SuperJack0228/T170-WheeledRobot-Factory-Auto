# T170C 交接文档（给下一任 AI）

更新：2026-10-20  
权威目录：`/home/ti5robot/T170Clean`  
用户：`ti5robot`。Ubuntu aarch64（Jetson/Tegra），无 ROS2。  
机器人：钛虎 T170C，双 7 轴 + 腰腿 5 轴 + 头 3 电机 + DEX1 夹爪 + 仙工底盘。

**以本文和当前代码为准。** `config/move_box_params.yaml` 里不少注释过时（传送带站号、收手目标、「料盘二仍走直线」）。站点以 `chassis:` 为准，动作以 `orchestrator/cpp/robot_runtime.cpp` 为准。

当前分支 `20261015`，已推送尖端 `c16f770`。工作区还有未提交的现场修改。不要改历史，除非用户明确要求。

---

## 0. 立刻要遵守的硬约束

1. **不要用 ROS2。**
2. **右臂当前解锁：`kLockRightArmMotors = false`**（`src/Ti5_socketcan.cpp`）。再锁定只改这一处为 `true`。
3. **右 DEX1 经常扫不到。** 启动允许少一个夹爪；缺侧跳过，手臂仍走。不要把「没夹爪」当成启动失败。
4. **YOLO 只给类别（+关联用粗点），不参与抓放定位。** 料盘孔位 XY 来自 ArUco。码不够禁止回退 YOLO 抓取。detect 权重走算法 2（框中心+深度），不再造伪 mask / 跑分割轮廓。
5. **不要把盘心再加进孔位 XY。**
6. **C++ 改完必须重编，并且用户自己重启进程。**  
   `cmake --build /home/ti5robot/T170Clean/build_robot -j2 --target t170c_debug`  
   进程会缓存手眼和 yaml。Python 料盘/传送带识别也是进程启动时加载，改 `board_*` 同样要重启。**不要擅自启动 `t170c_debug`。** yaml 数值可 `python3 tools/debug_client.py reload`，新分支逻辑不行。
7. **不要 git commit / push，除非用户明确要求。** 不要在文档或回复里写密码、token、私钥。
8. 软件 `abort` **不是急停**，也杀不掉正在跑的服务端命令。客户端 Ctrl+C 只断自己。停进程：`pkill -f build_robot/t170c_debug`。客户端 `Expecting value: line 1 column 1` 是对端收下连接后没有返回 JSON，多半是进程已死。连接被拒是服务没在听 8099。
9. 头远排俯仰 **`far_pitch_deg: 47`**。标定俯仰 40°。传送带拍照俯仰 38°。不要改成 65°。
10. **抹角长度 6 cm**（`kCornerBlendM = 0.06`）。抹角失败不要退回两段停车直线。
11. **不要在手臂 `set_Position` 之外再无锁开 CAN 线程。** 电机 SDK 有 `g_motor_sdk_mu`。已有的腰线程必须走这把锁。底盘是仙工 TCP，可以和手臂并行。
12. 坐标系：**基座 +X 前、+Y 左、+Z 上。** 料盘 / 传送带识别系同样前左上。
13. **行偏置叠在该手的全局偏置上**，不是替换。`from_robot` 1 = 最远，6 = 最近。`head_grasp.row_offset` 只管料盘一抓取；`tray2_place.row_offset` 只管料盘二放置。
14. **不要装 2026-10-10 那套棋盘格手眼**（平移约 28 mm，一致性更差）。当前用的是 10 月 9 日 `session_head`。

---

## 1. 站点

| 名字 | yaml | 工位 | 用途 |
|------|------|------|------|
| 料盘一抓取 | `tray_station` | **AP7** | `head_grasp`，5×5 码 ID 2/5/6/7 |
| 传送带一 | `belt_station` | **AP5** | `conveyor` |
| 传送带二 | `out_station` | **AP6** | `conveyor2` |
| 料盘二放置 | `tray2_station` | **AP9** | `tray2_place`，6×6 码 ID 0/1/2/3，空孔 class 1 |
| 调度初始等待 | `dispatch_wait_station` | **AP14** | 到站后保持 home 姿态等任务。任务结束且 20 s 内没有下一条，或普通失败，也回 `standby_point` 或 AP14。充电结束去 `charge_wait_station`（默认 AP12） |

底盘 `192.168.192.5`。导航完成才算到站：仙工 `task_status==4`，或发导航前已经在站上。行进中站名提前变成目标站**不算**到站，不能据此拍照。`note_station_arrived()` 只在这两种情况打。

传送带腰：`waist_x: 0.15`，`waist_z: 0.63`，头俯仰 38°。料盘抓取/放置腰高 `ready_z_m: 0.36`，`ready_forward_m: 0.03`。home 腰 `layer3_home` 是 x=0、z=0.63。料盘一远排前伸 `waist.far_row_forward_m: 0.20`。料盘二放置远排用自己的 `tray2_place.far_row_forward_m: 0.20`，不要和料盘一共用。

调度：型号 `T170C2.0`，编号 `TH-T170C-0124`，调度 `192.168.122.120:8080`。点位由系统下发，不要写死成 AP5/AP6/AP7/AP9。库在 `/home/ti5robot/Desktop/ti5-jindi-faactory-robot-client`，不在本仓库。

---

## 2. 命令现在实际做什么

客户端：`python3 tools/debug_client.py <cmd>`，只连 `127.0.0.1:8099`。

| CLI | 现在的行为 |
|-----|------------|
| `grasp` | **清空料盘一**：每轮收手后夹爪收到 0.2（不是全开），立刻再拍再抓，直到没有目标。停在原地，底盘不走，腰不去传送带。 |
| `cycle` / `grasp_belt` | 整圈。抓取**保持合爪**，可以出发。见 §3。 |
| `cycle1` / `grasp_belt1` | 只在 AP7 抓和 AP5 放之间循环。回 AP7 时站名一变就提前摆头、手臂和**降腰**。 |
| `cycle3` | 先在本地摆传送带二准备（不导航）→ 去 AP6 抓 → AP9 **真放置** → 停在 AP9，不升腰。 |
| `belt_grasp` | 去 AP5 抓，收手回 `conveyor.tcp`。**不出发。** |
| `belt2_grasp` | 去 AP6，先准备再抓，收手到料盘放置 `standby`。**不出发。** |
| `belt2` / `belt2_place` | 去 AP6 放置。不抓。 |
| `tray2` / `tray2_place` / `tray2test` | **都是精度测试** `vision_place_tray2_precision`，不是真放置。见 §4。 |
| `tray2center` | 底盘不动，夹爪保持闭合，去码中心。不要改，除非用户要求。 |
| `dispatch` | §6。不要和桌面 `调度通信测试.py` 同时开。 |
| `home` | 头标定（俯仰 40°）→ 双臂 `home_tcp` → 腰 yaw 0、x 回 home、z 到 0.63。**不动底盘，不开爪。** |

`cycle` / `cycle3` / 调度下料里的真放置才是 `vision_place_tray2`。不要把 `tray2` 调试命令当成它。

---

## 3. 整圈 `cycle` 的顺序

```
AP7 抓毛坯（夹爪保持闭合）
  收手到传送带准备 tcp
  收手结束：底盘去 AP5，同时腰到拍照位（远端先收 x 再升高，见 §5）
→ AP5 放置（松爪到 0.2）→ 原地抓半成品
  右手合爪后 2 s 底盘去 AP6，不等收手完成。这时不要一起降腰。
  收手：平移时保持腕角，到位再 40°/s 转到准备姿态
→ AP6 放置 → 原地抓成品
  收手到料盘放置 standby
  右手合爪后 2 s 底盘去 AP9
  右手收手到位后再等 2 s 才降腰。底盘和降腰不要绑在同一时刻。
→ AP9 真放置（张爪）
  放完：先升腰，再去 AP7，到站后再降腰
→ 下一圈
```

传送带和料盘的伸手、收手速度在 `config/move_box_params.yaml`。当前伸手/收手 **0.35 m/s**，下降/抬起 **0.25 m/s**。改之前先读文件，不要用文档里的旧 0.20 覆盖。腕再定向仍是 40°/s。接缝单关节上限仍是 25°/s，不要改回 8°/5 ms。

夹爪比例：`0` 全开，`1` 全合。传送带放置松爪、传送带抓取准备、料盘一抓取的预备和清空时松爪，都是 **0.2**。`closeAndWait` 的第二个 0.2 是容差，不是开合度。

---

## 4. `tray2` 精度测试（调试命令）

`vision_place_tray2_precision`：

1. 本地摆传送带二抓取准备（腰/头/手臂），此时不导航。
2. 底盘去 AP9。
3. 到站后降腰。
4. 头到标定角，手臂到料盘二 standby。
5. 两爪 `graspAndWait` **一次，之后保持闭合**。不要到孔位再张开。
6. 空孔从前排开始。到位后在跑 `t170c_debug` 的终端等回车。输入 `quit` 再回车则退出（返回 -4）。
7. 回车之后、收手之前，打印该臂 J1–J7 编码器：理论值、实际值、差值（计数，不是角度）。
8. 远排（`from_robot` 1–3）用 `tray2_place.far_row_forward_m`，头俯仰 47° 再检测。近排头俯仰 40°。
9. 结束时保持下蹲，不再导航。

---

## 5. 腰：什么时候能动

**去传送带拍照（远排抓完）：** 腰若在 x 比拍照 x 大过 2 cm 的位置，先在当前高度收到拍照 x，再升高。不要锁着前伸 x 从深蹲直接升到 0.63。近处仍是先站直再前伸。

**传送带二放完，下一步是料盘一抓取**（人已经在 `out_station`）：底盘先走，**到站后才降腰**。路上可以摆头和手臂。

**料盘二放完要去 AP7 抓**（人在 `tray2_station`，或 `vision_place_tray2` 的 `return_to_tray=true`）：**先升腰，再开底盘，到 AP7 后再降腰。** `cycle` 走这条。`cycle3` 和调度下料 `return_to_tray=false`，放完停在 AP9 不升腰；下一次从这里发出的料盘一抓取，同样先升、再走、再降。

**调度开局：** 去 AP14 的同时腰升到 home。到站后再 `go_home`，以 home 姿态等待，等待时不挥手。任务结束且没有下一条：先 `go_home`，再去 `standby_point` 或 AP14。`1101` / `1102` / `1201` 未解决时留在原地。充电结束去 AP12，不回 AP14。

**不要改**腰 ready 检查那条：dx 约 0.0126 大于 0.01 就放弃。用户没要求就别放宽。

传送带伸手时腰抖一下：没有新的腰指令，是直立位被手臂起步带动。用户要求过不改。

---

## 6. 调度

库是 `ti5-jindi-faactory-robot-client` **1.8.0**。没有 `report_state`，不要再发 `STANDBY` / `WORKING` / `GO_CHARGE` / `commuting` / `CHARGING` / `ERR_*`。这些只留在本机面板。调度靠每 2 秒的 `POSE` 看位置，报文里有 `current_point`（底盘当前站名，由程序写入 `RobotClient::current_point`）。

`apply_mission` 在料架、待命点、两个工序点都空时，先发 `NEED_POINTS` 再要任务。任务报文里的同名点才覆盖：`raw_material_point`、`well_done_point`、`standby_point`、`sequence_one`、`sequence_two`。任务结束不清。收到 `CHARGE` 时库清空料架、待命点和工序点，并写入 `charge_point`。无任务去 `standby_point`，还没有则去 yaml `dispatch_wait_station`（AP14）。到站后保持 home 姿态等任务，不等待时挥手。任务：`TASK_LOAD` / `TASK_TRANSFER` / `TASK_UNLOAD` / `TASK_CHARGE`。缺站点就不动。

| 任务 | 动作 |
|------|------|
| 上料 | 料架抓毛坯（保持合爪，可以出发）→ 去上料点的同时申请放置，超时 20 s 后直接放 |
| 转运 | 上料点不申请。整图检测框中心转到码系，窗口 x∈[-0.05,0.05]、y∈[0.10,0.13]：0 个不抓报 `1102`，1 个只右手，≥2 先右后左。手臂仍走 `grasp_offset`。抓完发 `PICK_UP_DONE`。去下料点路上申请放置；放置 Z 再低 5 mm |
| 下料 | 下料点不申请。到站拍一次，用同一抓取窗口决定伸哪只手。抓完去成品点真放置。`DONE` 等放置结束才发 |
| 充电 | home → 待命点 → 挥手下垂 → 腰降到抓取高度 → 失能腰/臂/头（右臂锁定则跳过右臂）→ 去充电点。达到 `charge_done_percent`（yaml 默认 95，可热改）后 `mission_done` 并断开充电继电器，重新使能。手臂回 `home_tcp`，腰高不动，底盘去充电等待点（默认 AP12） |

申请超时 **20 s**。放置申请超时：**直接放**。传送带抓取不再申请。客户端日志看不到「申请已发出」；只有 `t170c_debug` 控制台有「去某站的同时发送 APPLY_…」以及没有「发送失败」。

`1101` / `1102` / `1201` 上报后等网页。返回 true：不再申请，原地再拍，重做刚才的抓或放，不回 AP14。返回 false：停在当前站等下一条，20 s 超时也不回 AP14。传送带取帧、二维码、检测失败先自动再拍一次，第二次仍失败才上报 `1201`。

手里还有件的失败：留下，不报完成，不回料盘再抓（`kDispatchHoldStay=9`）。上料/转运/下料成功后在当前位置再听 **20 s**，没新任务才 `go_home` 并回待命点（`standby_point` 或 AP14）。充电成功停在充电等待点，在那里断线重连也不改去 AP14。

传送带抓放的逐步信号见 [`dispatch_flow.md`](dispatch_flow.md)。

进程内只有一个仙工连接，2 Hz 写位姿、电量和 `current_point`。不要再 `Disconnect`，不要再 new 第二个 `Seer`。

---

## 7. 视觉

### 分层与谁调谁

**目标模块（权威见草案）：** M1 Camera · M2 DL · M3 Marker · M4 Pose · M5 编排；Skill 在模块外。  
旧称对照：Sensor→M1；YOLO/码检测→M2+M3；Fusion/手眼→M4；`perceive_*`/Bridge→M5。

| 层（旧称） | 对应 | 职责 | 现状入口 |
|------------|------|------|----------|
| Sensor | M1 | 取图、K/dist、槽位 | `realsense_get` → `CameraFrameData` |
| Detector | M2+M3 | YOLO；料盘/传送带 ArUco | `feeding_*`、`tray_detect_api`、`belt_detect_api`；`SegPoseBridge` |
| Fusion | M4 | 孔融合、手眼→基座 | `tray_fusion` + `cam2robot`（仍散落） |
| 编排 | M5 | 取帧→检→融合收口 | `perceive_tray/belt_head`（Bridge 仍偏厚） |
| Skill | — | 抓/放/IK/导航 | `robot_runtime` / `move_box_runtime` |

业务路径（省略失败分支）：

```
Skill
  → perceive_tray_head / perceive_belt_head   （M5 铺路）
       → 取帧 + run_yolo / run_tray / run_belt （仍经 SegPoseBridge）
  → Skill：门禁 / 选孔 / TCP / 偏置 / 抹角
传送带门禁：整图检测框中心转到码系。抓取看固定窗口；放置看放置点 ±3 cm
```

**边界：** 感知不导航、不开爪；Skill 不直接 `import` Python 检测包（经 Bridge）；调试图只经 `vision_debug`。

**C 已落地（收口）：** 业务编排走 helper，勿再复制「取帧→YOLO→料盘/皮带」：

- `grab_head_fused_frames` / `perceive_tray_head` / `perceive_belt_head`（`seg_pose_bridge.h`）
- Bridge 稳定名：`run_yolo` / `run_tray` / `run_belt`（薄包装旧 API）
- 已改调用方：`fill_head_assign_from_frame`、`count_tray_zone_holes`、料盘调试命令、`tray2center`、皮带识别/抓取

**D 已落地（Python fusion）：** `board_yf100_aruco/tray_fusion.py` 专管孔↔YOLO 关联与多帧扶平平均；`tray_detect_api` 只做 ArUco/孔几何/画图并编排。C++ 仍调 `detect_and_annotate` / `fuse_and_annotate`。自测：`python3 board_yf100_aruco/tray_fusion.py`。

**Detect 链路已换：** `best1015` 检测 → 算法 2 框中心粗点（`pose_box.py`）；无伪 mask。关联仍用该粗点挂类别到孔；抓放 XY 仍是 ArUco。`vision_detect: 2`；分割权重仍可走 0/1。

**视觉模块权威约定：** [`vision_modules_draft.md`](vision_modules_draft.md)  
- **M1–M5 薄封装已落地（S3–S6）**：`include/vision/`（见该目录 README）。  
- 历史 `perceive_tray_head` / `perceive_belt_head` 转发到 `vision::pipeline`。传送带整图 YOLO 走 `BridgeDlEngine`（`belt_presence`），不再裁切 ROI。  
- **传送带码系窗口** ✅；**料盘 class 挂孔** ⏸（S2）；**S8 预热/并行** ✅（启动空图预热；料盘与传送带 YOLO∥检码）；**S7 搬家** ⏳。  
- `SegPoseBridge` 不改名。

### 调试图开关（VisionDebug）

环境变量 **`T170_VISION_DEBUG`**（进程启动时读，改完需重启）：

| 值 | 行为 |
|----|------|
| `0` 或未设 | **默认**：业务路径不写 `picture_debug`、不弹窗 |
| `1` | 写标注图 + 原图/标注成对（`pose_vis_save_*`） |
| `2` | 在 1 基础上允许 OpenCV GUI（本机无显示器勿开） |

实现：`include/vision_debug.h`、`src/vision_debug.cpp`；`orch_hw_main` 里 `vision_debug_init_from_env()` + `pose_vis_set_save_debug` / `pose_vis_set_gui_enabled`。历史名 `kDebugVisualize` 现为读该开关的代理，不再写死 `true`。路径规则：`picture_debug/<slot>/<stem>_YYYYMMDD_HHMMSS_mmm.jpg`（`vision_debug_make_path`）。

例：`T170_VISION_DEBUG=1 ./build_robot/t170c_debug`。计时另见 `T170_TIMING`（[`ProblemNote.md`](ProblemNote.md) 待处理第 3 条）。

### 当前在用的手眼

`free_calib/config/camera_to_head_connector_result.yaml`  
平移 mm：`[50.731, 25.915, 117.226]`，欧拉 xyz 度：`[-108.804, 3.475, -90.882]`。  
这是 10 月 9 日 `session_head`。备份不要删：

- `.bak-20261009`：9 月 21 日原标定
- `.bak-20261009-before-manual-xy`：同一套 session_head
- `.bak-20261009-manual-xy`：按偏置反推过、已弃用

`config/camera_to_base_result.yaml` 只是 `free_calib` 失败时的后备，仍是旧外参在 yaw=0、pitch=40、roll=0 下预计算的，**没有**按现在这套头相机矩阵重算。

头 CAN：**偏航 30，俯仰 31，横滚 32。** `T_base_cam = T_base_head * T_head_cam`，只读矩阵的 `a.matrix`。进程启动后缓存在 `g_calib_ok`，换文件必须重启。

把一个基座系平移写进手眼，在俯仰 40° 和 47° 上等价于一个全局偏置，消不掉孔和孔之间的差异。不要改旋转，也不要把「偏 35 mm」直接写进 yaml 第一分量。

### 料盘码

料盘一抓取**仍认** 5×5：前左 2、前右 5、后左 6、后右 7。打印 40 mm，黑框 31.2 mm，角距 0.30 m。无中心码。孔 6×6，孔距 75 mm。照片下方是前（+X）。远近按基座 X 重排，不按 ArUco 行号。

料盘二：6×6，前左 0、前右 1、后左 2、后右 3。黑框 32 mm。码的「上」指向照片上方，照片上方是远、是 +X、是 row 1。

**新的统一码组已写好，默认关闭**（`board_yf100_aruco/config.yaml` 的 `tray_grasp_code.enabled: false`）。5×5，黑框 31.2 mm，角距 0.30 m。左上和右下都是 **100**（定位），右上和左下都是 **2**（这一组的信息号；以后各盘这个号不同）。对角线同号，是为了料盘转 180° 时孔仍落在同一批对称孔上，定位框不靠四个不同 ID。启用：把 `enabled` 改成 `true` 并重启；或调用 `set_active_board("grasp_code")`。程序自己不会切过去。当前抓取不要改去认这组。

调试图（仅 `T170_VISION_DEBUG≥1`）`picture_debug/head/tray_yolo_*.jpg`、`belt_aruco_*.jpg` 会把四个 ArUco 角画成红点。要看到新画法必须重启进程。

### 孔位 Z

`refined_object_top_z`：相机 Z 与参考差不超过 `tray.z_refine_max_m`（3 mm）才用相机，否则用卷尺参考。参考是 `z_ref_m - (腰 z - z_ref_waist_z_m)`，**不看腰的 X**。所以远排和近排日志里的 TCP Z 可以一样，人看到的高度仍可能不同（腰前伸后的真实高度、腕俯仰都不同）。日志 Z 是 260 mm 的 TCP，不是钳口。

TCP 工具长 0.260 m，沿法兰 +X，左右都是。改 `config/Robot_Arm_Model.yaml` 后要重启。

---

## 8. 偏置

全局和行偏置都在 `config/move_box_params.yaml`，现场还在改数字。**改之前先读文件，不要用本文里的旧数覆盖。**

清零前的传送带和手眼相关偏置在 `config/offsets_backup_20261010.yaml`。那是清零当时的快照，不是现在的料盘二放置值。料盘二放置的全局和行偏置后来从调好的料盘一抄过来，之后用户又单独改过行。不要把备份盖回去。

传送带偏置后来又被调过，以 yaml 里 `conveyor` / `conveyor2` 的 `offset` 和 `grasp_offset` 为准。

---

## 9. 已经踩过、不要改回去

- 手臂轨迹和腰/头同时无锁打 CAN → `std::bad_alloc`，客户端只看到空 JSON。进程死了必须重启。
- 传送带抓取收手：平移保持腕角，接缝 ≤25°/s，到位再 40°/s 转腕。
- 拍照只用导航完成时刻，不用行进中的站名。
- 远排不要锁着前伸 x 直接升到 0.63 m。
- 手里有件时不发 `DONE`，不回料盘再抓。
- 传送带二去料盘二：底盘在右手合爪 2 s 后走；降腰在右手收手到位 2 s 后，不要和底盘同时开始。
- 从传送带二或料盘二去料盘一抓：降腰必须等底盘到站；从料盘二出发要先升腰。
- `home` 的混合逆解预览 `code=-3` 是腕部奇异，仍会下发，不是抹角失败。
- 肘高是因为准备姿态 ry=45° 且 IK 种子是肘朝上。用户没要求就不要改 standby 的 RPY，也不要改 `conveyor.tcp` 的高度。

---

## 10. 启动和关键文件

```bash
cd ~/T170Clean
sudo -v
./build_robot/t170c_debug
```

另开终端：`python3 tools/debug_client.py <cmd>`。

```
orchestrator/cpp/orch_hw_main.cpp          命令分发
orchestrator/cpp/robot_runtime.cpp         cycle、传送带、底盘、调度、腰的时序
src/grasp_to_standby.cpp                   料盘一轮；tray2 暂停、quit、编码器回读
src/move_box_runtime.cpp                   抹角、行偏置、远排前伸
src/vision_debug.cpp                       T170_VISION_DEBUG 写图/GUI 门控
src/function.cpp                           抹角、接缝、编码器理论/实际
board_yf100_aruco/config.yaml              料盘一 2/5/6/7、料盘二 0/1/2/3、备用 grasp_code
board_yf100_aruco/online.py                estimate_diagonal_tray_pose（备用，默认不走）
board_yf100_aruco/tray_detect_api.py       Detector 编排；set_active_board：tray / tray2 / grasp_code
board_yf100_aruco/tray_fusion.py           孔↔YOLO 关联、多帧扶平融合
docs/vision_modules_draft.md               视觉 M1–M5 约定与落地状态（权威）
docs/dispatch_flow.md                      调度信号与动作（比本文 §6 更细）
docs/AI_HANDOFF.md                         本文
docs/ProblemNote.md                        待办与已改记录
include/vision/*.h                         M1–M5 薄接口（camera/dl/marker/pose/pipeline）
src/vision/*.cpp                           上述实现；旧 Bridge API 仍可用
config/move_box_params.yaml                现场偏置和站点
free_calib/config/camera_to_head_connector_result.yaml
```

头相机 SN `261722072173`。截图在 `picture_debug/head/`。

Conda：`/home/ti5robot/anaconda3/envs/human_interaction_env`。直接跑 `./build_robot/t170c_debug` 即可。
