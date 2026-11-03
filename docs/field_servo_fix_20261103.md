# 2026-11-03 手相机伺服现场修复

基于 `938cf78`。此补丁尚未实机验证，不将离线仿真或图像合成测试视为真实抓放成功率。
调度协议、申请/完成上报、地图和任务错误恢复逻辑保持不变；本次只处理视觉、机械臂动作和抓取结束后的纯Z升腰协调。

## 1. 抓取后撤离：先抬，再规划返回

- 料盘抓取和传送带抓取不再预先把“抬升+转腕返回”拼成一条轨迹，避免返回或接缝规划失败导致完全无法抬手。
- 合爪结束后，保持夹取姿态、规划并执行独立竖直抬升。SDK规划失败时，以较低速度重试一次；不采用任意关节插值绕过路径边界。
- 抬升结束保持最后关节指令，允许停住，再规划单独返回。保留原有独占保持→轨迹交接，避免旧保持指令与收手指令交错。
- 第一阶段失败不会取消另一侧已能执行的竖直抬升；第二阶段失败时动作层保留夹爪和已抬升位置。
- `retract_arrival_tolerance_m=0.008`、`retract_arrival_timeout_s=2.0` 用于等待实际反馈到达，避免把“轨迹已播完”直接当成实物已到位。
- 当前抬升速度仍为0.40 m/s上限，实际速度由SDK轨迹和执行器决定。此次没有提高现场速度。

料盘一在本轮要求的两侧都持件后，可在双手实际抬离15 mm时并行纯Z升腰至home腰高，之后再规划返回。腰不在孔边前后移动或转向。若另一只手还需要抓取、处于清盘连续测试或料盘放置，暂不触发这段升腰，以免改变未完成目标的参考位置。

相关参数位于 `config/move_box_params.yaml` 的 `head_grasp`：

```yaml
rise_waist_with_lift: true
waist_lift_start_clearance_m: 0.015
retract_arrival_tolerance_m: 0.008
retract_arrival_timeout_s: 2.0
```

`waist_lift_start_clearance_m` 和下面的交接距离依据关节反馈FK判断，不是外部量具验证的机械净空，须结合工件长度及现场空间验证。上层调度失败恢复尚未调整，动作层保持位置不等于整机一定原地待命。

## 2. 传送带两手交接

第一只手收手线程启动后，另一只手先等待“实际TCP已竖直抬升50 mm”。收到该反馈后立即开始下一只手，与前一只手剩余抬升/返回重叠。未抬离、规划失败或没有反馈时，禁止下一只手进入；不依赖固定sleep。

左右传送带分别配置：

```yaml
next_hand_clearance_m: 0.050
next_hand_clearance_timeout_s: 4.0
```

该值必须与实际抬升行程和工件/夹爪空间匹配。它消除“线程启动但手还没撤开”的时序窗口，不替代完整机器人碰撞模型。

## 3. 料盘放置：可见区对准→目标锁定→受控遮挡末段

- 已重新开启 `visual_servo.tray_place.enabled`。
- 默认 `xy_only=true`：使用头部盘面提供的高度和法向，手相机只更新XY，不每帧自由重估Z和倾角。标记和邻孔仍须通过身份、数量和几何一致性检查。`xy_only=false` 保留原完整平面估计，便于对照。
- 在原灰度几何边缘之外，补充橙色环边缘；拟合真实圆环中心，不使用检测框中心。尺寸比允许通过独立参数适配内外缘，仍保留圆度、残差、关联范围和圆周覆盖检查。半圆或单个无标记邻孔不会直接作为可靠定位。
- 已知占用孔跳过灰度金属边缘，只允许质量合格的橙色盘面环参与定位；避免料盘接近放满时失去所有邻孔。目标孔本身仍始终排除。
- 虚拟孔心在高于C点65 mm范围内、连续3个独立新观测满足稳定和误差要求后锁定。进入前若仍没对准，则以高于C点50 mm为下降参考边界，留在较可见区域继续XY修正。
- 锁定后不再被末段正常遮挡打断，沿固定目标完成有限下降。完全没有建立可靠定位时仍超时退出，禁止盲目张爪。
- 2 mm仍作为优先精对齐误差；料盘放置最终接受误差单独设为4 mm、横向低速门限10 mm/s。末段参考停在C上方1 mm，实际到位允许在C上方5 mm以内；C参考本身仍由现场高度参数决定，没有新增相机深度Z修正。

```yaml
place_xy_accept_m: 0.004
place_lock_height_m: 0.065
place_visibility_floor_m: 0.050
place_goal_stability_m: 0.0025
place_lock_frames: 3
place_finish_speed_m_s: 0.010
place_height_margin_m: 0.001
place_z_accept_above_m: 0.005
```

这里的高度均为TCP相对名义C点的高度。65/50 mm来自此次日志中约42–44 mm后有效观测消失的现象，属于首轮验证参数，不是所有相机安装位置通用的机械常数。左右手 `left_place/right_place.reference_px` 仍应对应已夹工件轴线；当前未重新测量或替用户编造示教值。稳定的图像误差不直接证明真实工件轴心完全对准。

## 4. 门限、取图及诊断

- 抓取参考仍止于C上方2 mm，新增1 mm的实际FK Z测量容差，避免0.19 mm级反馈越界立即失败；没有允许参考轨迹深入C下方。
- 删除 `HandPlacePhotoGuard` 及其配置，不再有专门的放置存图线程与伺服抢帧。保留原有手动拍照工具和既有调试能力；默认正常伺服不额外保存本次放置照片。
- 原有跟随误差、指令连续性、双手距离及禁止未定位张爪检查保留，不通过全部关闭检查来增加表面成功率。
- 原13:53日志的反馈FK Z突跳尚不能仅凭源码确认根因。新增 `feedback_step`，同一行记录实际关节、FK、参考、命令、周期和读取耗时，帮助区分真实运动与反馈/发送时序问题；没有用滤波把异常隐藏。

新增日志事件：

| 事件 | 含义 |
|---|---|
| lift_plan / return_plan | 独立阶段规划尝试、返回码和轨迹行数 |
| retract_sdk_failed / lift_connector_failed | 区分SDK失败与起点交接失败 |
| lift_start / lift_feedback / lift_arrival / lift_done | 起抬、20Hz实测反馈、到达与完成 |
| return_start / return_arrival / return_done | 第二阶段开始、到达与完成 |
| lift_failed / return_failed | 分阶段失败原因 |
| peer_wait_clearance / peer_clearance | 后一只手等待/获得实际抬离放行 |
| waist_rise_start / waist_rise_end | 纯Z升腰开始/结束 |
| feedback_step | 实际FK高度变化超出当前下降速度预期，供诊断 |

CSV新增 `place_locked`、`stable_goal_frames` 及邻孔ROI/轮廓拒绝计数。`tools/analyze_visual_servo.py` 已支持新增事件并可继续读取旧日志。

## 5. 本地验证与机器人部署

9项硬件无关CMake测试通过：common_paths、visual_servo、visual_servo_runtime、retract_trajectory、retreat_gate、grasp_execution、visual_servo_config、tray_localizer、python_runtime_paths。

覆盖：双手延迟观测、可见区内锁定及遮挡末段、无观测禁止盲放、合理Z反馈容差与超界拒绝、橙色环和占用孔处理、移动相机下固定盘面、错误/超时阻止交接、配置快照回读、保持指令独占交接。仿真中的运动学及SDK均为合成适配器。

对实际 `function.cpp`、`grasp_to_standby.cpp`、`move_box_runtime.cpp`、`hand_shadow.cpp` 进行了C++语法检查；对本次修改涉及的编排函数检查了非调度部分的语法（缺少真实调度库时使用最小类型声明）。未完成机器人专用SDK的完整链接或真实运动验证。

机器人更新源码后重新配置/编译，运行正式程序，避免误启动旧 `t170c_debug`：

```sh
cmake -S . -B build_robot
cmake --build build_robot -j4
./build_robot/t170_factory 2>&1 | tee startup-servo-fix.log
```

沿用现有现场测试命令分别验证双手料盘抓取、传送带抓取和料盘放置。重点保留启动日志和新CSV；放置关注是否出现 `target_frozen_place_visible` / `ready_place_locked` 及确认张爪，撤离关注 `lift_done` 后是否才出现 `return_plan`，传送带关注 `peer_clearance` 是否发生于实测抬升后。

```sh
python3 tools/analyze_visual_servo.py logs/visual_servo --json servo-summary.json
```

这版是供现场验证的实现，不承诺尚未实测的成功率。模型权重、手眼文件、左右放置参考像素和真实碰撞净空仍以现场文件和验证为准。
