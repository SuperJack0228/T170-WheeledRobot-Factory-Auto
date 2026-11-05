# 抓放后两阶段撤离 V3：2026-11-05

基线 a8e22f3。本次针对11/04料盘抓取的上抬/收手故障；本地检查通过后仍需匹配Linux/ARM64 SDK编译及双手实机验证。历史故障与旧实现见 ARM_KINEMATICS_ASSESSMENT.md；其中“现在代码”的临时J2模型及单点IK返回已由本实现替换。

## 当前链路

1. 视觉下降、合爪保持逻辑沿用。规划时继续保持最后参考；真正开播时用 take_final_hold 原子移交发送权，等待旧保持线程的整组七关节发送退出。同一手的撤离原语串行，左右手仍可并行。
2. 上抬仍用局部FK/阻尼最小二乘，约10mm路点、上一点关节作种子，求解失败自适应缩步。TCP XY和姿态保持，抬升目标仍受 grasp_valid.z_max 限制。不接受只抬2mm后直接横向收手。
3. 上抬关节路点构成自然三次样条路径，全段使用一次五次时间曲线，加速/减速到零；不会在每个10mm节点停车。插值后逐采样用FK检查路径，避免关节插值破坏孔边的几何要求。
4. 两手实际抬离既有15mm门槛后，仍可并行升腰；两手抬完、腰结束后才进入返回。
5. 返回直接用 retract_motion.ready_right_deg / ready_left_deg 电机关节目标。不做准备位IK、不调用SDK Joint_Trajectory、不临时改写模型限位、不再连接SDK的另一套构型。
6. 返回与上抬共用 time_joint_path / run_joint_motion。每关节离散位置流检查速度、加速度和加加速度，包含起止静止边界；超限就延长整段时间，不改关节路径。此实现是保守时间规划，不是时间最优算法，也未引入Ruckig依赖。
7. 名义5ms发送；约20Hz主动查询反馈，通信变慢时不补发追赶突发指令。发送成功后会持续发送末点，直到反馈连续3次进入关节容差才成功。上抬还必须满足实测脱离高度和水平偏移条件。4秒是末点发完后的最大跟随等待，不是每次固定等待4秒。

本次反馈用于实际完成、脱离高度及故障判断，未实现运行中基于跟随误差的连续重定时。时间规划约束的是离散指令流；普通Linux线程、CAN和驱动的实际时序仍需现场日志确认。

## 固定准备位与范围

固定电机角来自 servo_1793782920594052_120317.csv 的左右手 begin 实测关节，原位置约为各自 standby（0.45, ±0.30, -0.25）。这是已出现过的抓取前构型，不是已验证的持件返回路径；它也不等于 conveyor.tcp。下一动作按反馈重新规划。

YAML角度为度，进入内存后为弧度。固定目标直接在电机角空间使用，不按SDK内部模型角限位作未经确认的比较。没有更改 Robot_Arm_Model.yaml；绝对行程、零位和SDK限位坐标系的统一仍待后续处理。

料盘抓取、料盘放置及传送带抓取调用同一上抬/关节返回原语。正常完成都回固定中转关节位。传送带空抓重试使用伸手前读取的实测关节快照，避免下一次重试误以为仍在原准备位。传送带松爪后的旧两段直线、调度协议、视觉PD/检测/抓放门限均未改。

## 外置参数

`config/move_box_params.yaml: retract_motion`：

| 参数 | 当前值 | 作用 |
|---|---|---|
| ready_right_deg / ready_left_deg | 七个实测电机角 | 固定中转构型；不需每次IK |
| max_joint_velocity_rad_s | 0.9 | 各关节速度上限 |
| max_joint_acceleration_rad_s2 | 2.5 | 各关节加速度上限 |
| max_joint_jerk_rad_s3 | 20 | 各关节离散加加速度上限 |
| arrival_joint_tolerance_rad | 0.12（约6.9°） | 中转构型到位容差，不是抓放精度要求 |
| settle_timeout_s | 4 | 发完后等待跟随的上限 |
| feedback_timeout_s | 0.5 | 连续读不到反馈的容许时间 |
| stable_samples | 3 | 到位连续确认次数，采样约20Hz |

抬升高度仍读原 lift_after_grasp_z / 放置高度；抬升速度参数只给期望节拍下界，实际还受关节限制约束。旧 return_vel_m_s、retract_return_vel_m_s 不再控制这条关节返回链路，速度调本组关节限制。旧连接器参数仍用于未迁移的SDK路径。

## 日志与现场验证

控制台 `[retract]` 输出手别（0左1右）、阶段、固定目标电机角(rad)、规划时长、关节限制、进度和跟随差；反馈约每0.5秒打印一次。CSV沿用 logs/visual_servo 原路径与配置快照；新增事件 reason=joint_motion_v2：

- lift_start / return_start：实测起点、参考起点。
- lift_feedback / return_feedback：约20Hz实测关节/FK、参考关节/FK、最大关节跟随差、指令一至三阶差分。
- *_command_complete：仅表示末点已发送，实测标志为false。
- *_measured_arrival：真实反馈到位，标志为true；上抬也满足脱离条件。
- *_arrival_timeout / *_feedback_timeout / *_write_failed / *_abort：执行器故障来源。
- lift_plan：实际求解抬高量、行数、规划失败细分原因。
- *_done：只在实测完成后记录；通用task_event可能含会话旧状态，位置分析应读取上述v2反馈行。

失败时保留夹爪状态；仍可通信则以实测关节保位，不继续催促远端目标。STOP/CAN故障后不补发保位。若读写链路故障，不能凭本机状态保证机械臂已停止，应按现场已有停止流程处理。

```sh
cmake -S . -B build_robot
cmake --build build_robot -j4
./build_robot/t170_factory 2>&1 | tee startup-retract-v3.log
python3 tools/analyze_joint_motion.py logs/visual_servo/servo_实际编号.csv
```

仍使用原双手料盘抓取入口测试；重点观察合爪后竖直脱离、两手实际抬离后并行升腰、返回中转构型是否顺畅。回传本轮CSV及会话参数/模型快照、startup-retract-v3.log；发生异常时，以新事件判断是规划、发送、反馈还是到位问题。其余技能调用虽已接线，不能写成已经实机通过。

## 本地检查

原14项离线回归保留，新增 joint_motion_test：多路点/点到点轨迹的离散v/a/j及静止尾界、端点、双手独立执行、跟随滞后、卡住、反馈中断、写失败、中止、额外脱离条件。配置检查增加七角数量、度转弧度、覆盖值与非法时间限制。

本机支持的 function.cpp / grasp_to_standby.cpp 通过整文件语法检查。robot_runtime 的运动部分可用最小调度声明作局部语法检查，不能替代真正调度SDK构建。机器人动态库是Linux ARM64，本机macOS不能完成真实链接和实机验收。
