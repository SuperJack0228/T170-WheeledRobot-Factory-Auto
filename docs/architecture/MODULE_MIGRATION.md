# 模块迁移表

计划基线更新为 `e205e9e`（wrist-servo）。第一步本地实现和检查已完成，匹配机器人SDK全量构建/实机验证尚未完成。

原始来源保留在 CSV 的 source 列；current_source 表示本轮归属。status 不能解释成已发布。

| ID | 步骤 | 职责 | 当前实现/待处理来源 | 目标 | 状态 |
| --- | --- | --- | --- | --- | --- |
| C01 | 1 | 现有公共路径/日志 | src/common<br>include/t170/common | src/common<br>include/t170/common | implemented_local_checked_robot_pending |
| V01 | 1 | 取帧、相机槽位、时间戳 | src/vision/camera<br>include/t170/vision | src/vision/camera<br>include/t170/vision | implemented_local_checked_robot_pending |
| V02 | 1 | 模型注册及检测接口 | src/vision/detection<br>include/t170/vision/dl.h | src/vision/detection<br>include/t170/vision | implemented_local_checked_robot_pending |
| V03 | 1 | 码与位姿接口 | src/vision/geometry<br>include/t170/vision | src/vision/geometry<br>include/t170/vision | implemented_local_checked_robot_pending |
| V04 | 1 | 感知编排与公共帧类型 | src/vision/pipeline.cpp<br>include/t170/vision/perception_types.h<br>include/t170/vision/types.h | src/vision<br>include/t170/vision | implemented_local_checked_robot_pending |
| V05 | 1 | Python 引擎、运行结果、PosePipeline 兼容入口 | src/vision/bridge/python_bridge.cpp<br>include/t170/vision/python_bridge.h | src/vision/bridge<br>include/t170/vision | implemented_local_checked_robot_pending |
| V06 | 1 | 调试显示和图像输出 | src/vision/bridge/visualization.cpp<br>src/vision/bridge/debug.cpp<br>python/t170_vision/visualization | src/vision/bridge<br>python/t170_vision/visualization | implemented_local_checked_robot_pending |
| V07 | 2 | 停止信号和应用生命周期辅助函数 | src/vision/bridge/python_bridge.cpp | src/app<br>include/t170/app | planned |
| V08 | 1 | 手相机观测、关联与邻孔观测发布 | src/vision/tracking/wrist_observer.cpp<br>include/t170/vision/servo_observer_port.h | src/vision/tracking<br>include/t170/vision | implemented_local_checked_robot_pending |
| V09 | 1 | 料盘邻孔/标记定位 | src/vision/geometry/tray_localizer.cpp<br>include/t170/common/tray_types.h | src/vision/geometry<br>include/t170/vision | implemented_local_checked_robot_pending |
| V10 | 1 | 头相机安装及姿态变换 | src/vision/geometry/head_transform.cpp<br>include/t170/vision/head_transform.h | src/vision/geometry<br>include/t170/vision | implemented_local_checked_robot_pending |
| P01 | 1 | 检测引擎、位姿、配置解析及类型 | python/t170_vision/detection | python/t170_vision/detection | implemented_local_checked_robot_pending |
| P02 | 1 | 料盘定位、网格、融合 | python/t170_vision/geometry/tray | python/t170_vision/geometry | implemented_local_checked_robot_pending |
| P03 | 1 | 传送带和单码定位 | python/t170_vision/geometry/conveyor<br>python/t170_vision/geometry/marker | python/t170_vision/geometry | implemented_local_checked_robot_pending |
| P04 | 1 | 变换计算类与数学函数 | python/t170_vision/calibration/head_transform.py | python/t170_vision/calibration | implemented_local_checked_robot_pending |
| P05 | 1 | 检测绘图 | python/t170_vision/visualization/detection.py | python/t170_vision/visualization | implemented_local_checked_robot_pending |
| K01 | 1 | 像素映射、控制器与状态历史 | src/control/visual_servo.cpp<br>include/t170/control/visual_servo.h<br>include/t170/common/servo_types.h | src/control<br>include/t170/control | implemented_local_checked_robot_pending |
| K02 | 1 | 基于 FK 的局部位姿求解 | src/motion/pose_solver.cpp<br>include/t170/motion/pose_solver.h | src/motion<br>include/t170/motion | implemented_local_checked_robot_pending |
| K03 | 2 | 实测状态历史、伺服执行、最终保位 | src/servo/runtime.cpp<br>include/servo/runtime.h | src/motion<br>include/t170/motion | planned |
| K04 | 1 | 异步伺服轨迹日志 | src/common/servo_trace.cpp<br>include/t170/common/servo_trace.h | src/common<br>include/t170/common | implemented_local_checked_robot_pending |
| H01 | 1 | 设备 SDK 路由、反馈和中止 | src/hardware/device_bus.cpp<br>include/t170/hardware/device_bus.h | src/hardware<br>include/t170/hardware | implemented_local_checked_robot_pending |
| H02 | 1 | 夹爪驱动与反馈 | src/hardware/gripper.cpp<br>include/t170/hardware/gripper.hpp | src/hardware<br>include/t170/hardware | implemented_local_checked_robot_pending |
| H03 | 1 | 头部与腰部驱动适配 | src/hardware/head.cpp<br>src/hardware/waist.cpp<br>include/t170/hardware | src/hardware<br>include/t170/hardware | implemented_local_checked_robot_pending |
| H04 | 1 | 独立腰腿运动学库 | libs/lower_body | libs/lower_body | implemented_local_checked_robot_pending |
| M01 | 2 | 轨迹规划、两阶段撤离、起点交接和局部求解适配 | src/function.cpp<br>include/function.h<br>include/servo/trajectory.h | src/motion<br>include/t170/motion | planned |
| M02 | 2 | 轨迹播放、关节回读和到位检查 | src/function.cpp<br>include/function.h | src/motion<br>src/hardware<br>include/t170/motion | planned |
| M03 | 2 | 通用位姿数学、单位、杂项旧全局 | src/function.cpp<br>include/head.h | src/common<br>include/t170/common | planned |
| S01 | 2 | 保位、夹爪复核、合爪及反馈 | src/grasp_execution.cpp | src/skills/common<br>include/t170/skills | planned |
| S02 | 2 | 料盘抓放技能主流程 | src/grasp_to_standby.cpp<br>include/move_box_runtime.h | src/skills/tray<br>include/t170/skills | planned |
| S03 | 2 | 目标几何、列分配、行姿态与旧偏置策略 | src/move_box_runtime.cpp<br>src/move_box_config.cpp<br>include/move_box_config.h | src/skills/tray<br>src/skills/conveyor<br>include/t170/skills | planned |
| S04 | 2 | 腰部配合和坐标补偿 | src/move_box_runtime.cpp | src/hardware<br>src/motion<br>src/skills/tray | planned |
| S05 | 2 | 传送带计数、窗口、抓取与放置 | orchestrator/cpp/robot_runtime.cpp | src/skills/conveyor<br>include/t170/skills | planned |
| T01 | 2 | 上料、转运、下料、充电与持件流程状态 | orchestrator/cpp/robot_runtime.cpp<br>orchestrator/cpp/robot_runtime.h | src/tasks<br>include/t170/tasks | planned |
| I01 | 2 | RobotClient 调度协议、申请、上报和连接 | orchestrator/cpp/robot_runtime.cpp | src/integration/dispatch<br>include/t170/integration | planned |
| H05 | 2 | 仙工导航、位置/电量、充电接口 | orchestrator/cpp/robot_runtime.cpp | src/hardware<br>include/t170/hardware | planned |
| A01 | 2 | JSON 服务、设备生命周期和硬件组装 | apps/t170_factory_main.cpp | apps<br>src/app<br>include/t170/app | planned |
| A02 | 2 | 任务面板和控制台输出捕获 | orchestrator/cpp/dispatch_board.cpp<br>orchestrator/cpp/dispatch_board.h | src/tasks<br>src/common<br>include/t170/tasks | planned |
| A03 | 1 | 阶段计时 | src/common/task_timing.cpp<br>include/t170/common/task_timing.h | src/common<br>include/t170/common | implemented_local_checked_robot_pending |
| A04 | 2 | 挥手、鞠躬、欢迎与音频调用 | src/function.cpp<br>orchestrator/cpp/robot_runtime.cpp | src/skills/common<br>src/app | planned |
| Q01 | 2 | 参数默认值、YAML 与校验/快照 | src/common/config.cpp<br>include/t170/common/config.h<br>src/servo/config.cpp | src/config<br>include/t170/config | planned |
| Q02 | 2 | 实际生效参数及标定 | config/*.yaml<br>config/devices.json<br>board_yf100_aruco/config.yaml<br>board_belt_aruco/config.yaml<br>online_pose/config.yaml<br>feeding_cylindrical_parts_alg/config/pose_params.yaml<br>free_calib/config/*.yaml | config/defaults<br>config/robot_models/t170<br>config/sites<br>config/robots/TH-T170C-0124<br>config/calibration/TH-T170C-0124<br>config/deployments<br>config/schemas | planned |
| Q03 | 2 | 仍在代码里的可部署硬件/工位/环境差异 | apps/t170_factory_main.cpp<br>tools/arm_plane_repeat.cpp<br>src/waist.cpp<br>include/Ti5_socketcan.h<br>orchestrator/cpp/robot_runtime.cpp<br>CMakeLists.txt | config/robot_models/t170<br>config/sites<br>config/robots/TH-T170C-0124<br>config/defaults<br>cmake | planned |
| R01 | 3 | 模型和训练材料 | feeding_cylindrical_parts_alg/models/*.pt<br>feeding_cylindrical_parts_alg/models/seg_factory_jindi.yaml | assets/models<br>docs/archive | planned |
| R02 | 3 | 实际提示音 | AUDIO/** | assets/audio | planned |
| R03 | 3 | 第三方及同事提供的 SDK/库 | eigen3/**<br>lib/**<br>ti5-device-sdk/**<br>ti5-agv-seer/**<br>include/Ti5_Arm.h<br>third_party/dex1/**<br>third_party/libserialport/** | third_party/eigen<br>third_party/ti5_arm<br>third_party/ti5_device<br>third_party/ti5_seer<br>third_party/unitree<br>third_party/libserialport | planned |
| R04 | 3 | 诊断、分析和标定命令入口 | tools/arm_plane_repeat.cpp<br>tools/analyze_visual_servo.py<br>free_calib/head_camera_to_base.py | tools/diagnostics<br>tools/analysis<br>tools/calibration | planned |
| R05 | 3 | 网页与公共运维入口 | web_debug/**<br>tools/debug_client.py<br>tools/servo_preflight.py<br>python/t170_common/** | tools/web_debug<br>tools<br>python/t170_common | planned |
| R06 | 3 | 启动、构建、安装和打包 | deploy/**<br>CMakeLists.txt<br>cmake/** | deploy/scripts<br>deploy/systemd<br>packaging/cpack<br>packaging/deb<br>cmake | planned |
| R07 | 3 | 当前14项回归及后续接口检查 | tests/** | tests/unit<br>tests/integration<br>tests/fixtures | planned |
| R08 | 3 | 历史日志、会话、备份和旧说明 | logs/**<br>tmp_calib_session_swap/**<br>free_calib/*.yaml<br>config/*.bak*<br>config/offsets_backup_*.yaml<br>free_calib/config/*.bak*<br>docs/** | tests/fixtures<br>docs/archive | planned |
| R09 | 3 | 入口文档、开发辅助和环境约束 | README.md<br>.gitignore<br>tools/push_today.sh<br>feeding_cylindrical_parts_alg/requirements.txt<br>free_calib/requirements.txt | docs/deployment<br>docs/validation<br>tools/dev<br>python<br>packaging | planned |
