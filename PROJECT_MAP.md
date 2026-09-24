# 工程地图

**当前状态、已修问题、下一步以 `AI_HANDOFF.md` 为准（2026-09-21 已重写）。** 本文只列目录，不描述进度。

这是清理后的 T170C 无底盘调试工程。建议按以下顺序阅读。

## 1. 入口与命令

- `orchestrator/cpp/orch_hw_main.cpp`：初始化真实硬件并监听本机调试命令。
- `tools/debug_client.py`：人用调试入口，提供 `status/qr/grasp/home/abort/reload`。
- `orchestrator/cpp/robot_runtime.cpp`：home / grasp / waist1/2 / belt / waist_jog / ArUco。
- `board_yf100_aruco/`：料盘 5×5 码 + `tray_detect_api.py`（嵌入 C++）。
- `board_belt_aruco/`：传送带 6×6 码，目前仅独立 `online.py`，尚未嵌入。
- `free_calib/`：头相机 `T_head_cam`，运行时按头角现算 `T_base_cam`。

## 2. 圆柱抓取

- `src/grasp_to_standby.cpp`：完整的一轮圆柱抓取状态流程。
- `src/move_box_runtime.cpp`：头相机分配、手相机复核、腰部补偿和夹取动作。
- `src/seg_pose_bridge.cpp`：C++ 嵌入 Python，完成相机帧与视觉算法之间的转换。
- `feeding_cylindrical_parts_alg/algorithm/`：YOLO 分割、PnP 和深度重心算法。
- `feeding_cylindrical_parts_alg/models/best.pt`：当前唯一保留的圆柱模型。

## 3. 二维码

- `online_pose/aruco_detect_api.py`：ArUco 检测和位姿估计。
- `online_pose/config.yaml`：码 ID、实际边长、相机参数等配置。
- `config/camera_to_base_result.yaml`：头相机到机器人基座的标定矩阵。

当前代码识别的是 ArUco 标记，不是普通 QR Code。如果现场贴的是能存储文本或网址的
QR Code，需要另加 QR 解码模块，不能只修改 ArUco 字典。

## 4. 硬件层

- `src/Ti5_socketcan.cpp`：双臂、头部和腰部 CAN 通信及中止状态。
- `src/function.cpp`：运动学、轨迹下发、关节反馈与停稳检查。
- `src/waist.cpp`：腰部笛卡尔运动。
- `src/gripper_interface.cpp`：DEX1 双夹爪。
- `src/realsense_get.cpp`：D435I + 双 D405 采集。

## 5. 配置

- `config/move_box_params.yaml`：抓取、安全范围、腰部和相机帧率。
- `config/Robot_Arm_Model.yaml`：T170 机械臂 MDH、关节限制和 CAN ID。
- `config/realsense_cameras.yaml`：三台相机序列号。
- `config/*_to_base_result.yaml`：头部和左右手相机外参。

## 安全边界

软件 `abort` 依赖程序和 CAN 通信仍然正常，不能代替机器人实体急停。第一次运行清理版时，
应先验证 `status` 和 `qr`，再在低速、空载、有人守急停的条件下执行 `home` 与 `grasp`。
