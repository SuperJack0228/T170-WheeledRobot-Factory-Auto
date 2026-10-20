# 问题记录

## 待处理

1. 放置和抓取用视觉判断传送带上能不能抓、能不能放。
   - **传送带已落地**：整图 YOLO，检测框中心转到码系。抓取窗口写死在 `robot_runtime.cpp`：x∈[-0.05, 0.05] m、y∈[0.10, 0.13] m。0 个不抓；1 个只右手（右臂锁定则左手）；≥2 双手先右。手臂仍走到 `grasp_offset`。放置是放置点 ±3 cm 内没有零件才放。调试图 `T170_VISION_DEBUG≥1` → `picture_debug/head/belt_yolo_det_*.jpg`。`yolo_roi_enable` 只是检测开关。定位仍用码。
   - 料盘：定位仍 ArUco；YOLO 仍按 class 挂孔（去 class / 有无+占用 **延期**）。
2. 整理视觉模块逻辑，添加模型预热，二维码与 YOLO 写成并行模块。
   - **权威约定**：[`vision_modules_draft.md`](vision_modules_draft.md)（含 M1–M5 / S0–S8 状态表）。
   - **已落地**：A/B/C/D/Detect；**M1–M5 薄封装（S3–S6）** → `include/vision/{camera,dl,marker,pose,pipeline}.h`。
   - **已落地 S8**：进程启动空图预热，拍照不再加载权重；料盘和传送带整图是 YOLO 与二维码并行。抓放决策在有码位姿之后，用框中心的码系坐标，不再裁切 ROI。
   - **尚未做**：目录大搬家（S7）；料盘有无语义（S2 延期）。
3. 测试各步骤，进行精确时间统计，区分 debug 模式与 release 模式，分开日志与文件写入等对资源占用的影响。
   - 已落地：`T170_TIMING=0|1|2` → `logs/timing_*.jsonl`。
   - 尚未做：debug/release 画像、写调试图单独计时、网页展示。
4. 进腰逻辑（远处三排抓取）需要优化，进腰后出现靠近机器人侧二维码识别不全的问题。

## 已修改

1. 守护进程添加 Ctrl+C 中断时的 CAN 口处理，防止下次运行时无法正常析构。
2. 超时 20 s 回到等待变为直接放置。
