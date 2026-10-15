# 问题记录

## 待处理

1. 放置和抓取添加视觉识别逻辑，用于分辨传送带上是否能够放置（抓放料的申请已废弃）。
   - **传送带已落地**：ROI 计数门禁 + 调试图画框（`T170_VISION_DEBUG≥1` → `belt_yolo_roi_*.jpg`）。抓：0 不抓报没有料；1 单手（默认右，下料右满则左）；≥2 双手先右。放：`count≤max`。定位仍用码。
   - 料盘：定位仍 ArUco；YOLO 仍按 class 挂孔（去 class / 有无+占用 **延期**）。
2. 整理视觉模块逻辑，添加模型预热，二维码与 YOLO 写成并行模块。
   - **权威约定**：[`docs/vision_modules_draft.md`](docs/vision_modules_draft.md)（含 M1–M5 / S0–S8 状态表）。
   - **已落地**：A/B/C/D/Detect/S1；**M1–M5 薄封装（S3–S6）** → `include/vision/{camera,dl,marker,pose,pipeline}.h`。
   - **尚未做**：目录大搬家（S7）、预热/并行（S8）；料盘有无语义（S2 延期）。
3. 测试各步骤，进行精确时间统计，区分 debug 模式与 release 模式，分开日志与文件写入等对资源占用的影响。
   - 已落地：`T170_TIMING=0|1|2` → `logs/timing_*.jsonl`。
   - 尚未做：debug/release 画像、写调试图单独计时、网页展示。
4. 进腰逻辑（远处三排抓取）需要优化，进腰后出现靠近机器人侧二维码识别不全的问题。

## 已修改

1. 守护进程添加 Ctrl+C 中断时的 CAN 口处理，防止下次运行时无法正常析构。
2. 超时 20 s 回到等待变为直接放置。
