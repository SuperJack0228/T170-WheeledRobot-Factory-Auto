# vision/ — M1–M5 薄封装（S3–S6）

| 头文件 | 模块 | 说明 |
|--------|------|------|
| `types.h` | — | `Frame` = `CameraFrameData` |
| `camera.h` | M1 | `ICameraBackend` / `RealSenseCameraBackend` |
| `dl.h` | M2 | `ModelRegistry`（`tray_parts` / `belt_presence` = best1015，`hand_parts` = best1027）/ `DlResult` / `BridgeDlEngine` |
| `marker.h` | M3 | `BridgeMarkerEngine`（单码 ArUco） |
| `pose.h` | M4 | `BridgePoseSolver`（`solve_tray` / `solve_belt`） |
| `pipeline.h` | M5 | `perceive_tray` / `perceive_belt` |

历史入口 `perceive_tray_head` / `perceive_belt_head`（`seg_pose_bridge.h`）已转发到本目录。  
`SegPoseBridge` 类名保留。目录级搬家（原 E / S7）另做。  
S8：`warmup` 在进程启动跑一次空图；`perceive_tray` 与整图 `perceive_belt` 里 YOLO 和检码并行。
