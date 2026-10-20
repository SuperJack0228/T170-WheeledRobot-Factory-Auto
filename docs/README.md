# 文档索引

仓库根目录只留工程 [`README.md`](../README.md)。行为以代码为准；下面按用途分开，避免把现场记录当成当前流程。

## 当前运行时

| 文档 | 看什么 |
|------|--------|
| [`AI_HANDOFF.md`](AI_HANDOFF.md) | 硬约束、站点、调试命令、调度摘要、手眼和偏置。下一任先读这篇。 |
| [`dispatch_flow.md`](dispatch_flow.md) | 调度信号、抓取窗口、放置 ±3 cm、异常码 true/false。比交接文档更细。 |
| [`vision_modules_draft.md`](vision_modules_draft.md) | 视觉 M1–M5 和 S0–S8 的落地状态。 |
| [`ProblemNote.md`](ProblemNote.md) | 还没做完的事，以及已经改过的记录。 |
| [`PROJECT_MAP.md`](PROJECT_MAP.md) | 目录地图。不写进度。 |

传送带抓取：整图检测框中心转到码系，窗口 x∈[-0.05, 0.05] m、y∈[0.10, 0.13] m。0 个不抓，1 个只右手，不少于 2 个双手。手臂仍走 `grasp_offset`。放置看放置点 ±3 cm。窗口在 C++ 里，改 yaml 不会改它。

## 专题说明

| 文档 | 看什么 |
|------|--------|
| [`wave_gesture.md`](wave_gesture.md) | 右手挥手的关键帧和命令。不要把里面的关节角写进抓取 yaml。 |
| [`BEZIER_GRASP_README.md`](BEZIER_GRASP_README.md) | 料盘抓取的二次 Bézier。传送带伸手不走这条曲线。 |

## 记录和未落地的方案

| 文档 | 看什么 |
|------|--------|
| [`tray2_placement_error.md`](tray2_placement_error.md) | 2026-10-09 料盘二放置偏差的现场记录。当天的 `tray2` 不导航；现在的命令顺序看交接文档 §4。 |
| [`tray2_placement_error_prompt.md`](tray2_placement_error_prompt.md) | 给分析者的约束。先读上面那篇记录。 |
| [`fallen_model_update.md`](fallen_model_update.md) | 倒件模型更换方案。还没写进正在跑的类别编号。 |
| [`T170-手相机视觉伺服误差修正技术方案-20261019.md`](T170-手相机视觉伺服误差修正技术方案-20261019.md) | 手相机伺服讨论稿。尚未接入当前抓放。 |
