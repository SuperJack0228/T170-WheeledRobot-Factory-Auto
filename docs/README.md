# 文档

产品介绍在仓库根目录的 [README.md](../README.md)。这里是流程和参数的入口。

| 文档 | 内容 |
|------|------|
| [dispatch_flow.md](dispatch_flow.md) | 上料、转运、下料、充电的信号顺序，抓取窗口和放置 ±3 cm |
| [vision_modules_draft.md](vision_modules_draft.md) | 相机、检测、标志、位姿、编排五层，以及检测和认码如何并行 |
| [AI_HANDOFF.md](AI_HANDOFF.md) | 站点、速度、调试命令、手眼和偏置 |
| [fallen_model_update.md](fallen_model_update.md) | 立件、空孔、倒件的类别，以及空抓后再拍的规则 |
| [BEZIER_GRASP_README.md](BEZIER_GRASP_README.md) | 料盘抓取使用的二次 Bézier |
| [wave_gesture.md](wave_gesture.md) | 挥手动作的关键帧 |
| [PROJECT_MAP.md](PROJECT_MAP.md) | 目录地图 |

传送带抓取看整图检测框在二维码坐标系里的窗口：x ∈ [-0.05, 0.05] m，y ∈ [0.10, 0.13] m。没有件不伸手，一件出右手，两件及以上先右后左。手臂走到 `grasp_offset`。放置看放置点 ±3 cm。窗口写在 C++ 常量里。
