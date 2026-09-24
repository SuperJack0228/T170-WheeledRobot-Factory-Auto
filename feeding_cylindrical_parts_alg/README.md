# 圆柱物料视觉模块

此目录仅保留 T170C 抓取流程实际调用的代码：

- `algorithm/`：YOLO 实例分割、圆盘 PnP、深度重心与统一引擎；
- `config/pose_params.yaml`：圆柱类别、实物半径和算法参数；
- `models/best.pt`：分割权重；
- `visualization.py`：调试帧的 mask、椭圆与坐标轴叠加；
- `paths.py`：配置和模型的相对路径解析。

运行时由 C++ `SegPoseBridge` 嵌入调用，没有独立相机入口。长度统一使用米，图像误差使用像素。当前只启用模型类别 `0`（`feeding_cylindrical_parts`）。

算法 `0` 使用已知圆柱半径与椭圆轮廓求 PnP；算法 `1` 使用对齐深度图计算 mask 内三维重心。实际抓取选择见根目录 `config/move_box_params.yaml`。
