# 圆柱物料视觉模块

此目录仅保留 T170C 抓取流程实际调用的代码：

- `algorithm/`：YOLO 检测（主）/分割、框中心粗点、PnP、mask 重心与统一引擎；
- `config/pose_params.yaml`：圆柱类别、实物半径和算法参数；
- `models/best1015.pt`：检测权重（当前）；
- `visualization.py`：调试帧的框、mask、椭圆与坐标轴叠加；
- `paths.py`：配置和模型的相对路径解析。

运行时由 C++ `SegPoseBridge` 嵌入调用，没有独立相机入口。长度统一使用米。

算法：`0` 分割 PnP；`1` mask 深度重心；`2` 检测框中心+深度（关联用，detect 权重强制）。料盘抓放 XY 仍来自 ArUco。见根目录 `config/move_box_params.yaml` 的 `vision_detect`。
