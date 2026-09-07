# seg_circle_pose

基于 YOLO 分割的圆盘 6D 位姿库，**相机与算法解耦**，便于集成到其他系统。

**长度单位统一为米 (m)**。参数集中在 **`config/pose_params.yaml`**（分割、按类别半径、算法 0/1 分区配置）。

## 目录

```
seg_circle_pose/
  config/pose_params.yaml   # 统一参数（必改）
  main.py
  models/best.pt
  algorithm/
    config_loader.py        # 加载 YAML
    engine.py
    types.py
    pose_pnp.py             # algorithm_id = 0
    pose_centroid.py        # algorithm_id = 1
  camera/realsense_d435.py
  visualization.py
```

---

## 统一参数文件 `config/pose_params.yaml`

两种算法共用一份配置，按区块划分：

| 区块 | 作用 |
|------|------|
| `algorithm` | **位姿算法**全局默认 `default_id`：`0`=PnP，`1`=重心（**默认 0**） |
| `segmentation` | YOLO 模型路径、`conf`/`iou`、可选 `class_ids` 过滤 |
| `defaults` | 未单独列出的类别默认 `radius_m`、`enabled`、`algorithm_id` |
| `classes` | **按类别名**配置：`class_id`、`radius_m`、`algorithm_id` 等 |
| `algorithm_pnp` | 算法 0 全局 PnP 参数 |
| `algorithm_centroid` | 算法 1 全局重心参数 |

### 类别配置示例（当前仅 1 类，可扩展）

```yaml
algorithm:
  default_id: 0            # 全局默认位姿算法

classes:
  feeding_cylindrical_parts:
    class_id: 0              # 须与 YOLO names 序号一致
    radius_m: 0.0275         # 该类圆盘半径 (m)
    algorithm_id: 0          # 该类使用的位姿算法，可覆盖 default_id
    enabled: true
    # 可选：仅覆盖该类
    # pnp:
    #   max_reproj_mean: 12.0
    # centroid:
    #   min_depth_m: 0.04
```

新增类别：在 `classes` 下增加一项，填写不同的 `class_id` 与 `radius_m`。

未在 `classes` 中登记的 `class_id`：使用 `defaults.radius_m`，`class_name` 输出为 `class_{id}`。

---

## 输入输出接口（核心）

### 调用流程

```text
load_pose_params("config/pose_params.yaml")
        ↓
CirclePoseEngine(params)     # 初始化一次
        ↓
engine.run(AlgorithmInput)   # 每帧
        ↓
AlgorithmOutput.targets[] → TargetPose
```

### 算法类型 ID

| 常量 | 值 | 含义 |
|------|-----|------|
| `ALGO_PNP` | `0` | 分割 + 椭圆 PnP（`depth` 可 `None`） |
| `ALGO_CENTROID` | `1` | 分割 + mask 深度重心（**必须** `depth`，单位 m） |

---

### 初始化

```python
from algorithm.config_loader import load_pose_params
from algorithm.engine import CirclePoseEngine
from algorithm.types import AlgorithmInput, ALGO_PNP

params = load_pose_params()  # 默认 config/pose_params.yaml
engine = CirclePoseEngine(params)
# 或: CirclePoseEngine("config/pose_params.yaml")
```

**`PoseParams` 主要字段**（由 YAML 解析，一般无需代码里逐项填写）：

- `params.segmentation` — 模型路径、`conf`、`iou`、`class_ids`
- `params.classes_by_id` / `classes_by_name` — 每类 `radius_m`、`enabled`、可选 `pnp`/`centroid` 覆盖
- `params.pnp` / `params.centroid` — 算法 0/1 全局默认
- `params.radius_m_for(class_id)` / `params.pnp_for(class_id)` — 运行时按类取参

---

### 每帧输入：`AlgorithmInput`

| 字段 | 类型 | 说明 |
|------|------|------|
| `rgb` | `(H,W,3)` BGR `uint8` | 彩色图 |
| `depth` | `(H,W)` `float32` **m** 或 `None` | 算法 1 必填，与 rgb 对齐 |
| `K` | `(3,3)` | 内参 (px) |
| `dist_coeffs` | `(5,)`+ | 畸变 |
| `algorithm_id` | `0` / `1` / `None` | `None` 用配置；非 `None` 时整帧覆盖配置 |

---

### 每帧输出：`TargetPose`

| 字段 | 说明 |
|------|------|
| `pose_4x4` | 相机系 **T_cam_obj**，平移 **m** |
| `class_id` | YOLO 类别 ID |
| `class_name` | 配置中的类别名，如 `feeding_cylindrical_parts` |
| `algorithm_id` | 该目标实际使用的位姿算法 `0` / `1` |
| `radius_m` | 该目标使用的半径 **m**（来自 `classes`） |
| `confidence` | 检测置信度 |
| `success` / `message` | 是否成功 / 失败原因 |
| `mask` / `ellipse` | 可视化辅助 |

失败时勿直接使用 `pose_4x4`，需检查 `success`。

```python
out = engine.run(AlgorithmInput(rgb=bgr, depth=None, K=K, dist_coeffs=dist))  # algorithm_id 默认读配置
for t in out.targets:
    if not t.success:
        continue
    T = t.pose_4x4
    print(t.class_name, t.algorithm_id, t.radius_m, T[:3, 3])
```

---

## 单位约定

| 量 | 单位 |
|----|------|
| `depth`、`pose_4x4` 平移、`classes.*.radius_m` | **m** |
| `K`、`dist_coeffs` | px / OpenCV 畸变 |
| `algorithm_pnp.reproj_thresh` 等 | **px** |

---

## 依赖

```bash
pip install numpy opencv-python ultralytics pyyaml
pip install pyrealsense2   # 仅 main.py 演示
```

## 运行演示

```bash
cd factory/seg_circle_pose
python main.py
# 默认使用配置 algorithm.default_id: 0
python main.py --algorithm-id 1   # 命令行覆盖整帧算法（半径仍读 yaml）
```

| 按键 | 作用 |
|------|------|
| `q` / ESC | 退出 |
| `s` | 保存 json + 可视化（`--save-dir`） |
| `x` | 算法 0：切换 XYZ 轴显示 |

演示绘制：mask、位置；算法 0 另绘拟合椭圆、投影圆、法向轴（每目标使用各自 `radius_m`）。
