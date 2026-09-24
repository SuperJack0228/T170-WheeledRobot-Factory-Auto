# free_calib

根据 **头部电机当前姿态**（roll / pitch / yaw）和 **已标定的相机→头部末端外参**，计算 **相机系 → base 系** 的 4×4 齐次变换。

本工程 **不做手眼标定**，只负责在标定结果固定后，随头部转动实时更新 `T_base_cam`。

---

## 功能

机器人头部相机安装在头部电机末端。已知：

1. **手眼标定结果** `T_head_cam`：相机系 → 头部电机末端系（4×4，平移单位 mm）
2. **头部几何** `link_length_mm`：头部末端原点在 base 系下的 z 向高度
3. **当前头部角** roll / pitch / yaw（度）

按下列变换链求相机在 base 系下的位姿：

```text
T_base_cam = T_base_head @ T_head_cam
T_base_head = Trans(0, 0, link_length_mm) @ R(roll, pitch, yaw)
```

### 坐标系与旋转约定

| 项目 | 约定 |
|------|------|
| base / 头部 / 相机 | **FLU**（前-左-上，x/y/z） |
| 欧拉角顺序 | 与 `ti5_calib_software` 一致：**R = Rz(yaw) @ Ry(pitch) @ Rx(roll)** |
| 平移单位 | **毫米 (mm)** |

---

## 目录结构

```
free_calib/
├── README.md
├── head_camera_to_base.py          # 主程序 + HeadCameraToBase 类
├── requirements.txt
└── config/
    ├── head_params.yaml            # 连杆长度、标定文件引用
    └── camera_to_head_connector_result.yaml   # 手眼标定 4×4 矩阵
```

---

## 依赖

```bash
pip install -r requirements.txt
```

需要：`numpy`、`PyYAML`。

---

## 配置说明

### `config/head_params.yaml`

```yaml
link_length_mm: 162.0
camera_to_head_yaml: camera_to_head_connector_result.yaml
```

- `link_length_mm`：头部末端在 base 系 z 方向高度（mm），按实物修改
- `camera_to_head_yaml`：相对 `config/` 的手眼标定结果文件名

### `config/camera_to_head_connector_result.yaml`

手眼标定输出，需包含 `a. matrix` 字段（4×4）。示例：

```yaml
a. matrix: [[...], [...], [...], [0, 0, 0, 1]]
b. translation_xyz_mm: [...]
c. euler_angles_xyz_deg: [...]
```

更新标定结果时，替换此文件或修改 `head_params.yaml` 中的引用路径。

---

## 使用方法

### 命令行

工作目录：

```bash
cd /home/wt/my_project/calib/free_calib
```

默认配置 + 零位头部角：

```bash
python head_camera_to_base.py
```

指定头部角（度）：

```bash
python head_camera_to_base.py --roll 0 --pitch -15 --yaw 30
```

JSON 输出（便于脚本对接）：

```bash
python head_camera_to_base.py --roll 0 --pitch -15 --yaw 30 --json
```

覆盖标定文件或连杆长度：

```bash
python head_camera_to_base.py \
  --calib-yaml config/camera_to_head_connector_result.yaml \
  --link-length-mm 162.0 \
  --roll 0 --pitch 0 --yaw 0
```

### 命令行参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `--config` | `config/head_params.yaml` | 头部参数配置 |
| `--calib-yaml` | — | 直接指定手眼标定 yaml，覆盖 config 引用 |
| `--link-length-mm` | — | 直接指定连杆长度，覆盖 config |
| `--roll` / `--pitch` / `--yaw` | 0 | 头部角（度） |
| `--json` | — | JSON 格式输出 |

### Python 调用

```python
from head_camera_to_base import HeadCameraToBase

# 从 config/ 自动加载
calc = HeadCameraToBase.from_config()

# 或手动指定标定矩阵与连杆长度
# calc = HeadCameraToBase.from_yaml("config/camera_to_head_connector_result.yaml", 162.0)

result = calc.compute(roll_deg=0.0, pitch_deg=-15.0, yaw_deg=30.0)

T_base_cam = result.matrix              # 4×4
t_mm = result.translation_xyz_mm      # 平移 [x, y, z] mm
rpy_deg = result.euler_angles_xyz_deg # 欧拉角 [rx, ry, rz] deg
```

---

## 输出示例

文本格式：

```text
a. matrix:
[[...]]
b. translation_xyz_mm: [50.595, 33.654, 97.465]
c. euler_angles_xyz_deg: [-99.316, 0.866, -90.453]
```

`--json` 时输出同结构的 JSON 字典（`matrix`、`translation_xyz_mm`、`euler_angles_xyz_deg`）。

---

## 典型使用场景

1. **头部可动相机**：视觉算法在相机系下得到目标位姿后，用当前头部角算出 base 系位姿，供运动规划使用。
2. **标定结果验证**：固定 `T_head_cam`，改变 roll/pitch/yaw，检查 `T_base_cam` 是否随头部运动合理变化。
3. **与 ti5 标定软件对接**：旋转约定与 `ti5_calib_software` / `HandEyeCalibCommon` 一致，可直接复用手眼标定 yaml。

---

## 注意事项

- `T_head_cam` 必须来自 **相机系 → 头部末端系** 的标定；若标定定义方向相反，需先求逆再填入。
- `link_length_mm` 与 `T_head_cam` 需与 **同一套机械/标定定义** 一致，改实物结构后应同步更新。
- 本工具假设头部原点与 base 的关系仅由 z 向平移 + 三轴旋转描述；更复杂连杆模型需扩展 `HeadCameraToBase.compute()`。
