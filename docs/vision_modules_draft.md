# 视觉模块约定（M1–M5）与落地状态

**权威文档。** 业务四模块 = Camera / DL / Marker / Pose；加上编排 = **M1–M5**。  
与代码冲突时以 §0.2「当前运行时」为准；与目标冲突时以 §3–§5 为准。  
目录：`/home/ti5robot/T170Clean` · 更新：2026-10-15

---

## 0. 一页总览（先看这里）

### 0.1 五个层次（编号固定，勿与 §0.3 的 S 步骤号混淆）

| 编号 | 名称 | 职责（目标） | 当前代码落点 |
|:----:|------|--------------|--------------|
| **M1** | Camera | 统一 `Frame`；型号/SN 可换 | `include/vision/camera.h` → `RealSenseCameraBackend`；SN=`realsense_cameras.yaml`；profile=`camera_profile_*.yaml` |
| **M2** | DL | YOLO 多模型；吐有无/数量/框 | `include/vision/dl.h` → `BridgeDlEngine` + `ModelRegistry`（`tray_parts` / `belt_presence`） |
| **M3** | Marker | 只检码 | `include/vision/marker.h` → `BridgeMarkerEngine`（单码）；板级仍经 M4 |
| **M4** | Pose | 手眼、孔网、fuse、占用关联 | `include/vision/pose.h` → `BridgePoseSolver::solve_tray/belt`（内仍调 Bridge） |
| **M5** | 编排 | `Camera→DL+Pose`；唯一胶水 | `include/vision/pipeline.h`；旧 `perceive_*` 已转发 |

Skill（抓放策略/IK/抹角）**不是** M1–M5，仍在 `robot_runtime` / `move_box_runtime`。

### 0.2 当前运行时 vs 目标（交叉核对）

| 主题 | 当前运行时（代码事实） | 目标约定（拍板） | 状态 |
|------|------------------------|------------------|------|
| 料盘定位 XY | ArUco 孔心 | 不变 | ✅ 已符合 |
| 料盘 YOLO | 按 **class** 挂孔（抓 0 / 放空孔 3） | 长期：有无+数量+孔占用；**近期料盘保持 class，不动** | ⏸ 延期（用户要求不影响料盘） |
| 传送带定位 | ArUco | 不变 | ✅ |
| 传送带 YOLO | **ROI 裁切**后计数；抓≥min、放≤max | 有无+数量；区域非全图 | ✅ 已落地（`yolo_roi_*`） |
| 手相机精修 | 可关 | 暂不做 | ✅ |
| `SegPoseBridge` 改名 | 原名 | 不改名，瘦身门面 | ✅ 不改名；业务经 `vision::` |
| M1–M5 拆分落地 | `include/vision/*` 薄封装已接 | 薄封装 → 再搬家 E | ✅ S3–S6；S7 目录大搬家另做 |
| 预热 / YOLO∥码并行 | `warmup` 桩 | 接口稳后做 | ⏳ S8 |
| VisionDebug | `T170_VISION_DEBUG` | 横切 | ✅ |
| 感知收口 C | `perceive_tray/belt` | 编排入口 | ✅ 铺路 |
| Fusion D | `tray_fusion.py` | 归 M4 Pose | ✅ 文件已拆，归属待迁 |
| Detect 算法 2 | 框中心粗点，无伪 mask | DL 的 detect 管线 | ✅ |

### 0.3 落地顺序（唯一编号：S1…）

与 **M1–M5** 分开编号，避免「1–5」歧义。

| 步骤 | 内容 | 状态 |
|:----:|------|:----:|
| S0 | 接口拍板（本文） | ✅ |
| S1 | 传送带 ROI 有无+计数 | ✅ |
| S2 | 料盘改为有无+占用（**明确延期**，不主动做） | ⏸ |
| S3 | **M1** Camera：`ICameraBackend` + SN/profile 配置拆分 | ✅ |
| S4 | **M2** DL：`ModelRegistry` + `model_key` + 统一 `DlResult` | ✅ |
| S5 | **M3+M4** Marker/Pose 切开（孔枚举进 Pose） | ✅ 薄切开（Python 孔网仍在同次 tray 调用） |
| S6 | **M5** `perceive_*` 只调四模块；Bridge 瘦身 | ✅ 业务路径经 `vision::pipeline` |
| S7 | 目录大搬家（旧文件迁入 `src/vision/` 实现体） | ⏳ |
| S8 | 预热、并行 | ⏳ |

**已完成 S3–S6。** 下一步：S7（可选搬家）或 S8（预热/并行）。料盘去 class = S2 仍延期。

---

## 1. 原则（目标）

1. 模块只依赖上游 DTO，不互相 `import` 内部实现。  
2. 调试图只经 `T170_VISION_DEBUG` / `vision_debug`。  
3. Skill 只调 **M5 编排**，不直接碰 Python 检测包。  
4. IK / 抹角 / 开爪 / 导航 ∈ Skill，∉ Pose。  
5. 手相机精修：本期不做。  
6. **传送带** YOLO：只做 ROI 内有无与数量（已落地）。  
7. **料盘** YOLO：运行时仍可按 class；目标文档保留「有无+占用」设计，**实现延期**，改前须再确认。

---

## 2. DTO（目标字段）

### 2.1 `Frame` ← Camera

`slot, width, height, color_bgr, depth_m?, K, dist, stamp?, ok, message`  
现状：`CameraFrameData`（可加 `stamp`）。

### 2.2 `DlResult` ← DL

业务字段：`has_part`, `count`, `detections[]?`（框/粗点）, `model_key?`  
`class_id` 可为模型原始输出；**传送带门禁不读类别名**，只按 `yolo_part_class_ids` 过滤后计数。  
现状皮带：`belt_yolo_count_in_roi` → count（未结构化为 `DlResult`）。

### 2.3 `MarkerResult` ← Marker

检码结果：`used_ids`, 盘/带相机系位姿, `reproj`…  
**不含**孔列表、YOLO 关联（目标）。

### 2.4 `ScenePose` ← Pose

基座系目标：`kind, id, xyz/rpy, occupied?, row/col, …`  
Skill 再做左右手分配与偏置。

---

## 3. 模块接口（目标签名）

### M1 Camera

```text
ICameraBackend: init / flush / grab|grab_wait|grab_fresh / stop
CameraRigConfig: profile + slots[serial, size, fps]
```

配置目标：`camera_profile_*.yaml`（型号）+ `robot_cameras.yaml`（本机 SN；现名 `realsense_cameras.yaml`）。

### M2 DL

```text
IDlEngine: load(registry) / run(Frame, model_key, opts) -> DlResult / warmup?
ModelRegistry: model_key → {type: detect|segment, weights, pipeline}
```

例：`belt_presence` → detect + ROI 在编排或 opts 里裁切后 `run`。

### M3 Marker

```text
IMarkerEngine: load / set_active_board / detect(Frame, board_id) -> MarkerResult
```

### M4 Pose

```text
IPoseSolver: set_extrinsics / solve_tray(...) / solve_belt(...) -> ScenePose
```

孔枚举、fuse、占用关联 ∈ Pose（拍板）。

### M5 编排

```text
perceive_tray / perceive_belt
# 内部：Camera → (DL ∥ Marker) → Pose
```

`SegPoseBridge`：**保留类名**，逐渐只做接线。

---

## 4. 归属表

| 功能 | M1 | M2 | M3 | M4 | Skill | Debug |
|------|:--:|:--:|:--:|:--:|:-----:|:-----:|
| 开流/SN/型号 | ● | | | | | |
| YOLO 推理/注册表 | | ● | | | | |
| 有无/数量 | | ● | | | ○门禁 | |
| 检码 | | | ● | | | |
| 手眼/孔网/fuse/占用 | | | | ● | | |
| 皮带点→基座 | | | | ● | | |
| 抓放门禁、选孔、偏置、IK | | | | | ● | |
| 手相机精修 | | | | | ✗ | |
| picture_debug | | | | | | ● |

---

## 5. 编排时序

### 5.1 料盘（当前运行时）

```text
perceive_tray_head → YOLO(class) + ArUco → 孔挂 class → Skill 按 class 抓/放
定位 XY = ArUco（不变）
```

### 5.2 料盘（目标，延期）

```text
DL → has_part/count + boxes
Marker → 盘姿态
Pose → 孔 + occupied
Skill → 抓 occupied / 放空孔
```

### 5.3 传送带（当前 = 目标，已落地）

```text
同一帧：Marker(码) + 裁切 yolo_roi → DL 计数
Skill：抓 count≥min；放 count≤max；XYZ 仍用码
```

yaml：`conveyor` / `conveyor2` 下 `yolo_roi_enable`, `yolo_roi:{x,y,width,height}`, `yolo_min_conf`, `yolo_part_class_ids`, `yolo_grasp_min_count`, `yolo_place_max_count`。

---

## 6. 配置归属

| 配置 | 模块 | 路径 |
|------|------|------|
| SN ↔ 槽位 | M1 | `config/realsense_cameras.yaml` |
| YOLO 权重/算法 | M2 | `feeding_.../pose_params.yaml` |
| 皮带 ROI/门禁 | M5/Skill 读，推理属 M2 | `move_box_params.yaml` → `conveyor*.yolo_*` |
| 码与孔距 | M3（孔距几何目标归 M4） | `board_*/config.yaml` |
| 手眼 | M4 | `free_calib/...` |
| 偏置/站点/速度 | Skill | `move_box_params.yaml` |
| Debug/计时 | 横切 | `T170_VISION_DEBUG` / `T170_TIMING` |

---

## 7. 拍板记录（§9）

| # | 结论 |
|---|------|
| 孔枚举 | **Pose (M4)** |
| 有无门禁 | DL 出数，Skill 做布尔 |
| 手相机 | **不做** |
| Bridge 改名 | **不改**，瘦身 |
| 料盘去 class | **目标有、实现延期**（保持现状） |
| 传送带 ROI 计数 | **已落地** |

---

## 8. 文档与代码曾出现的矛盾（已消解）

| 矛盾 | 处理 |
|------|------|
| 「料盘也去 class」vs「料盘不要动」 | 目标保留；**实现标为延期**；运行时表写清仍用 class |
| ProblemNote「有无链路未做」vs 皮带已做 | 改为：皮带✅，料盘⏸ |
| 「落地 1–5」指 §8 步骤还是 M1–M5 | **统一：M1–M5 = 模块；S3–S6 = 对应落地步骤** |
| 草案皮带流程写全图 YOLO | 改为 ROI 裁切（与代码一致） |
| `tray_cls` 注册表示例 | 改为 `belt_presence` / 预留 `tray_presence` |

---

## 9. 修订记录

| 日期 | 说明 |
|------|------|
| 2026-10-15 | 初稿 / 拍板 |
| 2026-10-15 | 交叉核对：状态表、M/S 编号、料盘延期、皮带 ROI 对齐代码 |
| 2026-10-15 | S3–S6：`include/vision/*` 薄封装落地；`perceive_*`/皮带 ROI 经模块；构建通过 |
