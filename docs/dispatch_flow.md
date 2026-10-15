# 调度模式：信号与动作流程

以当前代码为准（`orchestrator/cpp/robot_runtime.cpp`，库 `RobotClient` 1.8.0）。  
申请超时 **20 s**。任务结束后原地再听 **20 s**。二维码失败最多再试 **3 次**（不含第一次）。

调度靠每 2 秒的位姿报文看位置：电量、x、y、角度、`current_point`（底盘当前站名）。

---

## 1. 谁发给谁

### 1.1 调度下发，机器人收到后才动

| 收到 | 含义 | 机器人做什么 |
|------|------|----------------|
| `TASK_LOAD` | 上料 | 料架抓毛坯 → 上料点放置 |
| `TASK_TRANSFER` | 转运 | 传送带一抓半成品 → 传送带二放置 |
| `TASK_UNLOAD` | 下料 | 传送带二抓成品 → 成品盘放置 |
| `CHARGE` | 充电 | 回 home → 去充电点充到阈值 |
| `PLACE` | 允许放置 | 结束对应的放置申请，开始放 |
| `PICK_HALF` | 允许抓半成品 | 结束转运抓取申请，开始抓 |
| `PICK_WELL` | 允许抓成品 | 结束下料抓取申请，开始抓 |

任务报文里的点位才覆盖本机站点，收到新值才改，任务结束不清：

| 字段 | 用途 |
|------|------|
| `raw_material_point` | 上料料架 |
| `sequence_one` | 上料点 / 转运抓取点（传送带一） |
| `sequence_two` | 下料点 / 转运放置点（传送带二） |
| `well_done_point` | 成品盘 |
| `standby_point` | 无任务时的待命点；没有则用 yaml `dispatch_wait_station`（AP14） |
| `charge_point` | 充电点；没有则用 yaml `chassis.charge_station` |

四个工作点都空时，`apply_mission` 会先向调度要站点（`NEED_POINTS`），再等任务。未知任务字符串不执行，回 home → AP14。

### 1.2 机器人发给调度

| 发出 | 何时 |
|------|------|
| `APPLY_PLACE_RAW` | 上料：料盘抓完、底盘要去传送带一时，申请放置毛坯。等到 `PLACE` 或 20 s 超时 |
| `APPLY_PICK_HALF` | 转运：要在传送带一抓。等到 `PICK_HALF` 或超时 |
| `APPLY_PLACE_HALF` | 转运：去传送带二的路上申请放置。等到 `PLACE` 或超时 |
| `APPLY_PICK_WELL` | 下料：要在传送带二抓。等到 `PICK_WELL` 或超时 |
| `PICK_UP_DONE` | 这次抓取结束（抓到、抓空、或申请超时没抓） |
| `PLACE_DONE` | 这次放置结束 |
| `DONE` | 整个任务完成（充电达到阈值也发这条） |
| `RAW_MATERIAL_EMPTY` | 上料开抓前数到的毛坯这次被拿光 |
| `WELL_DONE_MATERIAL_FULL` | 下料放下后成品盘左右都没有空孔 |

申请超时：**抓取**不抓这只手，仍发 `PICK_UP_DONE`，任务继续。**放置**不再等，直接放。

### 1.3 只记在本机面板，不发给调度

`STANDBY`、`WORKING`、`GO_CHARGE`、`commuting`、`CHARGING`，以及 `ERR_MOTOR_LOST`、`ERR_MOTOR_DISABLE`、`ERR_GRIPPER_LOST`、`ERR_CAMERA`、`ERR_AGV_OBSTACLE`。

电机丢失、电机失能、夹爪丢失会停在原地等系统处理。其它失败回 home → AP14。二维码失败只在本机记 `ERR_CAMERA` 一下，然后重新申请并重拍，不把任务判死。

---

## 2. 视觉怎么决定动作

### 2.1 传送带检测（调度）

整张头图跑 YOLO，检测框画在 `picture_debug/head/belt_yolo_det_*.jpg`。每个工件中心用深度变到**二维码坐标系**。不再用 ROI 裁切计数。

抓取：左右手各自的对照点与零件做最优一一匹配。对照点 = `grasp_offset`，X 再后移 **3 cm**（零件相对抓取点统一偏后），Y 不变。门限 **±2 cm**（X、Y 都要在范围内）。手臂仍走到 `grasp_offset`，不跟着对照点后移。转运不再额外加 +5 mm。

| 匹配结果 | 动作 | 上报 |
|----------|------|------|
| 左右都没有 | 不伸手，原地等下一条 | `1102` UnloadPointEmpty |
| 只有一只手的点附近有料 | 只伸那只手。合爪后 **2 s** 底盘出发。这只手收回后发 `PICK_UP_DONE` | `PICK_UP_DONE` |
| 两只手的点附近都有料 | 只申请一次。右手合爪成功后左手马上抓，左手不再申请。底盘等**最后合爪的那只手**后再过 **2 s** 出发。左手收回后发 `PICK_UP_DONE`，不等右手收完、也不等底盘到站 | `PICK_UP_DONE` |

空抓不重试，发 `PICK_UP_DONE` 后按手里实际有没有件继续。

### 2.2 传送带放置（转运用）

这次要放的手，其放置点（码系 `offset`）与零件做最优匹配，门限 **±3 cm**。一个零件都配不上则计数为 0，允许放。配上了就不放，上报 `1101` LoadPortBlocked，手里有件原地待命。

### 2.3 料盘

上料抓取：ArUco 定孔，YOLO 按 class 0（有料）选孔。抓完前看到的件数这次被拿光，就发 `RAW_MATERIAL_EMPTY`。

下料放置：ArUco 定孔，YOLO 按空孔放到成品盘。放下后左右空孔都是 0，发 `WELL_DONE_MATERIAL_FULL`。

---

## 3. 总循环

1. 启动：home，去待命点（`standby_point` 或 AP14），以 home 姿态等任务。
2. 收到四个任务之一就执行。
3. 成功：发过该任务自己的 `DONE` 后，原地再听 20 s。20 s 内有下一条就连做，否则回 home → AP14。
4. 异常码发给调度后停住，等网页点已解决或未解决，**不回 AP14**：
   - `1101` 放置点 ±3 cm 内有零件，左右手都不放
   - `1102` 左右对照点（X 后移 3 cm）±2 cm 都没有零件
   - `1201` 二维码解失败
   - 返回 true：不再申请，原地再拍一次，重做刚才的抓或放
   - 返回 false：停在当前站等下一条任务
5. 手里还有件的失败同样停在原地，不报完成，不回料架再抓。
6. 充电结束停在充电等待点（默认 AP12），不断开后改去 AP14。

---

## 4. 上料 `TASK_LOAD`

1. 去料架，头相机认料盘，按有料孔抓（可两只手）。抓取收手时底盘已经朝上料点走，同时发 `APPLY_PLACE_RAW`，等 `PLACE`。
2. 若开抓前的件数这次拿完，发 `RAW_MATERIAL_EMPTY`。
3. 两手都没抓到：任务失败返回，不去放。
4. 到传送带一。申请已允许就停稳后拍照放；超时则直接放。放的是传送带一，右半框必须空。
5. 最后一手收回准备位后发 `PLACE_DONE` 和 `DONE`。停在传送带一。

料盘二维码连续失败：重试最多 3 次，仍失败则任务失败，回 AP14。

---

## 5. 转运 `TASK_TRANSFER`

放置松手在基座 Z 再低 **5 mm**。抓取点就是 `conveyor.grasp_offset`，不再另加 +5 mm。

1. 去传送带一，摆到拍照姿态。
2. 数左半框（见 §2.1）。0 个就停在这里等指令。
3. 1 个或 ≥2 个：发 `APPLY_PICK_HALF`，等到 `PICK_HALF`（超时则不抓，发 `PICK_UP_DONE`，继续）。
4. 按 §2.1 伸手。抓取准备姿态用 `conveyor.tcp`，抓取点用码 + `grasp_offset`，抓完收回 `conveyor.tcp`。
5. 空抓不重试。最后合爪的那只手闭爪 **2 s** 后底盘去传送带二。单手在该手收回后发 `PICK_UP_DONE`；双手在左手收回后发，不等底盘到站。
6. 手里没有件：跳过放置，仍发 `DONE`。
7. 手里有件：去传送带二的同时发 `APPLY_PLACE_HALF`，等 `PLACE`。超时直接放。
8. 传送带二右半框为空才放。放完发 `PLACE_DONE`。随后发 `DONE`。停在传送带二。

传送带二维码连续失败：重新申请并重拍，最多 3 次。手里有件仍解不出码则原地待命，不发 `DONE`。

---

## 6. 下料 `TASK_UNLOAD`

抓取规则与转运相同（§2.1），站点换成传送带二，参数用 `conveyor2`。抓完收回的是料盘放置准备 `standby`，不是 `grasp_tcp`。

1. 去传送带二，摆到拍照姿态。
2. 左半框 0 个：不抓，报没有料，原地等指令。
3. 有件：发 `APPLY_PICK_WELL`，等到 `PICK_WELL`。超时不抓，发 `PICK_UP_DONE`，继续。
4. 按数量伸右手或双手。最后合爪的手闭爪 **2 s** 后底盘去成品盘。
5. 手里没有件：不去成品盘，仍发 `DONE`。
6. 手里有件：成品盘真放置（张爪）。空孔用料盘 ArUco + 空孔类别。
7. 放下后若左右空孔都没有，发 `WELL_DONE_MATERIAL_FULL`。
8. 发 `DONE`。停在成品盘。

料盘二维码连续失败且手里有件：原地待命，不发 `DONE`。

---

## 7. 充电 `CHARGE`

1. 本机记下准备去充电。整机 home，去待命点。
2. 手臂下垂，腰降到抓取高度，失能腰、臂、头（右臂锁定则跳过右臂）。
3. 底盘去充电点。
4. 电量达到 `charge_done_percent`（可热改）后发 `DONE`，断开充电继电器。
5. 重新使能。手臂回 `home_tcp`，腰高不动，底盘去充电等待点（默认 AP12）等下一条指令。

---

## 8. 信号对照（一张表）

| 步骤 | 机器人发出 | 调度应回 | 超时 20 s |
|------|------------|----------|-----------|
| 要任务 | 申请任务（点位空则先要站点） | `TASK_LOAD` / `TASK_TRANSFER` / `TASK_UNLOAD` / `CHARGE` | 一直等到任务或断线 |
| 上料放 | `APPLY_PLACE_RAW` | `PLACE` | 直接放 |
| 转运抓 | `APPLY_PICK_HALF` | `PICK_HALF` | 不抓，`PICK_UP_DONE`，继续 |
| 转运放 | `APPLY_PLACE_HALF` | `PLACE` | 直接放 |
| 下料抓 | `APPLY_PICK_WELL` | `PICK_WELL` | 不抓，`PICK_UP_DONE`，继续 |
| 抓完 | `PICK_UP_DONE` | 应答 | 库内重发 |
| 放完 | `PLACE_DONE` | 应答 | 库内重发 |
| 任务完 | `DONE` | 应答 | 库内重发 |
| 料架空 | `RAW_MATERIAL_EMPTY` | 应答 | 未连接先缓存 |
| 成品盘满 | `WELL_DONE_MATERIAL_FULL` | 应答 | 未连接先缓存 |
