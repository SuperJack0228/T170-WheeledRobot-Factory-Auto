# 可执行技能接口 v1

日期：2026-11-16。范围：现有 T170 料盘抓放、传送带接触抓放与撤离。此文定义本项目的 C++ 技能接口，不是某个厂商或行业统一协议。

依据试用期目标第3项，交付重点是：按职责划分模块，统一输入输出、状态反馈和异常码，提供接口规范及调用示例，为后续 Agent 工作台编排准备。当前提供进程内调用及 JSON 结果格式，尚未实现 Agent/RPC 服务或远程控制入口。

## 1. 边界和职责

```text
调度/任务：去哪一台机床、抓几件、何时申请/上报、是否补抓/重试
             ↓ request + context + control
skills：完成一段有明确起点/终点和物理效果的操作
             ↓
motion / vision / hardware：规划、执行、纠偏、识别、夹爪反馈
```

skills 不包含调度协议、导航站点控制或充电轮换。现有传送带头相机识别、窗口计数和多手调度先留在任务宿主，识别出的目标通过显式参数传给技能。料盘完整技能包含其自身所需的识别、选孔和腰臂动作。

**机器人与机床不绑定。** Request.metadata.workcell_id 是本次任务的目标，ConveyorContext.station 是宿主根据本次目标选择的工位配置；同一机器人可依次使用不同工位上下文。单机标定、相机和夹爪属于机器人配置，工位几何与策略属于场景配置。本批不新增十台机器人/五组机床的调度分配算法。

## 2. 三类输入

| 类型 | 内容 | 使用规则 |
| --- | --- | --- |
| Request | 目标身份、手选择、目标位姿、已完成状态及少量执行选项 | 数据结构，不含设备指针或执行回调 |
| Context | 已启动的机器人设备引用、相机/视觉、当前工位配置、业务交接 hooks | 由应用组装；技能不自行启动电机、重建相机或修改机器人身份 |
| Control | 取消查询与 progress 回调 | 与业务数据分开；同步调用，不提供线程/进程的隐式后台任务 |

`Metadata`：request_id、workcell_id、target_id。request_id 为空时生成进程内关联号；**关联号不提供去重或幂等执行保证**，不能通过重复提交同一 request_id 假定机器人不会重复动作。

调用期间 Context 引用和配置必须有效，调用者不能同时修改它们。配置仍来自当前配置体系；本批没有把所有历史全局配置读取改成不可变任务快照。

## 3. 五个可执行入口

| 技能名 | C++ 入口 | 主要输入 | 成功结束位置/状态 |
| --- | --- | --- | --- |
| tray.grasp | execute_tray_grasp | 手选择、already_holding、检测引擎、姿态选项、是否允许并行升腰 | 已抓取并完成当前料盘抬升/准备位返回；已持件侧跳过 |
| tray.place | execute_tray_place | 手选择、already_placed、初始持件状态 | 放置、张爪确认并按现有流程撤离；已完成侧跳过 |
| conveyor.grasp_contact | execute_conveyor_grasp | 单手、已解析目标、工位配置 | 下降并完成合爪判定，停在接触高度；尚未收手 |
| conveyor.place_contact | execute_conveyor_place | 单手、已解析目标、初始持件状态、工位配置 | 下降并执行松爪，停在接触高度；尚未收手 |
| conveyor.retreat | execute_conveyor_retreat | 单手、抓后/放后模式、初始持件状态、可选空抓回程关节 | 完成抬升与返回准备位；不会重新抓取或松爪 |

传送带把接触和撤离分开，是为了保留“右手进入回程，左手开始进入”的并行节拍。完整抓取由 contact → retreat 组合；两份 Result 都应保存，不能只用 retreat 的成功掩盖 contact 的失败。

### 料盘参数

- `Hands` 使用具名 left/right，不沿用旧函数参数的 r/l 顺序。`Hand` 的数组下标始终 Left=0、Right=1。
- grasp.already_holding：已持件侧不重新分配、不张爪、不执行本轮动作；结果按 caller_reported 保留持件状态。
- place.already_placed：已放置侧跳过；initial_parts 使用 unknown/empty/holding。显式 empty 且要求执行放置会在动作前拒绝。
- Detector 可用 Default、Head、Hand，与既有模型角色编号对应。不增加新的检测模型或识别策略。
- 料盘技能要求机器人已经到达料盘作业区；它不是“导航到料盘”的任务。

### 传送带参数

- `goal` 为当前手臂基座坐标系，顺序 x/y/z/rx/ry/rz，单位为米和弧度，旋转采用现有 ZYX 欧拉约定。已经解析好的目标不能再次叠加工位 offset。
- 默认 goal 为 NaN；未填写或含非有限值在动作前拒绝，避免默认零点被当作真实目标。
- 两条传送带使用相同技能实现；配置由 Context.station 提供。原有 YOLO、帧数锁定和伺服策略不在本批调参。
- 空抓撤离可使用 retry_ready 保存的伸手前实测电机关节角；普通回程沿用已有固定关节准备位。

## 4. 统一输出 Result

| 字段 | 含义 |
| --- | --- |
| interface_version / skill / metadata | 接口版本、技能类型、本次请求和目标身份 |
| status | running / succeeded / partial / no_target / failed / cancelled / rejected |
| stage / failure_stage | 当前或最后阶段；首次失败发生在哪个阶段，后续阶段不会覆盖它 |
| error_code | 本层稳定错误分类 |
| native_code | 已获得的底层失败码或返回值；未知为 null |
| legacy_status | 旧入口的返回值：料盘为旧状态枚举，传送带为旧整数返回码；不是 normalized error_code |
| hands.left / hands.right | 每手动作、撤离、持件、证据和错误，禁止只看总返回码 |
| elapsed_s | 本次技能调用时间；不是整个调度任务耗时 |
| available_left / available_right | 同轮可分配目标数；null 表示没数清，0 才是已知零个 |
| waist_adjusted | 本次料盘周期是否调整过腰 |

每手输出包含：

- requested、started：是否被请求、是否进入实际动作阶段。
- action_completed：当前原语的动作是否完成；抓放与撤离分开。对 retreat 技能，该字段表示撤离原语本身完成。
- retreat_completed：是否完成本次返回；contact 技能即使抓到件，此值也为 false。
- part_state：unknown、empty、holding。失败不自动清空已确认/已记录的物理效果。
- evidence：unknown、caller_reported、legacy_policy、gripper_feedback。旧逻辑在无有效检测/禁用判空时可能按持件处理，标为 legacy_policy，**不冒充真实持件确认**。
- nominal_target：名义目标及坐标系。料盘输出 upright_arm_base 和 ArUco 行列；传送带输出 arm_base。该值不是最终 TCP 实测位置，也不是伺服修正后的最终像素/坐标；这些仍查原 trace。

例：夹住后收手失败，应是 status=failed、error_code=1301、part_state=holding、action_completed=true、retreat_completed=false。上层应处理收手问题，不应重放一次完整抓取。

no_target 只表示本轮没有完成可执行目标，不单独证明整个料盘已经空了；还必须结合 available_* 是否已知、感知结果和任务上下文判断。partial 表示只完成请求的一部分，`ok()` 为 false。

## 5. 错误码

| error_code | 分类 | 上层应关注的事实 |
| --- | --- | --- |
| 0 | 无错误 | 仍检查 status 和逐手完成状态 |
| 1001 | 输入无效 | 未下发本次技能动作，检查参数/手选择 |
| 1002 | 资源占用 | 同一只手或共享料盘状态已被其他技能占用，不排队、不双写 |
| 1003 | 取消 | 合作式中止，保留已产生的持件/松爪效果 |
| 1101 | 本轮目标不足 | 区分未知数量、已知零个与部分完成 |
| 1102 | 感知/准备失败 | 查看阶段、原始说明及相机日志 |
| 1201 | 空抓 | 未取得目标；是否补抓仍由任务层决定 |
| 1301 | 运动失败 | 保留 native_code，例如 -16；检查持件和撤离状态 |
| 1302 | 伺服/门禁失败 | 保留原运动/伺服返回码 |
| 1401 | 夹爪失败 | 查看单手夹爪状态 |
| 1402 | 松爪未确认 | 不能把未知状态记成已放置成功 |
| 1501 | 设备/通信故障 | 处理设备状态，不自动重放动作 |
| 1901 | 执行或关键回调异常 | 已发生的动作仍保留；不得假定“抛异常就没动过” |

归一化码只用于本项目技能层，**没有改写调度现有协议码**。诊断优先查看 error_code + failure_stage + native_code + 每手 part_state。

现有传送带松爪逻辑在等待未确认时仍可能返回旧码0并清旧闩锁。本批保留其旧调度兼容行为，但新 Result 明确报告1402及 unknown。旧任务层暂不因此改变分支；后续任务迁移时再统一采用标准结果。料盘原有严格张爪确认逻辑保持。

## 6. 状态反馈、并行和取消

- Control.progress 提供 request_id、skill、stage、所涉手、递增 sequence、elapsed_s；回调串行执行，应快速返回。观察回调异常隔离，不能使正在运行的动作突然退出。
- 取消在既有阶段检查及新技能边界处理；不承诺中断 SDK 内部阻塞调用，也不替代硬件急停。
- 每手技能使用资源租约；左右手 conveyor contact/retreat 可并行，同一只手重复请求返回 busy。料盘完整技能还占用双臂及共享腰/选孔状态。
- 租约约束通过新接口执行的技能。历史原始运动/诊断命令尚未全部纳入同一资源仲裁，不能宣称所有命令已实现全局互斥。
- hooks 是原有业务交接点：抬离后通知、升腰结束、开始返回等。它们属于 Context，Agent 输入数据不携带任意回调。

## 7. 调用示例及日志

可编译示例：`examples/skills/typed_calls.cpp`。它没有 main，也不自行启动设备，离线构建会编译校验。

```cpp
t170::skills::TrayGraspRequest request;
request.metadata = {"mission42/pick1", "machine-group-3", "input-tray"};
request.hands = {true, true};
auto result = t170::skills::execute_tray_grasp(context, request, control);
if (!result.ok()) {
    // Preserve result.hands first; decide recovery in the task layer.
}
```

`to_json(result)` 输出与 `config/schemas/skill_result.v1.schema.json` 对应。旧业务适配器将结果打印为 `[skill_result] {JSON}`，原伺服 CSV、调度统计与错误码继续保留。接口的结果 JSON 不意味着已经开放了 JSON 请求服务。

## 8. 验证范围和剩余工作

本批完成技能实现归位、五个类型化入口、统一结果/进度/错误分类、结果 schema、兼容适配及离线回归。外部 SDK 的最终链接与真机动作尚未验收。

剩余边界：料盘共用 cycle 仍承接有效远近排/错位策略；旧精度实验/手动循环与转发头已删除。任务、协议、底盘和应用已继续拆分，参见 REPOSITORY_GUIDE.md。内部运行结构、全局状态与全局配置仍有过渡绑定；不可变任务参数快照及正式安装发布仍未完成。
