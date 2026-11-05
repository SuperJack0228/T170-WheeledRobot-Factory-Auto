# 第一步仓库整理报告

日期：2026-11-04。远程 `wrist-servo` 与本地基线均为 `e205e9e49e4efe650a8dd140528ab29add9c12bb`。

**状态：第一步本地实现和检查完成；匹配机器人Linux/ARM64 SDK全量构建及实机验证待现场完成。未实施第二、三步，未发布第一版。**

## 1. 本次范围

将51组迁移表中第一步的23组职责落到目标结构，记录62项原始文件迁移关系。逐文件见 [step1_file_moves.csv](step1_file_moves.csv)，全部步骤见 [模块迁移表](MODULE_MIGRATION.md)。

```text
src/
├── common/          paths、logging、servo_types、servo_trace、task_timing
├── control/         visual_servo：像素映射、PD、制动、下降协调
├── motion/          pose_solver：局部FK位姿求解（其余执行器第二步迁入）
├── hardware/        device_bus、gripper、head、waist
└── vision/
    ├── camera/      RealSense取帧与相机接口适配
    ├── detection/   模型注册与检测接口适配
    ├── geometry/    marker、pose、tray_localizer、head_transform
    ├── tracking/    wrist_observer
    ├── bridge/      Python嵌入、数组转换、可视化、调试开关
    └── pipeline.cpp 感知编排

include/t170/{common,control,motion,hardware,vision}/
libs/lower_body/     原LowerBody库及自检
python/t170_vision/{detection,geometry,calibration,visualization}/
```

后续步骤的空目录按原规划保留 `.gitkeep`，不放假实现、假配置或假标定。已有内容的模块没有新增无用占位文件。

## 2. C++职责与接口

- 相机帧、时间戳、盘面几何、伺服状态和观测等数据类型从大型头文件抽出。原枚举值、字段、默认值及历史命名空间保留，避免本次引入ABI或数据语义改动。
- 纯控制器不再包含相机/邻孔实现头文件。局部求解独立为 motion_math，伺服CSV独立为common下的日志支持。
- 相机/检测/码与位姿/跟踪/桥接按职责归入vision。检测/位姿公开接口通过前置声明引用桥接，不再为了结果类型依赖整个嵌入引擎。
- 观察线程通过 `servo_observer_port.h` 查询实测状态、提交观测；没有电机发送接口或动作执行调用。原公平轮询、时间关联、左右手旋转与跟踪策略不变。
- Python数组转换与可视化已从主桥接实现分离；GUI/存图开关与原逻辑保持。
- CAN设备路由、夹爪、头、腰整块迁移；LowerBody源码、方向、数学和库目标保留。
- 旧头文件仅作为兼容转发。旧运行器/技能/调度大文件只调整必要包含关系，不在此步拆业务。

## 3. Python包

原散目录的算法主体迁入 `t170_vision`，包内使用相对导入。C++明确导入：

- `t170_vision.detection.config_loader` / `engine` / `types`
- `t170_vision.geometry.tray.api`
- `t170_vision.geometry.conveyor.api`
- `t170_vision.geometry.marker.api`
- `t170_vision.visualization.detection`

主程序不再反复向sys.path压入各板目录，不再从不同目录加载同名online模块。模型/配置路径仍指向原实际资源目录；旧Python文件保留薄转发，原命令仍可使用。

新增轻量包导入、跨工作目录导入、原入口模块身份、真实板配置加载、缺权重明确失败及数学自检。没有使用伪模型执行推理，也没有把静态导入当作YOLO实测。

## 4. 构建

根CMake负责机器人环境和应用组装；基础目标收进 `cmake/T170Foundation.cmake`，离线检查收进 `cmake/T170OfflineTests.cmake`。原 `T170_SERVO_TESTS_ONLY` 和 `T170_BUILD_DIAGNOSTICS` 开关保留。

模型登记独立为model_registry，登记元数据读取实际已加载的头/手引擎路径，不再重复硬编码权重名；加载和推理仍使用原配置。

基础目标包括common、model_registry、control、motion_math、servo_trace、tray_geometry、configuration；完整构建分别编译hardware/vision/observer对象模块，再与现有runtime宿主明确聚合。没有新建第二套算法，也没有引入CMake目标环依赖。临时宿主绑定见 [兼容清单](STEP1_COMPATIBILITY.md)。

## 5. 行为和参数核对

[equivalence.json](../validation/step1/equivalence.json)记录相对于e205e9e的只读核对：

- 16份实际配置/标定JSON或YAML字节一致；No.124、偏置、开关和速度没有在迁移中修改。
- 27份C++算法/执行主体token一致（检测登记与配置模型身份接线单独核对）；控制器中时钟和phase名称函数移往common时逐函数另行核对。
- 16份迁移Python实现，在排除导入和明确资源根定位改动后，算法AST一致。
- 31份抽出或保留的数据/控制类型定义token一致。

此核对不能证明完整运行环境或物理精度；桥接拆分和导入改线另由真实头文件语法检查、包导入及回归检查覆盖。

## 6. 检查结果

14项CMake离线检查全部通过：原9项回归完整保留，新增LowerBody自检、公共接口独立编译、Python视觉包、模块边界及模型登记5项。具体过程及范围见 [RESULTS.md](../validation/step1/RESULTS.md)。

兼容旧LowerBody构建入口另行构建并执行自检。支持本机环境的14份动作/视觉/桥接/腰部源码进行了整文件语法检查；Linux CAN驱动、实际RealSense实现及完整机器人链接仍需要真实依赖环境，不能以Mac语法检查替代。

## 7. 本次明确未处理

- 不重调PD、映射、下降/抬升速度、放置锁定高度或各种门限；不改调度通讯和错误恢复。
- 不删除调试命令、不删除旧偏置、不拆No.124参数。它们按上次约定分别在第二步独立归属、第三步核实后清理。
- 不搬权重、音频和第三方SDK，不改标定文件实际加载位置，不宣称完整依赖已打包。
- `function.cpp`、`move_box_runtime.cpp`、`grasp_to_standby.cpp`、`servo/runtime.cpp`、`robot_runtime.cpp` 的职责拆分仍属第二步。
- 不运行机器人，不创建发布包，不自动启动生产任务。

## 8. 后续使用

本次工作目录是最新代码快照 `T170-field-review-20261103`，不是仍有旧规划文件的另一个工作目录。提交/推送应从本目录检查改动；本轮未自动提交或推送。

机器人更新后沿用正式构建和启动入口；CMake将使用新源码布局：

```sh
cmake -S . -B build_robot
cmake --build build_robot -j4
./build_robot/t170_factory 2>&1 | tee startup-step1.log
```

先完成匹配环境全量构建，再按原测试范围验证本次迁移候选。不要将前一版伺服补丁或迁移后的本地回归结果写成已经通过实机验收。
