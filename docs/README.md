# 文档入口

当前代码与配置是事实来源，历史文档仅用于现场溯源。

- **[V1 候选版快速入口](deployment/V1_QUICK_START.md)**：快捷参数、统一应用、新机理论手眼、安装包与维护/生产启动。
- **[运行包升级与 No.120 迁移](deployment/PACKAGE_UPGRADE.md)**：自动备份、保留本机参数、相机方向迁移及抓放参考更新。
- **[同环境新机快速部署与调参](deployment/NEW_ROBOT_DEPLOYMENT_GUIDE.md)**：机型/编号、相机 SN、四份抓放参考及 SDK 安装。
- [仓库目录地图](REPOSITORY_MAP.md)：新工程师快速浏览目录、调用主线和参数入口。
- [仓库和模块指南](architecture/REPOSITORY_GUIDE.md)：目录、调用关系、各模块职责与剩余工作。
- [2026-11-18 仓库复核](validation/repository_review_20261118/RESULTS.md)：关键链路、发现的问题与本轮验证边界。
- [配置与启动](deployment/CONFIGURATION.md)：唯一数据来源、No124 包、运行目录、生产/维护与网页同源。
- [参数档案与重载](deployment/PARAMETER_OPERATIONS.md)：查询参数归属、安全重载及新机器人档案切换。
- [完整执行流水线](architecture/PIPELINE_AND_RUNTIME.md)：取图、C++/Python 桥接、推理并发与 ROS 2 对比。
- [3/5 kg 型号配置](deployment/ARM_MODELS.md)、[新 SDK 接入](deployment/ARM_SDK_MIGRATION.md)：当前模型角、设备绑定和现场验证入口。
- [双臂运动策略](deployment/ARM_MOTION_POLICY.md)：固定关节 Home、自适应肘部、统一限位与反馈日志；[验证记录](validation/arm_motion_policy_20261119/RESULTS.md)。
- [手相机伺服](WRIST_VISUAL_SERVO_ALGORITHM.md)：三种抓放路线、映射/控制与撤离交接。
- [技能接口 v1](architecture/SKILLS_INTERFACE_V1.md)：类型化输入输出、阶段/取消及逐手结果。
- [标定向导](calibration/PLACEMENT_WIZARD.md)、[头变换](calibration/HEAD_TRANSFORM.md)。
- [调度流程](dispatch_flow.md)、[发布验收](architecture/RELEASE_ACCEPTANCE.md)。
- [SDK 升级要求](ARM_SDK_UPGRADE_REQUIREMENTS_20261106.md)、[历史运动栈评估](ARM_KINEMATICS_ASSESSMENT.md)：不是本轮已替换 SDK 的声明。
- [保留现场事实](test_problems.md)、[11 月 9 日记录](field_1109_20261109.md)、[放置误差](tray2_placement_error.md)、[旧问题记录](ProblemNote.md)：原日期/数值保留，不代表问题仍存在。
- docs/validation 下的记录按各自日期/范围解释，不把旧测试通过数量当成本轮结果。
