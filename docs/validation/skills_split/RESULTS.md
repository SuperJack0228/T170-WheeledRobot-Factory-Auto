# Skills 拆分验证记录

日期：2026-11-16。平台：macOS / AppleClang；机器人 SDK 最终链接和现场执行未验证。

## 源码与链接符号核对

- 原 move_box_runtime.cpp 的137个辅助函数，133个函数体 C++ token 保持一致；4个增加进度/物理效果报告及阶段取消检查，见 helper_equivalence.json。
- 料盘周期迁移到 cycle.cpp，共用抓取结束阶段迁移到 grasp_finish.cpp，并加入状态记录；它们不属于上述133个未修改函数。
- 分别编译迁移前代码和拆分实现，原150个公开函数/状态符号全部保留，无缺失或重复，见 symbol_equivalence.json。编译器自动产生的内联构造/析构符号不纳入此对比。
- 传送带四段真实新适配代码使用实际项目头文件进行单独语法检查，见 adapter_syntax.json；这不是整个 SDK 相关调度宿主的全量构建。

## 离线构建及回归

```sh
cmake -S . -B /tmp/t170-deployment-build -DT170_SERVO_TESTS_ONLY=ON -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/t170-deployment-build -j 6
PYTHONPATH=/tmp/t170-deployment-deps ctest --test-dir /tmp/t170-deployment-build --output-on-failure
```

本机临时 Python 目录仅提供检查所需 PyYAML，不是机器人部署路径。网页测试需要本机临时监听端口。

- t170_skills_compilecheck 编译全部真实技能实现及对应诊断迁移文件。
- skills_contract_test：资源租约、异臂并行、同臂占用、部分完成、首次失败阶段、取消、回调隔离和终态稳定性。
- skills_contract_json_test：解析真实 C++ 输出，核对版本、JSON转义、字段集合/枚举与schema、名义目标和左右部分松爪。
- conveyor_skill_test：真实接触/撤离实现配合合成设备端口，验证目标和速度沿用、空抓、运动失败不合爪、取消、未确认松爪、持件失败保留及异臂交接。
- tray_skill_service_test：真实标准服务配合合成旧周期，验证左右参数、已持件跳过、数量、部分松爪、返回失败持件、异常后状态和任务模式恢复。
- 原抓取结束阶段回归改为编译新位置的真实实现；公开头文件、边界及配置等已有回归继续保留。
- examples/skills/typed_calls.cpp 通过编译检查，没有可独立执行的 main。

**本轮最终完整 CTest：29/29通过，0失败。** 逐项记录见 ctest_results.json。合成设备不打开 CAN、夹爪串口、相机或真实运动学 SDK，不能替代真机验收。

## 现场出口

1. 使用匹配 SDK 的 Linux/ARM64 环境重新配置并全量编译、链接 t170_factory。
2. 使用相同 No.124 生效配置，验证料盘抓取/放置、两条传送带抓取与放置、右臂撤离和左臂进入的交接。
3. 对照旧运行日志和新 skill_result，特别核对“已持件但返回失败”“只松开一侧”“传送带张爪未确认”“取消后持件状态”。
4. 调度协议和已有任务分支本批保持，正式转用标准 Result 的重试/恢复决策应在后续任务层改造时独立验收。
