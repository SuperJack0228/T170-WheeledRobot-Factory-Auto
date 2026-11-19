# 运动职责拆分检查记录

日期：2026-11-16；平台：macOS / AppleClang。机器人 SDK 最终链接及实机动作尚未验证。

## 行为保持

拆分前两个文件与 Git HEAD `9e023e775ecfb2d2742382f5a92427fc4f1e2907` 的原文件逐字节一致。迁移后 **179 个函数体全部对应且 C++ token 一致**，包括字符串、数值与运算符；比较仅忽略注释、空白，允许文件和私有命名空间位置变化。

逐函数目的路径与哈希见 `body_equivalence.json`。另将原文件与拆分实现分别编译，比对得到原有 **110 个公开函数符号全部保留，没有缺失或重复定义**，见 `symbol_equivalence.json`。这证明函数体迁移没有顺手改写算法，不证明线程时序或真实机器人效果完全等价。

## 构建与测试

```sh
cmake -S . -B /tmp/t170-deployment-build -DT170_SERVO_TESTS_ONLY=ON -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/t170-deployment-build -j 6
PYTHONPATH=/tmp/t170-deployment-deps ctest --test-dir /tmp/t170-deployment-build --output-on-failure
```

本机临时 Python 依赖目录提供 PyYAML；部署环境应使用已安装依赖的正式 Python 环境。网页测试需要本机临时监听端口。

- `t170_motion_compilecheck` 编译真实的八个手臂职责实现及应用演示实现，不链接伪造 SDK。
- 合成双臂 runtime 回归从拆分后的七个实际伺服实现文件编译和链接，覆盖已有下降、左右手隔离及保位交接行为。其关节/运动学设备是显式合成模型，不是 T170 真机。
- 新增 pose_math_test 验证独立数学库的旋转约定、变换往返、退化矩阵伪逆和插值边界。
- 扩展模块边界回归，检查公开头文件不带 SDK/app/全局命名空间污染、共享状态唯一、兼容实现不被重复编译。
- 扩展 SDK-free 公开头文件编译覆盖。
- 旧抓取/标定/目标计算调用者、arm_plane_repeat 和兼容入口的单独语法检查见 `syntax_checks.json`。

**本轮最终结果：25/25 通过，0 失败。** 逐项结果见 `ctest_results.json`。本机 SDK 适配层编译检查通过；机器人 SDK 的全量链接尚未进行。

## 外部验收缺口

1. 在机器人匹配的 Linux/ARM64 SDK 环境重新配置并全量链接 t170_factory。
2. 使用同一份 No.124 生效配置，复核双手料盘抓取、放置、两条传送带抓取、保位到抬升/返回的交接以及中止路径。
3. 保存现场构建版本、配置快照与运行日志。通过后再将本批标为现场验证完成。
