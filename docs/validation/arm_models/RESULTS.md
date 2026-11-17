# V2.0 3 kg / 5 kg 型号配置验证

日期：2026-11-17；修改基线 `fb7feb4`，分支 `deployment1117`。

本页记录首次型号配置准备时的验证，不代表当前接入状态。随后已接入新 SDK，最新变更及验证见 [SDK 接入结果](../arm_sdk/RESULTS.md)；下文“后端未切换”等表述仅属于此前阶段。

## 已完成

- 复制用户提供的两份 URDF 原始 XML，哈希在对应 `config/robot_models/<model_id>/model.yaml` 中记录。
- 3 kg / 5 kg 各双臂各 1003 个姿态，共 4012 个姿态，完整 MDH 加固定 frame 与原始 URDF FK 的矩阵元素误差小于 3e−15。
- 3 kg 的倾斜轴、约 0.1 mm 腕偏移及 J7 不同限位均保留；不能仅替换三段长度。
- 部署生成测试覆盖：通过 `robot_model` 切换 3/5 kg、加载实例 TCP、保持旧后端配置不变、错误型号/跨型号手工映射/URDF 版本不符拒绝、手工编辑 SDK YAML 后实际加载且不沿用原几何核验状态。
- 新配置不启动设备，不打开相机，不加载算法库。
- 当前正式 C++ 后端、运动学角度契约和旧任务标定均未切换。No.124 的目标机型登记为 `t170c_v2_3kg`，启动前会准备对应新 SDK 配置。

## 构建与回归

硬件无关构建目录：`/tmp/t170-dependency-build`。

```bash
cmake -S . -B /tmp/t170-dependency-build -DT170_SERVO_TESTS_ONLY=ON \
  -DPython3_EXECUTABLE=/path/to/python-with-yaml-and-numpy
cmake --build /tmp/t170-dependency-build --parallel 2
ctest --test-dir /tmp/t170-dependency-build --output-on-failure
```

本机离线构建及真实 motion/skills/factory 翻译单元编译检查通过。35 个 CTest 项目均已通过：其中 34 项在普通沙箱通过；HTTP 标定测试首次因沙箱禁止监听 127.0.0.1 失败，允许临时本地监听后单独复跑通过。该测试使用模拟机器人传输，无设备访问。

后续外置 YAML 编辑支持变更后，单独复跑 deployment 与模型几何测试，两项通过。

## 尚未验证

- 新 Linux `.so` 的实际加载、T7 正逆解及直线轨迹性能。
- 3 kg 的精确 MDH 是否受供应方固定 J2 解析/直线接口完整支持。
- No.124 在新法兰定义下的真实 TCP、设备零位/方向与任务标定。
- 单手、双手和完整工厂任务运行。

完整使用说明见 `docs/deployment/ARM_MODELS.md`。这份结果不作为新 SDK 真机验收报告。
