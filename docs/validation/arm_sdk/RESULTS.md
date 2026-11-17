# 新手臂 SDK 接入验证记录

日期：2026-11-17；基线 `fb7feb4`，分支 `deployment1117`，包括此前双机型配置和本次 SDK 接入的未提交修改。

## 已执行

macOS 本地硬件无关构建目录：`/tmp/t170-sdk-build`。

```bash
cmake -S . -B /tmp/t170-sdk-build -DT170_SERVO_TESTS_ONLY=ON \
  -DPython3_EXECUTABLE=/path/to/python-with-yaml-and-numpy
cmake --build /tmp/t170-sdk-build --parallel 2
ctest --test-dir /tmp/t170-sdk-build \
  -E '^placement_calibration_web_test$' --output-on-failure
ctest --test-dir /tmp/t170-sdk-build \
  -R '^placement_calibration_web_test$' --output-on-failure
```

36 项普通离线测试通过，网页标定测试允许临时监听本机端口后单独通过，共 37 项。网页测试使用模拟机器人接口，不访问 CAN。真实 motion、skills、factory 翻译单元及包含供应方头文件的 `vendor_arm.cpp`、纯计算工具均完成编译检查。

| 验证内容 | 结果与边界 |
|---|---|
| 两机型、双臂 URDF / MDH | 共 4012 个姿态，矩阵元素差 < 3e−15；数学实现独立于新 `.so` |
| 模型角 / 设备角 | 左右臂往返一致；真实 arm_io 与模拟设备接口验证只转换一次、方向及偏移正确 |
| 连续局部逆解与直线 | 3/5 kg 双臂固定姿态 ±5 mm 求解、轨迹原始首点、速度界、同位姿分支保持通过；使用显式参考 FK 后端，不冒充供应方运行结果 |
| 失败隔离 | 模拟供应方返回超限码并改写解时，不提交失败关节值；部分反馈失败不覆盖完整状态，写入失败不更新完整命令缓存 |
| 非理想 3 kg 几何 | 固定 J2 请求使用局部约束求解，不调用理想 T7 原生解析分支 |
| 部署选择与来源 | 3/5 kg 切换、错误型号/重复映射/来源 URDF 哈希拒绝；手改 SDK YAML 可生效但撤销原等价验证状态 |
| 工具与手眼 | 转换前后 TCP 到相机关系保持，准备位转换回设备角与原目标一致；这是坐标代数验证，不是精度重标定 |
| 旧任务参考 | C++ 配置加载将双手抓取、放置四份旧参考标记 pending，不使用其几何结果，原文件不变 |
| 其他回归 | 视觉伺服、料盘邻孔、抓放流程、撤离、配置、模块边界、调度适配及网页回归通过；不表示这些场景已做新 SDK 实机验证 |

随后补充使用实际生成的 260 mm 工具及 No.124 示教准备位验证局部求解和轨迹首点，测试通过；新增模型角绑定、模型元数据和 frame 日志快照，相关回归另行复跑。交付动态库与原始压缩包的字节摘要一致，新旧手臂依赖清单逐文件校验通过。

## 尚未执行

- Linux ARM64 / x86_64 新交付库实际加载、C++ ABI 和内部 DH/FK 的运行核验。
- Linux 真实 SDK 数值求解耗时、双实例并发及完整机器人依赖链接。
- No.124 设备角方向/零位、真实 TCP 与手眼准确度、新参考保存后的单手/双手抓放及调度完整任务。
- 碰撞、动力学、力矩/阻抗控制；本次不接入 A6 专属动力学功能。

新增 `T170_ARM_SDK_CHECK_ONLY` 可以在 Linux 单独构建纯计算检查，完全不链接设备/相机/调度库，命令见 [接入说明](../../deployment/ARM_SDK_MIGRATION.md)。验证通过后再构建完整程序并进行维护模式测试。
