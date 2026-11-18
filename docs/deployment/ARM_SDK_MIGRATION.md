# T170C V2.0 新运动学 SDK 接入

日期：2026-11-17。代码基线：`fb7feb4`，本次修改包含此前未提交的双机型配置工作。

## 接入范围

正式构建从 `Ti5_Arm_2204.so` 切换为 20261109 交付的 `libti5_robot.so`。旧库原样归档，不再链接。No.124 选择 `t170c_v2_3kg`；5 kg 使用独立型号配置。调度协议及抓放任务顺序未在本次重新设计。

调用链为：

```text
任务/抓放技能/视觉伺服
    → Arm：模型角、连续求解、轨迹策略
        → Ti5_Robotics：新 SDK 正运动学、法兰变换、受检逆解后备
    → arm_io：唯一的正常运动角度转换边界
        → Device SDK：设备角 rad、原有位置控制模式
```

`Arm` 是项目自有适配器，并非旧 `Robot_Arm` 的类型别名。底层实现只在 `vendor_arm.cpp` 中出现，上层不直接依赖供应方类。

## 参数对齐

| 内容 | 来源及处理 |
|---|---|
| 尺寸、轴方向、DH offset、模型限位 | 所选 `robot_models/t170c_v2_3kg/` 或 `t170c_v2_5kg/`；由各自原始 URDF 派生，保留 3 kg 的细微非理想轴结构 |
| 电机编号、设备方向/零偏 | `robots/<robot_id>/arm_binding.yaml`，只在读写电机时换算一次 |
| 实际 TCP | `arm_tools.yaml`；旧法兰坐标声明明确，生成时转到新法兰 |
| 手眼 | 原始文件不改；生成目录中转换法兰坐标，保持 TCP 到相机的相对关系 |
| 准备关节位 | 已由旧设备角转换成模型角，字段仍以度存储，物理电机目标不变 |
| 轨迹速度、加速度、jerk | 复用当前 `retract_motion` 配置，经部署合成写入运行时绑定文件 |
| 采样周期 | 当前播放器固定 5 ms，要求 `_plan.sample_rate=0.005`，不接受悄悄改变时间基准 |

设备转换为 `q_device = direction × q_model + offset`。左手方向为 `[-1,1,1,-1,1,-1,1]`，J2 offset 为 −π/2；右手方向全为 1，J2 offset 为 +π/2。这继承 No.124 已有设备约定，CAD 本身不能证明真实编码器零位。

SDK 的任务基座仍是原有腰部局部运动坐标系，不能额外叠加 URDF 移动底盘 `base_link` 到腰部的平移。URDF 基座、SDK 法兰和腕部 frame 关系保存在 `arm_frames.yaml`。

示意：3 kg 的原始 TCP `[0.260,0,0,0,0,0]` 转到新法兰后，左右手 rx 约为 ±0.002474 rad。这是坐标定义转换，不是重新添加抓取补偿；它也不能替代工具重新标定。

## 求解与轨迹行为

1. 数值逆解以当前模型关节角为初值，采用阻尼最小二乘和步长回退，优先保持当前分支。普通局部求解不锁 J2。
2. 通用单点逆解先尝试局部求解；失败后允许调用供应方逆解，再独立检查返回码、模型限位和 FK 残差。供应方返回 1 不是成功；失败时不将其可能改写的关节值提交给调用方。
3. 历史调用显式传入 J2 时，仍尊重该约束。3 kg 非理想几何使用局部约束求解，不调用未经验证的理想 T7 解析公式。5 kg 理想几何可尝试供应方固定 J2 解，并经过 FK 复核。因此本次不等于“所有历史锁 J2 策略已经移除”。
4. 直线逐点插值 TCP，以前一点的关节解为下一点初值，再使用已有时间参数化满足速度、加速度与 jerk。首点保持原始状态；输出经过限位和直线偏差检查，不用供应方接口的缩线结果替代原始终点。
5. 每只手的 SDK 实例有独立互斥保护；双手不共用一把求解锁。计算性能需由目标机实测，不能以本机参考模型耗时替代。

上述 Arm 逆解/轨迹接口与手相机伺服的 `solve_local_pose()` 是两条求解路径。后者通过新 SDK FK 进行独立局部迭代，其模型限位检查受 `enforce_sdk_joint_limits` 控制，当前默认关闭；不能将 Arm 自身的限位保证扩展到这条路径。两段约束交接需统一，见 [2026-11-18 复核](../validation/repository_review_20261118/RESULTS.md)。

历史展示动作的数据保持设备角，其转换边界在 `diagnostic_gestures.cpp` 内独立声明；正式工厂运动链路统一使用模型角。原有设备控制模式保持位置控制，不启用新的 MIT/力矩接口。

## 旧标定的处理

四份旧料盘参考为双手的抓取、放置记录，原文件均保留。它们包含旧模型下的 TCP 参考、平面/映射信息，却缺少可用于精确重算的原始关节状态。因此模型指纹不匹配时，加载器将记录加入 `pending_model_calibrations`，跳过其几何结果；机器人、相机身份与分辨率仍按原规则核验。

这允许程序加载配置、进入维护及重新标定；不意味着这些任务已经可以自动运行。受影响任务仍依赖有效参考，在新后端下重新保存前不应作为自动化验收结果。不得只更新旧文件的模型指纹来绕过复核。

## Linux 上先运行纯计算检查

以下程序不链接电机、CAN、相机或调度 SDK，不发送运动指令。依赖 C++20 编译器、CMake、yaml-cpp 以及目标架构的交付库。

在机器人仓库根目录执行，`repo`、`runtime` 使用绝对路径；每次生成使用尚不存在的源码树外目录：

```bash
repo="$(pwd)"
runtime="$HOME/t170-sdk-check-$(date +%Y%m%d-%H%M%S)"
python3 "$repo/tools/resolve_deployment.py" \
  "$repo/config/deployments/TH-T170C-0124.yaml" "$runtime"
cmake -S "$repo" -B "$repo/build_sdk_check" -DT170_ARM_SDK_CHECK_ONLY=ON
cmake --build "$repo/build_sdk_check" --parallel 2
cd "$runtime"
"$repo/build_sdk_check/t170_arm_sdk_check" "$runtime/config" \
  > "$runtime/arm_sdk_check.csv" 2> "$runtime/arm_sdk_check.log"
```

检查内容：真实 `.so` 构造及 DH/工具回读；左右臂各 100 个姿态的 FK/法兰与独立 MDH 一致性；固定姿态 ±5 mm 的连续局部逆解、残差与耗时；同位姿轨迹首点；双实例并发 FK 一致性。返回非零时查看两个结果文件。这个检查覆盖计算接口，不覆盖完整任务、碰撞或驱动行为。

通过后回仓库构建完整程序，再用维护模式测试：

```bash
cd "$repo"
cmake -S . -B build_robot
cmake --build build_robot --parallel 2
./deploy/run_maintenance.sh 2>&1 | tee sdk-maintenance.log
```

维护启动会初始化真实硬件。首先核对双手启动行中的机型与 `dh_tool_readback=matched`，静态 TCP/关节反馈，再验证短距离固定姿态移动及已有抓放流程，重新保存受影响的四份任务参考。最后进行调度自动任务回归。

旧构建产物不能靠替换一个 `.so` 原地升级，必须重新 CMake 构建；旧二进制与新类的 ABI 不兼容。启动采用生成运行目录和部署脚本，不能在源码根目录直接启动并假定 SDK 会找到正确的 YAML。

## 日志与排查

- 启动：`[arm_sdk] backend=Ti5_Robotics version=20261109 model=... joints=sdk_model_rad dh_tool_readback=matched`。
- 状态快照：`arm_backend`、`arm_model`、`arm_joint_coordinates`、`pending_model_calibrations`。
- 伺服 CSV：新增末列 `joint_coordinates=sdk_mdh_model_rad`；不能把新旧关节角列直接当作同一坐标比较。
- 运行目录：`Robot_Model.yaml`、`arm_binding.yaml`、`arm_model_metadata.yaml`、`arm_frames.yaml` 和 `deployment_resolved.json` 记录实际输入及来源。

离线检查结果见 [SDK 接入验证记录](../validation/arm_sdk/RESULTS.md)。Linux 动态加载、完整设备链接和实机任务验证尚未在本机执行。
