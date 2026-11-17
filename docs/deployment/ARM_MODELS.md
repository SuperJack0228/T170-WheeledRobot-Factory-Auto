# T170C V2.0 双臂机型配置

更新：2026-11-17。No.124 经现场负责人确认属于 3 kg 版本。

## 当前交付边界

正式 C++ 构建已切换到 `Ti5_Robotics` / `libti5_robot.so`，运动模块通过项目自有 `Arm` 适配器调用新 SDK。旧 `Ti5_Arm_2204.so` 仅归档在 `dependencies/sdk/arm_legacy/`，不参与构建。运动、轨迹、反馈及伺服统一使用 SDK 模型角，仅硬件边界进行模型角与设备角转换。

运行目录中的 `Robot_Model.yaml` 是实际输入，不再生成旧 `Robot_Arm_Model.yaml`。新适配器启动时比较 SDK 内存中的 DH、工具参数与当前配置，发现不一致即在初始化硬件前报错。本机已完成离线回归及真实调用代码的编译检查；Linux 动态库实际加载、完整机器人链接和实机动作仍需现场验证。接入方法及验证命令见 [SDK 迁移说明](ARM_SDK_MIGRATION.md)。

## 目录与选择

```text
config/robot_models/
  t170/                          现有共用运动/相机参数及旧后端配置
  t170c_v2_3kg/
    model.yaml                   型号、来源哈希、验证状态
    robot.urdf                   用户提供的原始 URDF，字节保留
    Robot_Model.yaml             新 SDK 的完整 MDH、模型角限位
    joint_conventions.yaml       URDF 关节与 SDK 模型角关系
    frames.yaml                  基座、法兰和 URDF 腕/手的固定变换
  t170c_v2_5kg/
    ...                          同一结构
config/robots/TH-T170C-0124/
  arm_tools.yaml                 实例工具 TCP；与臂长分开管理
  arm_binding.yaml               电机编号、方向、零偏和手眼源坐标约定
```

No.124 的 `config/deployments/TH-T170C-0124.yaml` 已选择：

```yaml
robot_model: t170c_v2_3kg
```

部署 5 kg 机器人时，在那台机器人的 deployment 中使用：

```yaml
robot_model: t170c_v2_5kg
```

选择器自动读取对应目录，不需要改 C++ 中的尺寸或分支。每台机器仍使用自己的身份、设备、TCP、示教和相机/任务标定，不能通过复制 No.124 的整套单机参数完成另一台机器的验收。

两个模型在供应方 SDK 中都使用现有键 `T7_170`。适配器构造 `Ti5_Robotics("T7_170", "left/right")`；区别在启动前选好的配置文件内容，不必修改 SDK 源码来添加新机型枚举。

## URDF 对比结果

| 项目 | V2.0 3 kg | V2.0 5 kg |
|---|---:|---:|
| 肩部等效尺寸 | 左右约 ±192 mm | 左右 ±200 mm |
| 上臂等效长度 | 270 mm | 270 mm |
| 前臂等效长度 | 252 mm | 271 mm |
| J6 URDF 范围 | ±1.04 rad | ±1.04 rad |
| 左 J7 URDF 范围 | −2.23～1.57 rad | −1.75～1.57 rad |
| 右 J7 URDF 范围 | −1.57～2.23 rad | −1.57～1.75 rad |
| J1/J2 URDF effort | 66 N·m | 102 N·m |
| J3/J4 URDF effort | 34 N·m | 66 N·m |

effort 和 velocity 是 URDF 中的模型字段，不直接写入驱动器，不作为已确认的任务负载能力或建议运行速度。

3 kg XML 的若干 axis 含约 0.1417° 的倾斜，腕部还有约 0.1 mm 的位移。配置保留了这些细节：不仅改 d1/d3/d5，还派生了相应 DH offset、alpha 和微小 d6/d7。不能把 5 kg 文件简单改成 192/270/252 就宣称与 3 kg URDF 完整一致。

两型号双臂各验证 1003 个姿态，共 4012 个姿态。完整 URDF FK 与派生 MDH 加固定 frame 变换的矩阵元素误差均小于 3e−15。将 3 kg 的细小轴差直接理想化，在这些抽样姿态中会产生最高约 1.4 mm 的腕部位置差，并出现姿态差。

这些是数学一致性结果。新 SDK `.so` 在 Linux 的实际计算、解析 IK、碰撞和实机精度尚未验证。3 kg 的完整 MDH 与理想 T7 几何有细微差异，当前适配器对该几何使用以当前状态为初值的局部数值 IK，不调用未验证的理想 T7 固定 J2 解析公式。直线使用逐点连续局部 IK 和现有时间参数化，不直接使用可能缩短终点的供应方直线接口。

## 角度与工具

SDK 模型角和 URDF 关节角的关系在 `joint_conventions.yaml` 中声明：

```text
q_urdf = direction × q_sdk_model + offset
```

这是两个数学模型的坐标关系，不是从 URDF 自动推导的电机编码器零位。每台机器实际的设备角转换仍需在硬件适配层确认和维护。

`Robot_Model.yaml` 模型模板的 tool 为零，表示 SDK 法兰；部署合成时从 `config/robots/<robot_id>/arm_tools.yaml` 填入每只手的实际工具位姿。因此臂长与安装工具不会因更换机型相互覆盖。

No.124 的工具源值继承旧部署的 `[0.260,0,0,0,0,0]`，源坐标声明为 `legacy_t7_flange_xyz_rpy_m_rad`。生成时利用 `frames.yaml` 将其转换到新法兰；3 kg 左右手 TCP 的 rx 因此分别约为 +0.002474 / −0.002474 rad。转换保持原有工具的物理姿态约定，不是额外调偏置。旧手眼矩阵同步转换至新法兰，原始标定文件保留不变。工具仍标记为 `legacy_tcp_not_revalidated_for_v2`，真实 TCP 精度需要现场复核；5 kg 机器使用自己的工具参数。

## 生成运行配置

仅生成配置，不打开相机、不加载算法库、不访问电机：

```bash
python3 tools/resolve_deployment.py \
  config/deployments/TH-T170C-0124.yaml \
  /absolute/path/to/new-runtime
```

输出目录必须不存在，且位于源码树外。新增资源为：

```text
config/Robot_Model.yaml
config/arm_model_metadata.yaml
config/arm_model.urdf
config/arm_joint_conventions.yaml
config/arm_frames.yaml
config/arm_binding.yaml
```

`deployment_resolved.json` 会记录选中的型号、URDF/SDK 模板哈希、工具来源及准备后配置的哈希。未知机型、混入另一型号的手工 `Robot_Model.yaml` 映射、URDF 来源不一致和缺少实例工具会在生成前报错。

允许直接更改或替换所选型号目录里的 `Robot_Model.yaml`。重新生成运行目录后，修改即进入新 SDK 输入；系统会记录实际文件哈希，并把这套配置标记为 `sdk_yaml_modified_not_equivalence_checked`，不沿用原始派生配置的数学核验结论。仅调整实例 TCP 时修改 `arm_tools.yaml`，不必改臂长模板。URDF 几何本身更新时，按下面的方法重新派生型号配置。

参数变化应生成新运行目录。供应方 SDK 构造时读配置，不要通过进程运行中替换 YAML 假设它会自动切换模型。同一个机器人进程的双臂应属于同一个选定机型。

## 更新 URDF 后重新派生

开发工具使用 Python、NumPy、PyYAML，不依赖 A6 提取源码或供应方动态库。例如生成到新的审阅目录：

```bash
python3 tools/generate_arm_model.py \
  --urdf /absolute/path/to/robot.urdf \
  --model-id t170c_v2_3kg --payload-kg 3 \
  --output /absolute/path/to/new-model-review
```

检查生成差异、数学核验与模型来源后再更新型号目录。该工具限定 T170C V2 双臂结构，不是通用机器人 URDF 转换器；额定 3/5 kg 标签来自机械型号确认，不通过质量或惯量字段猜测。

原始 URDF 含 CAD mesh 引用，本次型号目录只保存几何定义，未复制 STL；不能把该目录直接当成完整 RViz 可视化包。

## 对前次几何评估的更正范围

前次基于 5 kg URDF 得到的 ±200/270/271 mm 参数只用于 5 kg 版本，不能用于已确认是 3 kg 的 No.124。

对同一组 20 个历史状态、保留旧 260 mm 工具比较，3 kg 新旧模型 TCP 差约为 1.2～2.1 mm；5 kg 与旧模型差约为 14.6～21.2 mm。No.124 的不匹配主要是毫米级，不应把此前的厘米级结果套用到它，也不能因此把全部伺服问题归因于臂长。

模型更新会影响旧 FK 解释和标定指纹。No.124 的双手抓取、放置共四份任务参考缺少原始关节状态，无法精确自动重算；当前保留原始记录，加载时标记为 `pending_model_calibrations`，不再使用旧几何参考。需在新后端下重新确认并保存，不能仅改文件指纹让它们重新生效。
