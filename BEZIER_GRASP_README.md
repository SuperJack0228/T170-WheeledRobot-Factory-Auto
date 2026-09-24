# 机械臂抓取 Bézier 轨迹改造说明

## 修改目的

本次修改废除了原有的“折线 + 途经点局部圆角”轨迹。旧方案实机运动效果不稳定，不能形成预期的连续抓取圆弧。

新的抓取流程使用一条二次 Bézier 曲线，使机械臂从待机位置连续运动到最终抓取点，并在靠近物料时自然转为竖直下降。目标是在保证运动连续、逆解稳定的同时，让最终落点 C 保持明确且可校准。

## ABC 点定义

- A：开始运动时，根据机械臂编码器和正运动学得到的实际 TCP 位姿。
- B：曲线控制点，暂定为最终点 C 正上方 12 cm，即 `B=(C.x, C.y, C.z+0.12)`。
- C：最终抓取位姿，由二维码孔位、盘面高度和 XYZ offset 共同计算得到。

曲线公式：

```text
P(u) = (1-u)^2 A + 2(1-u)u B + u^2 C，u∈[0,1]
```

B 只负责引导曲线形状，机械臂不需要经过 B。由于 B 和 C 的 XY 相同，曲线到达 C 前的末端方向为竖直向下。

## 当前执行流程

```text
头相机二维码定位 + YOLO类别判断
→ 计算最终抓取点C
→ 读取实际起点A
→ 生成C上方12 cm的控制点B
→ 完整规划A-B-C Bézier轨迹
→ 全轨迹IK、限位、FK和关节连续性检查
→ 一次性执行到C
→ 编码器FK复核C点
→ 稳定等待0.5秒
→ 夹爪合拢
→ 倒放原关节轨迹返回A
```

手相机当前关闭，因此不会再执行“先到 hover 点，再直线下压”的旧流程。

## C 点精度保护

- 最后一个笛卡尔轨迹样本直接使用 C，不进行近似或高度钳位。
- 整条轨迹必须全部求解成功后才会开始运动。
- 规划时检查关节限位、相邻关节跳变以及 IK/FK一致性。
- 到达C后根据编码器重新计算实际TCP。
- 实际TCP需要连续3次进入误差范围，才允许合爪。
- 默认XYZ到位门限为3 mm；超差时禁止合爪。

这使后续XYZ offset调试能够主要反映视觉标定和机械安装偏差，而不是轨迹没有真正到达C造成的误差。

## 可调参数

参数位于 `config/move_box_params.yaml`：

```yaml
head_grasp:
  bezier_guide_height_m: 0.12
  bezier_vel_m_s: 0.05
  bezier_orient_finish_ratio: 0.65
  bezier_endpoint_xyz_tol_m: 0.003
  bezier_endpoint_rpy_tol_deg: 3.0
```

- `bezier_guide_height_m`：B点高于C点的距离。
- `bezier_vel_m_s`：曲线TCP峰值速度。
- `bezier_orient_finish_ratio`：姿态旋转完成时的路径比例；当前在前65%完成，后35%保持抓取姿态。
- `bezier_endpoint_xyz_tol_m`：C点实际位置允许误差。
- `bezier_endpoint_rpy_tol_deg`：C点实际姿态允许误差。

## 实机测试日志

测试时重点查看：

```text
[arm] Bezier 右/左 A=(...) B=(...) C=(...)
[bezier_C] 右/左 target=(...) actual=(...) xyz_err=... rpy_err=...
```

如果 `xyz_err` 很小，但夹爪相对物料仍存在稳定偏差，再调整以下参数：

```yaml
goal_x_offset
goal_y_offset
grasp_z_offset_m
```

首次实机验证建议使用单臂、空夹爪和当前低速 `0.05 m/s`，确认曲线路径及C点落位正常后再进行双臂抓取测试。
