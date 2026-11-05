# 第一步兼容入口与临时边界

基线 `e205e9e`，分支 `wrist-servo`。本步只迁移与解耦，不更改机器人动作、参数或调度语义。

## 保留的转发入口

| 旧入口 | 唯一实现 |
| --- | --- |
| `include/servo/visual_servo.h` | `t170/control/visual_servo.h`；旧的求解声明转发到 `t170/motion/pose_solver.h` |
| `include/servo/trace.h` | `t170/common/servo_trace.h` |
| `include/servo/tray_localizer.h` | `t170/vision/tray_localizer.h` |
| `include/vision/*.h` | `include/t170/vision/*.h` |
| `realsense_get.h`、`seg_pose_bridge.h`、`hand_shadow.h` 等旧头文件 | 对应 `t170/vision` 公开头文件 |
| `Ti5_socketcan.h`、`gripper_interface.hpp`、`head_control.h`、`waist.h` | 对应 `t170/hardware` 公开头文件 |
| `orchestrator/cpp/dispatch_timing.h` | `t170/common/task_timing.h` |
| `LowerBody/CMakeLists.txt`、`LowerBody/include/LowerBody.h` | `libs/lower_body` 的原库及公开头文件 |
| `feeding_cylindrical_parts_alg/algorithm/*.py`、`paths.py`、`visualization.py` | `python/t170_vision/detection`、`visualization` |
| `board_yf100_aruco/*.py`、`board_belt_aruco/*.py`、`online_pose/aruco_detect_api.py` | `python/t170_vision/geometry/{tray,conveyor,marker}` |
| `free_calib/head_camera_to_base.py` | `python/t170_vision/calibration/head_transform.py` |

C++旧头文件只包含新头文件，没有第二套定义。Python旧模块只引导包路径并转发到同一个模块对象，不保留算法副本。旧命令入口仍可执行；裸同名 `online` 不用于主程序，不承诺在同一个旧脚本进程内将两个不同模块都命名为 `online`。

LowerBody旧构建目录可继续 `cmake -S LowerBody ...`；原始 `.cpp`、examples 的源码位置已迁移，外部自行硬编码旧源码路径的编译脚本需改用库目标或新路径。正式程序及本仓库构建已更新。

兼容入口第三步核查外部使用情况后收口。这一步不删除仍可能用于现场诊断的命令。

## 尚待第二步拆分的宿主绑定

- `src/function.cpp` 继续提供 SDK规划、播放、回读、撤离及局部运动学适配。
- `src/servo/runtime.cpp` 继续拥有每手运动会话、命令发送、保位、观测状态历史。
- `src/move_box_runtime.cpp`、`src/grasp_to_standby.cpp` 继续提供工位目标与抓放流程。
- `orchestrator/cpp/robot_runtime.cpp` 的任务和调度实现不在本步重写。
- Python桥接中旧的 `PosePipeline` 组装、信号退出、结果/选目标兼容辅助函数暂留，分别在第二步迁往 app/skills/config 等归属。
- 腰/头实现仍使用旧 `head.h` / `function.h` 的位姿数学或状态声明。腰头文件显式使用 `<head.h>`，避免与新 `t170/hardware/head.h` 同名误引用。
- 头外参查询仍由硬件读取头角；感知流水线和观察线程仍查询既有只读中止状态。没有借此修改停止语义。

这些绑定是明确列出的过渡接口，不是可独立部署的硬件/视觉服务。第二步需要继续消除其中的全局状态和业务耦合。

## 构建关系

```text
t170_common             路径、日志、公共数据/时钟、计时
  ↑ control             纯控制器
  ↑ servo_trace         异步CSV
  ↑ tray_geometry       OpenCV邻孔几何
control → motion_math   局部FK求解
common/control → configuration

hardware / vision / observer  各自编译为对象模块
runtime                      暂留的动作、技能、任务宿主
最终程序                     显式聚合各模块对象和依赖库
```

实际链接方向以 `cmake/T170Foundation.cmake` 和根 CMake 的目标为准：control依赖common，motion_math依赖control；common、trace不反向链接硬件、Python或视觉定位。完整程序显式添加对象模块，避免依靠间接传播导致漏链接或重复实现。

`t170::servo_core` 只作为旧测试/调用的 INTERFACE 聚合别名，不会再编译第二份控制器。

## 配置与资源兼容

旧 YAML、No.124身份/序列号、手眼、模型路径和板参数原位保留。`t170_vision` 通过 `t170_common.paths.load_paths()` 查找这些真实资源，未创建示例标定或伪权重。

当前采用源码包路径导入；`pyproject.toml`、安装树和运行环境锁定仍属第三步。Python包根和配置/几何模块导入不加载Torch或启动相机；请求真正的检测引擎仍需要匹配的Torch/Ultralytics和真实模型。
