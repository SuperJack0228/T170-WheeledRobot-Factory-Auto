# Ti5 Robotics 双臂算法 SDK

交付版本：20261109；来源：同事提供的 `ti5-robot-dynamics-20261109` 原始压缩包。头文件及两个动态库均按原始字节保存。

```text
include/Ti5_Robotics.h
lib/aarch64-glibc227/libti5_robot.so
lib/x86_64-glibc227/libti5_robot.so
```

对应原包：`ti5-robot-dynamics-aarch64-glibc227-20261109.tar.gz`、`ti5-robot-dynamics-x86_64-glibc227-20261109.tar.gz`。选择 glibc 2.27 交付版以降低最低系统要求；最终是否可加载，仍应由目标机动态链接器、libstdc++ 和真实运行结果确认。macOS 无法直接加载这些 Linux ELF。

CMake 按目标架构选择平台，支持 `TI5_ARM_SDK_PLATFORM` 显式指定，选定值记录在构建目录 `arm_sdk_platform.txt`。其他 glibc 交付版未复制入库。完整来源和文件摘要见 `dependencies/manifest.json`。

供应方 SDK 根据进程工作目录读取 `config/Robot_Model.yaml`；通过项目部署启动脚本运行。项目适配器在硬件初始化前核验选定型号及 SDK 的 DH、工具回读。不能把一份任意位置的 YAML 路径传给上层后，就假设底层已读取该文件。

生产代码仅在 `src/motion/vendor_arm.cpp` 包含此头。旧 `Ti5_Arm` 的头、库迁至同级 `arm_legacy/` 供历史追溯，不参与链接。模型角转换、连续局部 IK、直线轨迹策略归项目 motion 层管理，不修改交付二进制。

本次只接入 T7 运动学，不启用此库仅对 A6 有效的动力学、零重力、阻抗或碰撞接口。离线回归不能代替 Linux ABI 和实机验收。
