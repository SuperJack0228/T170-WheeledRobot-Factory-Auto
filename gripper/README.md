# DEX1 夹爪直接串口控制

宇树 DEX1-1 夹爪 C++ 库，**直接 USB 串口**，无需 ROS2、无需 DDS。

与主工程 `include/gripper_interface.hpp`、`src/gripper_interface.cpp` **源码同步**。

---

## 两套 API（并存，互不影响）

| 类型 | 前缀 | 力矩限位 | 适用 |
|------|------|----------|------|
| **经典** | `open` / `close` / `graspAndWait` | 监测 `\|torque\|≥grasp_torque_limit_nm` | 搬箱全速 |
| **Soft** | `openSoft` / `closeSoft` / `graspSoftAndWait` | ros2_170D kp/kd 反算 + pos_filter | 防过夹、遥操精细 |

Soft 参数（Config）：
- `soft_torque_limit_nm` = 2.0（对应 ros2 `soft_torque_limit`）
- `pos_filter` = 0.2
- `soft_slew_rate` = 10.0（对应 ros2 `default_slew_rate`）

参考：`ros2_170D/src/gripper_driver/src/dex1_gripper_driver_node.cpp`

---

## 快速开始

```bash
cd gripper && ./install_deps.sh
sudo ./build/gripper_grasp_demo        # 经典 graspAndWait
sudo ./build/gripper_grasp_soft_demo   # Soft graspSoftAndWait
sudo ./build/gripper_demo
```

主工程对比测试：`cd build && sudo ./move`（经典 + Soft 左右开合）

---

## 同步

```bash
cp include/gripper_interface.hpp gripper/
cp src/gripper_interface.cpp      gripper/
./gripper/sync_vendor.sh && cd gripper && ./build.sh
```

完整 API 说明见 `include/gripper_interface.hpp` 内注释。
