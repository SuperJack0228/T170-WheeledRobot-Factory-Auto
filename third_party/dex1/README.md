# 夹爪依赖说明（换机部署用）

本工程夹爪相关文件已全部放在 `market_simple_2026_1_21_using` 内，**不依赖** 目标机器上的 `~/dex1_1_service`。

## 已打包内容

| 路径 | 内容 |
|------|------|
| `include/gripper_interface.hpp` | 夹爪对外 API |
| `src/gripper_interface.cpp` | 夹爪实现源码 |
| `third_party/dex1/include/` | 宇树串口 SDK 头文件（SerialPort、unitreeMotor 等） |
| `lib/libUnitreeMotorSDK_Arm64.so` | aarch64 电机协议动态库 |
| `lib/libUnitreeMotorSDK_Linux64.so` | x86_64 电机协议动态库 |
| `third_party/debs/libserialport*.deb` | libserialport 离线安装包（aarch64） |
| `gripper/` | 独立演示/标定程序（可选，与主工程同源） |

## 新机器部署步骤

```bash
# 1. 安装 libserialport（仅需一次）
sudo dpkg -i third_party/debs/libserialport0_*.deb third_party/debs/libserialport-dev_*.deb
sudo ldconfig

# 2. 编译主程序
cd build && cmake .. && make -j

# 3. 运行（夹爪需串口权限）
sudo ./move
```

## 标定（首次或换夹爪后）

```bash
cd gripper && ./build.sh
sudo ./build/gripper_calibrate
```

官方文档：https://support.unitree.com/home/zh/dex1-1_gripper/dex1_1
