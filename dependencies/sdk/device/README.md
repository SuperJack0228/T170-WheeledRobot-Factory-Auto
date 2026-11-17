# ti5-device-sdk（设备通信与命令面）

原包名 **communication-manager**。`apt install ti5-device-sdk` 会卸掉旧包。桌面文件夹 **ti5-device-sdk**，系统路径：`/opt/ti5-device-sdk/`。

包含 `Ti5_Device_SDK.hpp` 后进 `main` 前按 `devices.json` 自动配对，电机编解码缓存一并初始化；进程退出时 `close()`。不管调度、运动学、笛卡尔控制。仙工 AGV 代码在独立库 `ti5-agv-seer`，装本包时会一并装上。

依赖 **ti5-can-mini**、**ti5-serial**、**ti5-motor-codec**、**ti5-rohand-codec**、**ti5-dex1-codec**、**ti5-agv-seer**，会一并装上。

链接仍是 `-lcommunication_manager`。

## 路径

| 路径 | 内容 |
|------|------|
| `/opt/ti5-device-sdk/include/Ti5_Device_SDK.hpp` | 总头：自动配对 / `pair` / `close` |
| `/opt/ti5-device-sdk/include/Ti5_Motor.hpp` | 电机命令面（输出轴 rad / rad/s / A） |
| `/opt/ti5-device-sdk/include/Ti5_ROHand.hpp` | 灵巧手命令面 |
| `/opt/ti5-device-sdk/include/Ti5_Dex1.hpp` | 夹爪命令面（输出侧开合） |
| `/opt/ti5-device-sdk/include/Ti5_Motor_codec.h` | 电机编解码（`Ti5_Motor.hpp` 会 `#include`） |
| `/opt/ti5-device-sdk/include/Ti5_ROHand_codec.h` | 灵巧手编解码（对照副本） |
| `/opt/ti5-device-sdk/include/Ti5_Dex1_codec.hpp` | 夹爪编解码（对照副本） |
| `/opt/ti5-device-sdk/lib/libcommunication_manager.so` | 本库 |
| `/opt/ti5-device-sdk/share/devices.json` | 配对配置（改完重启进程生效） |

编解码动态库在各自的 `/opt/ti5-*-codec/lib/`，不打进本包。

## 依赖

编译自己的程序需要 **C++20**（`g++`）。运行需要上述 `Depends` 包。

电机和灵巧手**不共一条 CAN**。夹爪只走串口 6 Mbps。自动配对后控制期间不要再 `pair()`。空闲 CAN 不再限 8 路：按枚举打开，并补扫 `can0`～`can31`，两块 6 通道卡会打开全部 12 路。

## 最短例子

```cpp
#include "Ti5_Device_SDK.hpp"

int main() {
    Motor_Enable(1);
    set_Position(1, 0.5);
    Motor_Disable(1);
}
```

```bash
g++ -std=c++20 \
  -I/opt/ti5-device-sdk/include \
  -L/opt/ti5-device-sdk/lib \
  -Wl,-rpath,/opt/ti5-device-sdk/lib \
  -Wl,-rpath,/opt/ti5-serial/lib \
  -Wl,-rpath,/opt/ti5-can-mini/lib \
  -Wl,-rpath,/opt/ti5-motor-codec/lib \
  -Wl,-rpath,/opt/ti5-rohand-codec/lib \
  -Wl,-rpath,/opt/ti5-dex1-codec/lib \
  -o demo demo.cpp \
  -lcommunication_manager -lti5_motor -lrohand_codec -lTi5_Dex1_codec \
  -lserial_port -lpthread
```

## 配对

`/opt/ti5-device-sdk/share/devices.json` 决定扫哪些设备。未知键（含旧的 `agv`）会跳过。id：`"1-50"` 闭区间，`"1,3,8"` 离散，可混写 `"1-10,20"`。电机 / 手 1～128；夹爪总线 id 0 右 / 1 左。CAN 探活和一问一答超时 5ms；手串口探活 50ms、一问一答 200ms。默认扫描：电机 `1-35`，手 `2,3`，夹爪 `0,1`。

- 包含 `Ti5_Device_SDK.hpp` 即自动 `pair()`；已配对再调直接返回
- `close()` 之后可以再 `pair()`
- 仙工 AGV：`#include "Ti5_Seer.hpp"`（随本包装上的 `ti5-agv-seer`）

## 电机 / 灵巧手 / 夹爪

电机量都是**输出轴**：位置 rad、速度 rad/s、电流 A。配对后已自动 `motor_Init`。CSP / MIT 热路径把反馈写入调用方变量或 `span`。

灵巧手：`finger_id` 0 拇指 … 5 拇指根；逻辑位置 0～65535；角度 = 实际 ×100；`speed` 0～255。无力控。

夹爪：输出侧开合（全开约 5 rad，全闭 0）。`set_Gripper_Pos` 只发；`set_Gripper_Pos_get_State` / `get_Gripper_State` 发 20 等 26。

## 排障

| 现象 | 常见原因 |
|------|----------|
| 自动配对后没有设备 | 检查 `devices.json`、总线是否 UP、手和电机是否抢同一条 CAN |
| 找不到头文件 | `-I` 应指向 `/opt/ti5-device-sdk/include`；总头是 `Ti5_Device_SDK.hpp` |
| 运行找不到 `.so` | 链接时加上本库和各 `/opt/ti5-*-codec/lib` 的 rpath |
