# T170C 圆柱抓取与头相机二维码调试工程

本目录已收缩为一个无底盘调试工程，只保留：

- T170C 左右七轴机械臂、腰部、DEX1 双夹爪；
- D435I 头相机和左右 D405 手相机；
- 圆柱物料 YOLO 分割、位姿估计与抓取；
- 头相机 ArUco 二维码识别及相机系/机器人基座系位姿；
- 归位、状态读取、配置重载和动作中止。

## 构建（机器人 Ubuntu 主机）

```bash
cmake -S . -B build
cmake --build build -j$(nproc)
./build/t170c_debug
```

程序只监听本机 `127.0.0.1:8099`。另开终端执行：

```bash
python3 tools/debug_client.py status
python3 tools/debug_client.py qr
python3 tools/debug_client.py grasp
python3 tools/debug_client.py home
python3 tools/debug_client.py abort
```

`qr` 对应当前已有的 ArUco 方形码，不是存储文本/网址的通用 QR Code。

## 建议调试顺序

1. 不装物料运行 `status`，确认双臂和腰部反馈正常。
2. 运行 `qr`，只验证头相机、码尺寸配置及手眼标定。
3. 低速执行 `home`，确认关节方向和待机位。
4. 放置单个圆柱后执行 `grasp`，旁边保留实体急停人员。
5. 任意异常立即从另一终端执行 `abort`，同时使用实体急停。

关键配置位于 `config/`，视觉模型位于
`feeding_cylindrical_parts_alg/models/best.pt`。

## 清理说明

已移除底盘/VMR SDK、Modbus 机床、语音、Web 大编排台、放货/传送带业务、
重复视觉工程、重复模型、历史入口、构建产物和调试图片。需要恢复这些能力时，
请从原始备份按模块恢复，不要把旧目录整体覆盖回来。
