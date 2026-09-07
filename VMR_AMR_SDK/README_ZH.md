# VMR AMR SDK 中文说明

本仓库提供 VMR AMR 机器人客户端 SDK，包含 C++ 头文件、动态库、Python 绑定库以及示例程序。SDK 通过 RPC 连接机器人端服务，默认端口为 `9000`，可用于设备状态查询、传感器订阅、导航控制、灯光/音频控制、机器人任务下发等场景。

> 注意：示例程序中部分命令会让真实机器人移动、顶升或改变硬件输出。运行前请确认机器人周围环境安全，并确保操作人员具备现场控制权限。

## 目录结构

```text
.
├── include/              # C++ SDK 头文件
├── lib/
│   ├── arm64/            # ARM64 平台 C++ 动态库和 Python 绑定库
│   └── x86/              # x86/x86_64 平台 C++ 动态库
├── examples/
│   ├── C++/              # C++ 示例程序
│   └── python/           # Python 示例程序
├── packUpdate.sh         # 打包脚本
└── README_ZH.md          # 中文说明文档
```

## 支持模块

当前 SDK 主要包含以下能力：

| 模块 | 说明 |
| --- | --- |
| `audio` | 音频播放、停止、音量查询和设置、音频状态订阅 |
| `battery` | 电池设备枚举、电池状态订阅 |
| `bridge_drive` | 底盘速度指令订阅、里程计上传 |
| `bridge_scan` | 桥接激光数据订阅、激光数据上传 |
| `depth_semantic` | 深度语义识别结果订阅、RGB 读取开关 |
| `exception` | 机器人异常等级设置、异常状态订阅 |
| `imu` | IMU 设备枚举、IMU 数据订阅 |
| `io_manager` | IO 状态订阅、DO 输出控制 |
| `laser_2d` | 2D 激光设备枚举、扫描数据订阅 |
| `lift` | 顶升控制、顶升状态订阅 |
| `light` | 灯光控制、灯光状态订阅 |
| `localization` | 定位状态订阅 |
| `map_manager` | 地图列表、当前地图查询、地图管理任务 |
| `map_reader` | 当前地图点位和路径读取 |
| `nav` | 第三方速度控制、速度比例设置、速度下发 |
| `obstacle_avoidance` | 虚拟障碍物新增、查询、删除 |
| `odometry` | 里程计状态订阅 |
| `point_cloud` | 点云数据订阅 |
| `qrcamera` | 二维码相机开关控制、识别结果订阅 |
| `robot_task` | 拓扑移动任务、充电任务、充电继电器控制 |

## 运行环境

- 操作系统：Linux
- 编译器：支持 C++17 的 GCC/G++
- 构建工具：CMake `3.12` 及以上、Make
- Python：当前 ARM64 绑定库文件名为 `cpython-310-aarch64-linux-gnu`，建议使用 Python `3.10`
- 网络：运行示例的设备需要能访问机器人 RPC 服务 IP 和端口 `9000`

## C++ 快速开始

### 构建示例

```bash
cd examples/C++
bash build.sh
```

构建产物默认生成在：

```text
examples/C++/build/client_demo
```

### 运行默认安全烟测

```bash
cd examples/C++
bash start.sh
```

默认连接机器人：

```text
192.168.22.244:9000
```

也可以指定机器人 IP：

```bash
bash start.sh 192.168.22.244
```

默认安全烟测会执行：

- 检查各设备 handle 是否存在
- 测试 `audio`、`qrcamera`、`light`、`obstacle_avoidance` 接口是否可调用
- 查询当前地图、地图列表、地图点位和路径
- 测试 `nav` 零速度控制链路
- 设置 `exception` 等级为 `NORMAL`

### 查看 C++ demo 参数

```bash
bash start.sh --help
```

常用参数：

| 参数 | 说明 |
| --- | --- |
| `[ip]` | 第一个非 `--` 参数作为机器人 IP |
| `--safe` | 运行默认安全烟测 |
| `--handles` | 检查设备 handle |
| `--map-query` / `--map-list` / `--mq` | 查询当前地图、地图列表、地图点位和路径 |
| `--subscriptions` | 短暂注册订阅回调并打印首帧 |
| `--wait <seconds>` | 订阅等待时间，默认 `3` 秒 |
| `--audio-name <name>` | 音频名称或路径，支持 `modbus:xxx`、`tts:文本` |
| `--qrcamera-location <0|1>` | 二维码相机位置，默认 `1` |
| `--point-cloud-loop` | 只订阅点云并持续打印 |
| `--point-cloud-file <path>` | 点云循环模式下保存 CSV |
| `--map-start-mapping <name>` | 开始建图，地图名为 `<name>` |
| `--map-stop-mapping` | 停止当前建图 |
| `--map-start-extension` | 启动当前地图续建 |
| `--map-stop-extension` | 停止当前地图续建 |
| `--map-upload <name> <zip_file>` | 上传地图 zip 文件 |
| `--map-download <name> <zip_file\|dir>` | 下载地图 zip 数据到本地文件；若第二个参数是目录则保存为 `<name>.zip` |
| `--map-download-dir <name> <dir>` | 下载地图 zip 数据到指定目录，文件名为 `<name>.zip` |
| `--map-change <name>` / `--map-use <name>` / `--mc <name>` | 切换当前地图 |
| `--map-delete <name>` / `--map-rm <name>` / `--md <name>` | 删除指定地图 |

## 点云订阅

只订阅并持续打印点云：

```bash
cd examples/C++
bash start.sh --point-cloud-loop
```

按 `Ctrl+C` 退出。

只打印指定秒数：

```bash
bash start.sh --point-cloud-loop --wait 5
```

输出示例：

```text
point_cloud[1] timestamp_ns=1777549708392999936 location=-1 points=962 non_zero=930 first_valid=(0.12, -0.03, 1.4)
```

字段说明：

| 字段 | 说明 |
| --- | --- |
| `timestamp_ns` | 点云时间戳，单位 ns |
| `location` | 点云设备位置，深度相机通常为 `-1 ~ -8` |
| `points` | 当前帧点数量 |
| `non_zero` | 非 `(0,0,0)` 的点数量 |
| `first_valid` | 第一个非零点 |

保存点云到 CSV：

```bash
bash start.sh --point-cloud-loop --wait 5 --point-cloud-file /tmp/point_cloud.csv
```

CSV 格式：

```csv
frame,timestamp_ns,location,point_index,x,y,z
1,1777549708392999936,-1,0,0,0,0
```

一行表示一个点：

| 字段 | 说明 |
| --- | --- |
| `frame` | demo 接收到的帧序号 |
| `timestamp_ns` | 当前帧时间戳 |
| `location` | 点云设备位置 |
| `point_index` | 当前点在该帧中的索引 |
| `x/y/z` | 点坐标，单位 m |

## 订阅测试

订阅接口和普通 RPC 控制接口建议分开运行。当前 C++ demo 默认不运行订阅测试，需要显式打开：

```bash
cd examples/C++
bash start.sh --subscriptions --wait 3
```

订阅测试会尽量只打印首帧，避免长时间刷屏。多实例设备如 `battery`、`imu`、`laser_2d` 会先枚举 handle，再按 handle 注册订阅。

## 地图管理

默认安全烟测会执行 `--map-query`，查询当前地图、地图列表以及当前地图点位和路径：

```bash
cd examples/C++
bash start.sh --map-query
bash start.sh --mq
```

以下 `map_manager` 操作会改变机器人地图状态或本地文件，需要显式传参运行。

开始/停止建图：

```bash
bash start.sh --map-start-mapping new_map
bash start.sh --map-stop-mapping
```

开始/停止当前地图续建：

```bash
bash start.sh --map-start-extension
bash start.sh --map-stop-extension
```

上传/下载地图 zip 数据：

```bash
bash start.sh --map-upload demo_map /tmp/demo_map.zip
bash start.sh --map-download demo_map /tmp/demo_map.zip
bash start.sh --map-download-dir demo_map /tmp/maps
```

切换/删除地图：

```bash
bash start.sh --map-change demo_map
bash start.sh --mc demo_map
bash start.sh --map-delete demo_map
bash start.sh --md demo_map
```

## 会改变真实设备状态的测试

以下命令会让真实设备动作或改变输出，必须确认现场安全后再运行。

顶升：

```bash
cd examples/C++
bash start.sh --lift 100
```

拓扑移动：

```bash
bash start.sh --topo P1
```

充电任务：

```bash
bash start.sh --charge 1.0 2.0 0.0
```

充电继电器：

```bash
bash start.sh --charge-relay 1
bash start.sh --charge-relay 0
```

异常 STOP 测试：

```bash
bash start.sh --exception-stop
```

IO DO 输出：

```bash
bash start.sh --io-set-do 0 1 1
bash start.sh --io-set-do 0 1 0
```

深度语义 RGB 读取开关：

```bash
bash start.sh --depth-rgb-enable 0 1
bash start.sh --depth-rgb-enable 0 0
```

## C++ 集成方式

业务程序集成 SDK 时，需要：

1. 添加 `include/` 到头文件搜索路径。
2. 按目标平台和系统版本添加 `lib/<arch>/<ubuntu>/` 到动态库搜索路径，例如 `lib/arm64/22.04/`、`lib/x86/18.04/`。
3. 链接所需的 `client_*` 动态库以及 `client_internal`。
4. 运行前设置 `LD_LIBRARY_PATH`。
5. 在调用任何模块接口前先调用 `rpc_client::InitClient`。

最小示例：

```cpp
#include <iostream>

#include "client/common/client_common.h"
#include "nav.h"

int main() {
  rpc_client::ClientConfig config;
  config.ip = "192.168.22.244";
  config.port = 9000;
  config.connect_timeout_ms = 3000;
  config.call_timeout_ms = 60000;

  if (!rpc_client::InitClient(config)) {
    std::cerr << "init client failed" << std::endl;
    return 1;
  }

  if (nav::HasHandleNavDev()) {
    nav::Twist zero{};
    nav::set_third_speed_ctrl();
    nav::send_third_expected_speed(zero);
    nav::close_third_speed_ctrl();
  }

  rpc_client::CloseClient();
  return 0;
}
```

运行前设置动态库路径：

```bash
export LD_LIBRARY_PATH=/path/to/SDK/lib/arm64/22.04:$LD_LIBRARY_PATH
```

`examples/C++/CMakeLists.txt` 会根据当前机器架构和 `/etc/os-release` 自动选择库目录；如果需要手动指定 Ubuntu 版本，可在构建时传入：

```bash
cmake .. -DSDK_UBUNTU_VERSION=20.04
```

## Python 快速开始

Python 示例位于 `examples/python/`。`start_py.sh` 会根据当前机器架构和 Ubuntu 版本自动选择 `lib/<arch>/<ubuntu>`，并设置 `LD_LIBRARY_PATH` 和 `PYTHONPATH`。

使用脚本运行：

```bash
cd examples/python
./start_py.sh
./start_py.sh lift_example.py
```

不传参数时默认运行 `rpc_test.py`。

手动设置环境变量后运行：

```bash
cd examples/python
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:../../lib/arm64/22.04/
export PYTHONPATH=../../lib/arm64/22.04/:$PYTHONPATH
python3 rpc_test.py
```

Pylance 类型检查可参考 [examples/python/README.md](examples/python/README.md)。

## 打包

仓库提供 `packUpdate.sh` 用于生成发布压缩包：

```bash
bash packUpdate.sh
```

压缩包命名格式：

```text
VMR_AMR_SDK_<branch>_<short_hash>_<YYYYMMDD>.zip
```

## 常见问题

### 连接失败

请检查：

- 机器人 IP 是否正确。
- 当前设备与机器人网络是否互通。
- 机器人端 RPC 服务是否已启动。
- 防火墙是否放行端口 `9000`。

### 运行时报找不到动态库

请确认 `LD_LIBRARY_PATH` 包含对应平台和系统版本的 `lib` 目录，例如：

```bash
export LD_LIBRARY_PATH=/path/to/SDK/lib/arm64/22.04:$LD_LIBRARY_PATH
```

### Python 无法 import 模块

请确认：

- Python 版本与绑定库 ABI 匹配。
- `PYTHONPATH` 已包含对应版本目录，例如 `lib/arm64/22.04/`。
- `LD_LIBRARY_PATH` 已包含对应版本目录，例如 `lib/arm64/22.04/`。

### 订阅回调和普通 RPC 调用异常

当前示例程序中订阅测试默认关闭，并建议与普通 RPC 控制接口分开运行。需要测试订阅时，请单独使用：

```bash
cd examples/C++
bash start.sh --subscriptions --wait 3
```

### C++ 示例链接到错误平台或系统版本

`examples/C++/CMakeLists.txt` 会自动选择 `lib/<arch>/<ubuntu>/`，并在配置阶段打印实际使用的 `SDK library dir`。如需强制指定版本，可删除 `examples/C++/build/` 后执行：

```bash
cmake .. -DSDK_UBUNTU_VERSION=18.04
```
