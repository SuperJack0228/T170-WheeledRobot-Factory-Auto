# 统一依赖目录

本目录收纳同事交付的 Ti5 SDK、预编译库和通用数学/通信库，统一称为 dependencies。SDK 不因由其他同事交付就叫第三方。

```text
dependencies/
├── sdk/
│   ├── arm/
│   │   ├── include/Ti5_Robotics.h
│   │   └── lib/{aarch64,x86_64}-glibc227/libti5_robot.so
│   ├── arm_legacy/                 旧手臂 SDK 原始归档，不参与构建
│   ├── lower_body/                 腰腿源码、头、独立构建和自检
│   ├── device/include/             设备 SDK 头文件交付副本
│   └── chassis_seer/include/       底盘 SDK 头文件交付副本
├── libraries/
│   ├── eigen/                      Eigen 头文件
│   ├── unitree_motor/lib/
│   │   ├── arm64/
│   │   └── x86_64/
│   └── serialport/                 串口头文件与 Linux ARM64 库
└── manifest.json                   当前交付文件摘要与外部依赖说明
```

CMake 依赖路径集中在 cmake/T170Dependencies.cmake，新手臂库选择在 cmake/T170ArmSdk.cmake。根目录原有 LowerBody/ 是转发兼容壳，现已删除；真正的腰腿源码在 sdk/lower_body，算法内容未改。

2026-11-17 手臂后端已切换为 Ti5_Robotics 20261109，不能再使用旧二进制直接替换库。具体来源见 [手臂 SDK](sdk/arm/README.md)，模型角、法兰与任务参考处理见 [接入说明](../docs/deployment/ARM_SDK_MIGRATION.md)。清单中手臂 active/archive 条目的 content_sha256 为按路径排序后 `path + NUL + file_sha256 + LF` 的 UTF-8 摘要，hash_scheme 单独声明；其他依赖条目保留原始摘要。

## 本地文件与机器人安装环境

手臂库、腰腿源码、Eigen、Unitree 和串口回退库来自本目录。设备/底盘目录当前只有头文件，不含完整实现动态库；正式构建仍默认使用现有 /opt/ti5-* SDK 的头文件和库。底盘离线编译检查使用本目录头文件，与迁移前选择方式一致。

外部路径可通过原有 TI5_*_ROOT CMake cache 参数配置。机器人上的通信、编解码、底盘、调度客户端和 Python/RealSense 等环境没有被这次迁移复制进仓库。manifest 明确标记哪些文件在本地、哪些仍依赖安装环境；不能把本目录理解为已经完成独立发布。

保留安装头文件优先级是为了避免在未核对 ABI 时切到另一份头；正式发布前应核对本地交付头与实际 /opt 库的一致性，固定版本及哈希。当前 manifest 的本地哈希是已测值，外部安装版本没有数据就不填假版本。

Unitree 和 serialport 仍被原构建链接，二者暂时保留。现有 Unitree 二进制依赖 libserialport.so.0；不能只因上层未直接调用串口接口就删除它。旧 dex1/param.h 无源码引用，旧演示说明失效，已退出。

## 开发与运行

项目自有硬件/运动适配代码在 src/hardware、src/motion；这里保留交付实现的边界，日常业务改动不复制 SDK 源码到这些模块。变更 SDK 时应更新本目录清单，核对头/库、架构和实际加载位置，再运行完整构建/现场回归。

源码开发需要相应头文件；运行包只携带运行所需实现和资源，不必把 Eigen 全套开发头都装到机器人运行目录。当前 CMake 仍是源码运行布局，install/打包和可重定位路径后续单独验收。
