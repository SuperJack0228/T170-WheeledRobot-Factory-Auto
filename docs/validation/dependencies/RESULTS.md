# SDK 与依赖集中整理

日期：2026-11-17。工作区：T170-field-review-20261111。本轮只改依赖归属、构建/资源路径和相应文档/检查，不升级 SDK，不提交或推送 Git。

## 目录变化

全部本地交付集中 dependencies：sdk/{arm,lower_body,device,chassis_seer}，libraries/{eigen,unitree_motor,serialport}。原根目录 libs、LowerBody、lib、eigen3、ti5-device-sdk、ti5-agv-seer、third_party 全部退出。Ti5_Arm.h 从自有 include 移到 SDK 的 include。

删除旧 LowerBody 转发壳、空占位、无源码消费者的旧 dex1/param.h 与失效演示说明。保留实际腰腿源码/自检；Unitree/serialport 仍有原构建依赖，归位保留，不把未直接调用当成已证明可删。

CMake 路径集中 cmake/T170Dependencies.cmake；更新包含目录、库路径、RPATH、预检、源码摘要和开发工具。机器人 /opt 的安装 SDK 选择方式保持；设备/底盘本地仅有头文件，未宣称包含全部实现库或已完成 ABI 核对。

## 检查

- 从新的 /tmp/t170-dependency-build 配置 Release 离线构建，全量构建通过，34/34 CTest 通过。
- 实际 lower_body_self_test 通过，fail=0。
- 迁移的 616 个文件与清理前逐个 SHA256 比较，只有 SDK README 构建路径说明改变；代码、头文件和预编译库内容均保留。
- src、include/t170、apps、python、config、assets 与迁移前内容核对保持；Web 按本轮开始时哈希核对，保留用户已有修改。
- 模块边界测试仍解析新位置的手臂头文件，避免迁移后因找不到 SDK 而漏查。
- 本地依赖 manifest 哈希、Python 语法、Markdown 本地链接和 git diff --check 通过；新库未被 .gitignore 忽略。

本轮未启动机器人。Mac 离线构建不能替代 Linux ARM64 完整 SDK 链接、实际动态加载和现场回归。软件安装/打包依然是后续步骤。

清理前完整副本、迁移表和原始日志位于源码树外 /Users/superjack/Ti5/T170-field-analysis-20261117/dependency-consolidation/，其中 before 为本轮前的工作区内容。依赖说明见 ../../../dependencies/README.md。
