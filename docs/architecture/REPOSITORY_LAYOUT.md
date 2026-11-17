# 当前仓库布局

更新：2026-11-17。实际目录和各模块用途见 [仓库指南](REPOSITORY_GUIDE.md)。同事交付的 SDK 和通用库统一放入 dependencies，不再以 third_party 泛称 Ti5 自有 SDK。

```text
apps/                      正式入口
src/                       应用、任务、技能、运动、控制、视觉与硬件实现
include/t170/              上述模块公开接口
python/                    Python 视觉和配置包
config/                    默认、型号、工厂、单机、标定和部署选择
assets/                    模型和音频
dependencies/
  sdk/                     arm、lower_body、device、chassis_seer
  libraries/               eigen、unitree_motor、serialport
  manifest.json            本地交付摘要与外部环境依赖
cmake/                     显式构建清单与依赖路径
deploy/                    生产/维护/网页启动及服务模板
tools/                     控制客户端、分析、预检与校准
tests/                     离线回归和固定样本
examples/skills/            标准技能调用示例
web_debug/                 当前网页
packaging/                 安装打包预留
.github/                   CI 预留
docs/                      当前文档和验证证据
```

原来的 libs/、LowerBody/、lib/、eigen3/、ti5-device-sdk/、ti5-agv-seer/、third_party/ 均已退出根目录。旧 LowerBody 只是兼容壳，真实 SDK 随迁移完整保留。

dependencies 是依赖来源边界，不是硬件功能模块；硬件适配仍在 src/hardware，运动算法/调用仍在 src/motion。Robot/工位参数和标定继续在 config，不移进 SDK。

机器人实际安装在 /opt 的 SDK 动态库仍由原环境提供，详见 [依赖说明](../../dependencies/README.md)。未来安装树可采用 /opt/t170-factory（软件）、/etc/t170-factory（现场配置）和 /var/log、/var/lib（日志状态）等边界；这些安装位置不是本轮已完成的交付。
