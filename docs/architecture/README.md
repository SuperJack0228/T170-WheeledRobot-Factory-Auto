# 架构入口

首次阅读先看 [仓库目录地图](../REPOSITORY_MAP.md)。当前代码复核见 [2026-11-18 记录](../validation/repository_review_20261118/RESULTS.md)。

当前请按 [仓库模块指南](REPOSITORY_GUIDE.md)、[配置启动指南](../deployment/CONFIGURATION.md)、[技能接口](SKILLS_INTERFACE_V1.md) 阅读。历史拆分说明已合并，避免并存多套相互矛盾的操作指引。

进度见 [当前进度](CURRENT_REFACTOR_PROGRESS.md)；逐项原始来源和当前归属见 [迁移表](module_migration.csv)；正式打包前检查 [发布验收](RELEASE_ACCEPTANCE.md)。目标布局仍见 REPOSITORY_LAYOUT.md，其中空骨架不代表已经实现。

机器人与机床按每次调度任务动态组合，不设置 124/120 与某组机床的固定绑定。正式发布仍需匹配 Linux ARM64 SDK 构建、资源安装、升级回滚与本轮现场回归。
