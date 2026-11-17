# 模块迁移表说明

更新：2026-11-16。逐项清单见 [module_migration.csv](module_migration.csv)，当前目录及职责见 [仓库指南](REPOSITORY_GUIDE.md)。

source 是迁移前来源，保留旧路径用于追溯；current_source 是当前归属；status 标明已本地验证、仍部分实现或等待外部环境。历史 source 中出现已删除文件是有意留存，不代表仍有运行依赖。

运动、技能、任务和应用的实际拆分已完成本地实现；旧 C++ 转发头/实现、Python 转发入口、历史实验退出；配置和资源转为 canonical 路径。R01/R02 资源、R04 工具和 R08 现场数据清理已推进，SDK 归一/安装包/CI 不能因目录存在就标为完成。

新的完整结构可支持继续封装，但仍有全局配置快照、受控参数覆盖、供应商依赖版本和机器人构建/现场回归等明确剩余项。见 [当前进度](CURRENT_REFACTOR_PROGRESS.md) 与 [发布验收](RELEASE_ACCEPTANCE.md)。
