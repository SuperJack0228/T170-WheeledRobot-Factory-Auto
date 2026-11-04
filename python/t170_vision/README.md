# t170_vision

从工程 python/（PYTHONPATH）或C++统一嵌入入口导入，不依赖当前工作目录。

- detection：原algorithm实现、配置加载和模型路径。
- geometry.tray / conveyor / marker：独立命名空间的料盘、传送带和单码几何。
- calibration：头相机变换计算，原标定仍由原位置读取。
- visualization：检测绘图，不是必需的生产GUI。

包根/配置/几何导入不启动设备，也不加载Torch或模型；检测引擎仍需要现场真实依赖和权重。
旧目录仅保留兼容转发与现有资源。安装元数据和资源迁移属第三步。
