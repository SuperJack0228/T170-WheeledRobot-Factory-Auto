# vision（第一步已迁入）

公开接口在 `include/t170/vision`；Python算法在 `python/t170_vision`。

- camera：相机取帧、帧时间戳、相机接口适配。
- detection：逻辑模型登记与检测适配；实际权重由已加载Python引擎提供。
- geometry：码与位姿、邻孔、头外参。
- tracking：手相机观察、目标关联与观测发布；只读状态端口为servo_observer_port.h。
- bridge：Python嵌入、数组转换、可视化；旧PosePipeline/退出等宿主职责待第二步拆开。
- pipeline.cpp：取帧与检测/检码编排。

本步不修改模型/控制参数/采集时序，资源仍在旧目录。兼容与待验证项见 docs/architecture/STEP1_REPORT.md。
