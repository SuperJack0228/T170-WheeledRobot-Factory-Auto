# 头相机到基座变换

实现为 python/t170_vision/calibration/head_transform.py；C++ 动态头变换入口为 src/vision/geometry/head_transform.cpp。默认部署数据分别来自 config/calibration/TH-T170C-0124/head/geometry.yaml 和 extrinsics.yaml，运行目录映射为 config/head/geometry.yaml、extrinsics.yaml。

几何文件包含头部安装/连杆参数及外参路径；外参文件保留实际矩阵与坐标约定，不能根据文件名猜方向或把毫米直接当米。调用接口仍为 HeadCameraToBase、euler_to_rotation_matrix 和 make_transform，纯计算不启动电机。

命令行入口迁为 `python3 tools/calibration/head_transform.py --help`。它调用同一 canonical main；具体俯仰/回转和输出参数以 --help 为准。源代码模式按选中的 deployment 找对应数据；机器人应用必须使用生成后的统一运行目录。更换机器人应导入真实测量结果，不能复制 No124 外参当通用标定。

料盘抓取/放置补偿和参考点向导另见 [标定向导](PLACEMENT_WIZARD.md)，二者不是重新测得头相机完整六维手眼。
