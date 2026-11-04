# 第一步检查范围与结果

基线：e205e9e；检查日期：2026-11-04。执行环境为macOS，本次没有连接机器人。

## 已通过

| 检查 | 结果 |
| --- | --- |
| 新离线构建目录配置、编译 | 通过 |
| 原9项回归 | 全部通过，未静默跳过 |
| LowerBody独立库自检 | 通过 |
| 公开头文件不依赖设备SDK/Python/RealSense实现 | 通过 |
| 模型登记角色与配置路径绑定 | 通过；不加载权重 |
| Python视觉包检查 | 5个用例通过：跨工作目录导入、旧入口身份、真实板配置、缺模型错误、算法/数学/语法 |
| 模块边界检查 | 4个用例通过：control/trace依赖、观察线程权限、构建源路径、Python命名空间 |
| 旧LowerBody CMake入口构建及自检 | 通过 |
| 配置、C++算法主体、Python算法及数据类型等价核对 | 16/27/16/31项全部通过，见equivalence.json |
| 支持本机依赖的C++整文件语法检查 | 14份源码通过，包含拆分后的Python桥接/可视化和原动作宿主 |

CMake共14项检查。Python用例需要NumPy、OpenCV ArUco和PyYAML；缺少依赖会明确失败，不作为跳过成功。

机器可读记录：[CTest结果](ctest_summary.json)、[C++语法检查](cpp_syntax.json)、[迁移等价核对](equivalence.json)。完整机器人构建配置也实际尝试过，当前在查找机器人专用Python环境时失败，见[诊断记录](robot_configure.json)；没有将配置失败算作通过。

## 本机检查命令

```sh
cmake -S . -B build_servo_tests -DT170_SERVO_TESTS_ONLY=ON
cmake --build build_servo_tests -j4
ctest --test-dir build_servo_tests --output-on-failure
python3 tools/analysis/verify_step1_equivalence.py --output docs/validation/step1/equivalence.json
```

本机运行时为CMake提供了临时yaml-cpp安装和Homebrew OpenCV路径；PyYAML/pybind11安装在临时测试目录，没有改动机器人/系统Python环境。项目不依赖这些个人临时路径，命令参数可替换为匹配环境的真实依赖位置。

## 未验证

- 本机缺少Linux CAN系统头、机器人ARM64实际SDK/动态库及机器人Conda环境，不能完成全程序真实链接或ABI验证。
- RealSense原实现只作原文迁移/包含路径更新；实际取帧、时间戳和多设备行为待相机环境验证。
- 本机稀疏检出没有模型权重，也未运行Torch/Ultralytics真实推理；检查覆盖配置角色和缺失模型的明确失败，没有创建假权重。
- 未执行实际抓取/放置/升腰/底盘或调度任务，不包含连续自动化运行的现场验收。

因此当前结论是“第一步本地实现和检查完成”，不是“完整第一版发布验收完成”。
