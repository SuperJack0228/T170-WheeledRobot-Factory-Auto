# T170 同环境新机快速部署

适用：机器人使用与 No.124 匹配的系统、视觉和 SDK 环境。源码与安装包共用启动和应用入口；安装包不需要在第二台机器编译 C++。完整说明见 [V1 快速部署与调参](V1_QUICK_START.md)。

## 建档

进入仓库或解压后的包目录，用当前视觉环境执行；将示例编号、相机 SN 替换为实际信息：

```bash
./deploy/setup_robot.sh TH-T170C-0125 --model 3 \
  --head-serial 实际头相机SN --left-serial 实际左手SN --right-serial 实际右手SN
```

5 kg 使用 `--model 5`；底盘地址不同增加 `--chassis-host 实际IP`。档案存在时不会覆盖本机标定。默认选择保存在 `config/deployment.yaml`，型号保存在该机 deployment 的 `robot_model`；编号同时写入身份文件供调度使用。

新机器人默认使用公共目录 `config/robot_models/t170c_v2_common/cameras/` 中的头相机和左右手相机理论值，不需导入 No.124 的实测手眼。当前 No.124 档案保留原有手眼。相机 SN 位于本机 `cameras/realsense_cameras.yaml`。工具和装配不同才修改本机 `arm_tools.yaml`、`arm_binding.yaml`。

## SDK 与构建

沿用公司 APT 源，安装与运行包发布清单匹配的版本。现有包名：

```bash
sudo apt update
sudo apt install ti5-device-sdk ti5-jindi-faactory-robot-client
```

APT 源的实际版本由运行包环境清单确定；不要用未确认的升级版本替代已验证 SDK。首次安装不能使用 `--only-upgrade`。运动学动态库随本项目运行包提供。

源码部署时编译一次：

```bash
cmake -S . -B build_robot -DCMAKE_BUILD_TYPE=Release
cmake --build build_robot --parallel 3
```

## 启动与标定

```bash
./deploy/apply_config.sh --validate-only
./deploy/run_maintenance.sh
```

另开终端：

```bash
./deploy/run_web.sh
```

网页地址为启动打印的 `http://机器人IP:8080/?token=...`。通过向导建立本机左/右抓取、左/右放置四份参考，结果进入 `config/calibration/本机编号/tasks/`。新版安装模式直接在运行包内保存 `config/`，源码模式为仓库根目录。rc.3 的隐藏工作区迁移见 [升级说明](PACKAGE_UPGRADE.md)。

理论手眼不等于免除抓取点/放置点图像参考。四份参考中的像素、相机内参、实际姿态及几何指纹属于本机，不复制另一台机器的任务结果。

## 调参和生产

当前网页列出的常调项全部位于本机 `quick_params.yaml`，包括抓放高度偏置、动作速度及夹爪准备开度。网页点保存会永久保存；直接编辑任何被当前档案引用的 YAML 后统一执行：

```bash
./deploy/apply_config.sh
```

支持热更新的改动在空闲时应用。需要重启的改动保留并提示，下次启动自动加载。旧的 `move_box_params.yaml` 不再使用，自动生成的 `effective_params.yaml` 不作为调参文件。

维护验证结束后停止维护服务，再启动：

```bash
./deploy/run_production.sh
```

生产入口由调度统一分配任务。每次应用和启动都可查到当前机器人、型号、参数来源和输入摘要。
