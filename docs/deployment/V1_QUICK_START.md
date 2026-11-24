# T170 V1 候选版部署与参数操作

版本：1.0.0-rc.3。程序、配置和运行数据独立保存。`move_box_params.yaml` 已退出当前运行链路。已有机器人升级见 [运行包升级说明](PACKAGE_UPGRADE.md)，十台机床高度配置见 [信息码与高度绑定](CONVEYOR_HEIGHT_BINDINGS.md)。

## 日常调参

网页参数页对应当前机器人的 `config/robots/机器人编号/quick_params.yaml`，保存会写入持久文件。程序空闲且改动支持热更新时立即应用；忙碌、标定中或涉及模型/相机资源时，保存为下次启动配置并明确提示。

手工修改任何被选中档案引用的 YAML 后，统一执行：

```bash
./deploy/apply_config.sh
```

未启动主程序时只做完整校验，下次维护或生产启动加载。`--validate-only` 明确只校验。非法配置不替换正在执行的配置；网页保存校验失败会恢复原文件。

每个参数只有一个持久来源。网页中的 98 项常调字段归 `quick_params.yaml`；稳定控制默认值、场景视觉、身份、模型仍归对应文件。同一个键重复定义会报错并指出归属。程序自动合成 `config/effective_params.yaml`，此文件只供读取和日志快照，不手工维护。更改不在活动档案中的 YAML 不影响当前机器人。

源码模式配置工作区是仓库根目录。安装模式默认为 `~/.config/t170-factory/workspace`，日志和运行快照默认为 `~/.local/state/t170`。启动终端会打印实际运行目录和日志目录。

## 新机器人

安装包解压后，进入包目录，使用与构建时匹配的 Python 和设备/调度 SDK 环境：

```bash
./deploy/setup_robot.sh TH-T170C-0125 --model 3 \
  --head-serial 实际头相机SN --left-serial 实际左手SN --right-serial 实际右手SN
./deploy/apply_config.sh --validate-only
./deploy/run_maintenance.sh
```

5 kg 使用 `--model 5`；底盘地址不同增加 `--chassis-host 实际IP`。首次建立新档案，不覆盖已有标定；再次运行选择已有档案，不静默重写相机信息。已存在机器人的配置可从备份恢复到持久工作区。

另开终端在同一包目录执行：

```bash
./deploy/run_web.sh
```

新档案默认使用头、左右手相机的理论值，来源在 `config/robot_models/t170c_v2_common/cameras/`。理论值从 URDF 腕部转换到所选 SDK 法兰。标准图像方向为左手原图、右手旋转 180°；本机 `cameras/realsense_cameras.yaml` 的 `image_rotation_deg` 为统一来源。No.124 装反，明确保留左 180°／右 0° 的独立配置。新机器人仍需建立自己的左/右抓取、左/右放置四份图像参考。

完成维护验证后，停止维护服务：

```bash
./deploy/run_production.sh
```

生产入口听调度分配任务。更换机器人、3/5 kg、相机、手眼或模型后重新启动；无需手工生成临时目录。

## 候选包构建

候选代码冻结并建立本地版本标签后，在匹配机器人的 Linux 环境执行：

```bash
./packaging/build_release.sh
```

生成目录为 `build_release/packages`，包含 TGZ 和 SHA256。构建脚本检查安装目录中的 Linux 动态库，缺失依赖或仍引用源码目录会停止打包。发布包不包含 C++ 源码、历史日志、旧权重及单机任务标定；Python、网页和在用权重作为运行资源安装。`release.json` 和 `environment.lock.json` 记录版本、构建环境及动态依赖解析结果。

Mac 离线测试不替代 Linux 链接和真机检查。第二台机器人部署使用已验证的二进制包及匹配环境，不需要再编译 C++。

systemd 模板读取 `/etc/t170-factory/runtime.env`，至少指定 `T170_INSTALL_ROOT`；自定义配置位置同时设置 `T170_WORKSPACE`，Python 环境不同设置 `T170_PYTHON`。升级前停止主程序，保留配置备份，再切换版本。`Restart=no` 保留，任务中断不会自动重新启动并 Home。
