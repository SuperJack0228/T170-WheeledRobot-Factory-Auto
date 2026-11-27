# V1.0.0 发布与首次部署

历史版本操作记录。当前 V1.1.0 请使用 [V1.1.0 发布与部署](V1_1_RELEASE.md)，避免误用旧标签和包名。

适用版本：1.0.0。操作顺序为：提交源码 → 建立版本标签 → Linux 编译打包 → GitHub Release 分发 → 新机解压建档 → 维护标定与验证 → 生产运行。

本地版本文件已设为 1.0.0，发布说明位于 `packaging/RELEASE_NOTES_1.0.0.md`。提交、标签、Linux 运行包和 GitHub Release 是后续独立操作；修改版本文件本身不会生成或发布运行包。

## 1 这次交付什么

| 内容 | 用途 |
|---|---|
| GitHub 分支 | 日常开发源码 |
| `v1.0.0` 标签 | 固定本次发布对应的源码提交 |
| `t170-factory-1.0.0-Linux-aarch64.tar.gz` | 给机器人使用的已编译运行包 |
| 对应 `.tar.gz.sha256` | 检查下载和拷贝后的文件是否完整 |
| GitHub Release | 展示发布说明并提供运行包下载 |

下面沿用现有 Linux aarch64 运行包命名；构建机和机器人必须是兼容架构，以实际产物文件名为准。x86_64 Linux 电脑不能通过当前原生构建脚本直接生成 aarch64 包。

程序、动态库、Python、网页、权重和实际配置目录随包提供。整个 Conda 环境、系统驱动及 `/opt` 下的设备／调度 SDK 不包含在 TGZ 中。新机使用与已验证机器人匹配的系统镜像、SDK 和视觉环境，包括依赖路径。`environment.lock.json` 是环境版本记录，不是自动创建 Conda 环境的安装脚本。

## 2 在 Mac 提交并固定版本

在当前本地仓库执行：

```bash
cd /Users/superjack/Ti5/T170-field-review-20261111
git status --short
git diff --stat
git add -A
git commit -m "Release T170 Factory v1.0.0"
git tag -a v1.0.0 -m "T170 Factory v1.0.0"
git push origin deployment1117
git push origin v1.0.0
```

本次使用的实际开发分支为 `deployment1117`，不要求先合并到 main 才能打包。标签固定提交，构建机使用标签获取相同代码。不要移动或复用旧 rc 标签。

打包脚本要求工作区干净、`packaging/VERSION` 为 `1.0.0`，并且当前提交具有 `v1.0.0` 标签。提交后又改源码，需要重新安排版本提交，不能拿未提交修改生成同名交付包。

## 3 在匹配机器人的 Linux 环境构建

推荐在之前能成功打包的机器人或同架构构建机上使用独立源码目录。不要为了获得干净目录而重置机器人正在使用的现场源码和配置。

```bash
git clone --branch v1.0.0 --single-branch \
  https://github.com/SuperJack0228/T170-WheeledRobot-Factory-Auto.git \
  "$HOME/t170-build-v1.0.0"
cd "$HOME/t170-build-v1.0.0"
conda activate human_interaction_env
uname -m
./packaging/build_release.sh -DHUMAN_INTERACTION_ENV_ROOT="$CONDA_PREFIX"
```

该目录首次不存在时使用上述 clone。已有专用源码目录也可以在工作区干净的前提下，执行 `git fetch origin --tags`、`git switch --detach v1.0.0` 后构建。

脚本依次执行 Release 编译、安装目录生成、动态依赖检查和 CPack 压缩。它不会启动机器人，也不会自动上传 GitHub。输出位于：

```text
build_release/packages/
├── t170-factory-1.0.0-Linux-aarch64.tar.gz
└── t170-factory-1.0.0-Linux-aarch64.tar.gz.sha256
```

如果使用其他兼容架构，文件名中的架构会相应变化。保留本次 TGZ、校验文件、`release.json` 与 `environment.lock.json`；后二者已包含在 TGZ 内。

## 4 发布到 GitHub Release

### 网页操作

1. 打开仓库的 Releases 页面。
2. 点击 Draft a new release，选择已经推送的 `v1.0.0` 标签。
3. 标题填写 `T170 Factory v1.0.0`，正文使用 `packaging/RELEASE_NOTES_1.0.0.md`。
4. 将 Linux 生成的 TGZ 和 SHA256 作为附件上传。可以在构建机浏览器上传，也可以先复制回 Mac 再上传。
5. 尚在进行本批真机确认时保存草稿；确认后发布，正式版不勾选 pre-release。

机器人下载的是手工上传的 `t170-factory-...tar.gz`。GitHub 自动提供的 Source code ZIP／tar.gz 是源码归档，不能作为已编译运行包直接启动。

已安装 GitHub CLI 时，可在 Linux 源码仓库根目录先创建附带产物的草稿：

```bash
gh auth login
gh release create v1.0.0 \
  build_release/packages/t170-factory-1.0.0-Linux-aarch64.tar.gz \
  build_release/packages/t170-factory-1.0.0-Linux-aarch64.tar.gz.sha256 \
  --repo SuperJack0228/T170-WheeledRobot-Factory-Auto \
  --verify-tag --draft \
  --title "T170 Factory v1.0.0" \
  --notes-file packaging/RELEASE_NOTES_1.0.0.md
```

`--verify-tag` 要求远程标签已经存在，避免从默认分支自动创建不同的标签。`gh auth login` 首次登录使用，私有仓库需要相应访问权限。最后可在网页点击发布。

本地离线编译和 54 项回归检查已完成。本批新增的偏置清理、旧视觉分支删除及维护组切换还需在真机确认：至少核对当前组对应站点，并回归一次料盘抓放和传送带抓放流程，再结束本次交付验证。

## 5 新机器人下载运行包

以下按新机与已验证机器人的系统、SDK、视觉环境已配好一致执行。新机不需要下载完整源码，也不需要重新编译 C++。

### 下载

在新机器人或现场电脑的浏览器进入仓库 Releases，选择 `v1.0.0`，下载 TGZ 和 SHA256，将两份文件放入机器人 `~/t170/downloads/`。机器人无法直接访问 GitHub 时，可用 U 盘或局域网拷贝这两个文件。

已安装并登录 gh 的终端也可以执行：

```bash
mkdir -p "$HOME/t170/downloads"
gh release download v1.0.0 \
  --repo SuperJack0228/T170-WheeledRobot-Factory-Auto \
  --pattern 't170-factory-1.0.0-Linux-aarch64.tar.gz' \
  --pattern 't170-factory-1.0.0-Linux-aarch64.tar.gz.sha256' \
  --dir "$HOME/t170/downloads"
```

发布前的草稿不作为普通部署下载入口，发布后按本节分发。

### 校验并解压

在新机器人终端执行：

```bash
cd "$HOME/t170/downloads"
sha256sum -c t170-factory-1.0.0-Linux-aarch64.tar.gz.sha256
mkdir -p "$HOME/t170/releases"
tar -xzf t170-factory-1.0.0-Linux-aarch64.tar.gz -C "$HOME/t170/releases"
cd "$HOME/t170/releases/t170-factory-1.0.0-Linux-aarch64"
cat release.json
```

校验应显示 `OK`。解压后，`bin/`、`config/`、`deploy/` 都在当前目录。后续启动和修改配置均在这个包目录进行。

## 6 新机器人建档

示例为全新 No.121、3 kg 型号；把编号和相机 SN 换成实际值。所有命令在解压后的包目录执行：

```bash
conda activate human_interaction_env
unset T170_WORKSPACE T170_ROOT T170_RUNTIME_DIR T170_CONFIG_DIR T170_DEPLOYMENT T170_BINARY_ROOT
./deploy/setup_robot.sh TH-T170C-0121 --model 3 \
  --head-serial 实际头相机SN \
  --left-serial 实际左手相机SN \
  --right-serial 实际右手相机SN
./deploy/apply_config.sh --validate-only
```

5 kg 使用 `--model 5`。底盘地址与默认 `192.168.192.5` 不同，增加 `--chassis-host 实际IP`。`unset` 清除旧终端遗留的覆盖路径，使本次使用当前包内配置。

首次建档会创建本机配置并选择该档案。新机默认使用理论头／手眼，左手原图、右手旋转 180°；本机任务标定另行建立。

## 7 启动维护与网页

终端一：

```bash
cd "$HOME/t170/releases/t170-factory-1.0.0-Linux-aarch64"
conda activate human_interaction_env
./deploy/run_maintenance.sh
```

保持此终端运行。另开终端二：

```bash
cd "$HOME/t170/releases/t170-factory-1.0.0-Linux-aarch64"
conda activate human_interaction_env
./deploy/run_web.sh
```

从现场电脑浏览器访问网页终端打印的地址，将 `<机器人IP>` 换为该机 IP，保留打印的 token。默认端口为 8080。

在网页选择当前维护机床组；例如第 2 组显示 AP6、AP9、AP8、AP10。通过标定向导建立本机左右抓取、左右放置参考，执行单步抓放验证。切组只应用站点配置，点击动作才执行导航和运动。

常用文件：

| 内容 | 包内路径 |
|---|---|
| 本机常调参数，包括抓放速度、人工 XYZ、高度参考 | `config/robots/TH-T170C-0121/quick_params.yaml` |
| 相机 SN 与画面方向 | `config/robots/TH-T170C-0121/cameras/realsense_cameras.yaml` |
| 本机抓放参考 | `config/calibration/TH-T170C-0121/tasks/` |
| 维护组别与站点 | `config/sites/current_factory/maintenance_workcells.yaml` |
| 五组机床信息码与高度 | `config/sites/current_factory/conveyor_heights.yaml` |

网页保存永久写入本机文件。手工修改后统一执行 `./deploy/apply_config.sh`；是否热更新由返回结果说明。

## 8 进入生产运行

维护验证完成后，正常退出维护主程序并等待退出完成，再在主程序终端执行：

```bash
./deploy/run_production.sh
```

此入口自动进入生产调度流程，作业点使用调度下发值。网页可保持运行观察；模式页面以机器人服务返回的实际模式为准。

手动运行与开机自启是两个步骤。首次先完成手动部署；需要开机自启时再安装现有 systemd 模板，并按实际用户、Python 和包路径设置 `/etc/t170-factory/runtime.env`。不要直接把模板中的 `ti5robot` 用户套到其他系统用户上。

## 9 已有 No.120 升级

No.120 已有 rc.3 配置和标定，不走全新建档。停主程序与网页、解压 V1.0.0 后，在新包根目录执行：

```bash
conda activate human_interaction_env
unset T170_WORKSPACE T170_ROOT T170_RUNTIME_DIR T170_CONFIG_DIR T170_DEPLOYMENT T170_BINARY_ROOT
./deploy/upgrade_robot.sh --from-workspace "$HOME/.config/t170-factory/workspace"
```

原本使用自定义工作区时填写实际旧目录。升级导入本机身份、SN、标定及参数到新包内 `config/`，同时执行已确认的偏置和维护站点迁移；旧目录和备份保留。完整行为见 [运行包升级说明](PACKAGE_UPGRADE.md)。

## 10 后续版本和 APT

日常仍向开发分支推送源码。修复需要交付机器人时，版本改为 1.0.1，提交并建立 `v1.0.1` 标签，重新构建并发布新包；新包导入旧包内的本机配置。已经发布的 v1.0.0 和同名文件保持固定。

当前采用 GitHub Release + TGZ，暂不要求 GitHub Packages、Docker 或 APT 仓库。`sudo apt update` 更新系统软件源的索引，不会更新这份 TGZ。公司 SDK 可由已配置的公司软件源安装，例如：

```bash
sudo apt update
sudo apt install ti5-device-sdk ti5-jindi-faactory-robot-client
```

已按同一镜像配好的机器人可沿用已验证版本；全新环境的 SDK 版本以运行包环境清单为依据。首次安装不能使用 `--only-upgrade`。

要让本项目也支持 `sudo apt install t170-factory`，还需额外制作 DEB、声明依赖并维护 APT 分发源；这不是当前 TGZ 发布的必要步骤。

## 官方操作参考

- [GitHub Release 网页发布](https://docs.github.com/en/repositories/releasing-projects-on-github/managing-releases-in-a-repository)
- [gh release create](https://cli.github.com/manual/gh_release_create)
- [gh release download](https://cli.github.com/manual/gh_release_download)
- [Ubuntu 软件包管理](https://documentation.ubuntu.com/server/how-to/software/package-management/)
