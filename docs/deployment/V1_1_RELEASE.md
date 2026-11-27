# V1.1.0 构建、发布与部署

适用版本：1.1.0 候选源码，尚未冻结。先在 No.120 使用源码完成调度 SDK 1.12 联调，验证通过后再执行下面第 1 节起的标签、构建和发布操作。

## 0. 当前先用源码验证

Mac 提交并推送开发分支，本轮不创建 v1.1.0 标签：

```bash
git status --short
git add -A
git commit -m "Adapt dispatch SDK 1.12 and merge v1.1 fixes"
git push origin deployment1117
```

No.120 停止主程序和网页后，进入当前实际使用的源码目录（以下以 ~/T170-dev 为例）：

```bash
cd "$HOME/T170-dev"
conda activate human_interaction_env
./deploy/update_dev.sh
./deploy/run_maintenance.sh
```

另一个终端在相同目录执行 ./deploy/run_web.sh。网页选择 No.120，再手动进入调度。首次接入本批源码，120 的调度 SDK 头文件与动态库都必须是支持文字异常和 RETRY／SKIP／OFFLINE 的 1.12 接口版本；已经安装则不重复升级。

异常选择及现场检查见 [SDK 1.12 适配检查](DISPATCH_SDK_1_12_REVIEW.md)。本轮不解压新包，不迁移标定，不改变现场快捷参数。packaging/VERSION 中的 1.1.0 是候选版本号，单独修改它不会发布运行包。

## 1. 提交并固定源码

在 Mac 的项目仓库检查本批改动，提交后建立新标签：

```bash
git status --short
git diff --stat
git add -A
git commit -m "Prepare T170 Factory v1.1.0"
git tag -a v1.1.0 -m "T170 Factory v1.1.0"
git push origin deployment1117
git push origin v1.1.0
```

标签固定本次交付的源码。不要移动已有 v1.0.0，也不要覆盖已发布的同名运行包。以上命令在实际执行后才产生提交、标签和远程更新。

## 2. 在机器人兼容的 Linux 上构建

No.120 或 No.124 均可作为构建机，前提是架构、SDK 和视觉环境与部署机器人一致。使用独立目录构建，不影响正在调试的源码目录：

```bash
conda activate human_interaction_env
git clone --branch v1.1.0 --single-branch \
  git@github.com:SuperJack0228/T170-WheeledRobot-Factory-Auto.git \
  "$HOME/t170-build-v1.1.0"
cd "$HOME/t170-build-v1.1.0"
./packaging/build_release.sh
```

脚本检查 Linux、干净工作区和标签，然后编译、安装到全新的临时检查目录、检查动态库依赖并生成压缩包。不启动设备，也不自动发布。

ARM64 构建机的输出一般为：

```text
build_release/packages/t170-factory-1.1.0-Linux-aarch64.tar.gz
build_release/packages/t170-factory-1.1.0-Linux-aarch64.tar.gz.sha256
```

以实际文件名和 release.json 的 architecture 为准。包内不选择构建机机器人，不携带本机抓放任务参考、网页令牌或多机访问令牌。相同环境只需构建一次，再把这一份包部署到其他机器人。

## 3. No.120 从当前调试版迁入

结束当前任务，正常停止主程序和网页。将 TGZ 与 SHA256 放在机器人同一目录，校验并解压：

```bash
sha256sum -c t170-factory-1.1.0-Linux-aarch64.tar.gz.sha256
mkdir -p "$HOME/t170/releases"
tar -xzf t170-factory-1.1.0-Linux-aarch64.tar.gz -C "$HOME/t170/releases"
cd "$HOME/t170/releases/t170-factory-1.1.0-Linux-aarch64"
conda activate human_interaction_env
unset T170_WORKSPACE T170_ROOT T170_RUNTIME_DIR T170_CONFIG_DIR T170_DEPLOYMENT T170_BINARY_ROOT
./deploy/upgrade_robot.sh --from-workspace "$HOME/T170-dev"
```

最后一行填写 No.120 **刚刚实际测试的源码目录**；若使用其他路径或旧运行包，就改为那个目录。路径指向包含 config/ 的项目根目录，不能只指向 config/。源码目录不需要 release.json。

升级会复制本机档案、快捷参数、任务参考、相机信息及网页令牌；旧目录保持原样。备份位于 ~/.local/state/t170/config_backups/时间戳/，结果打印 source_kind、source_workspace、config、backup 和实际迁移项。脚本不会替机器启动。

本次已完成的现场调参优先保留。只有明确识别的历史偏置迁移和旧理论手眼父坐标声明会被修正；不整体覆盖共享配置，也不把 No.124 的标定填给 No.120。旧手眼对应的任务参考若不兼容，会校验失败并恢复目标原配置，不会伪造标定指纹。

在两个终端分别启动：

```bash
./deploy/run_maintenance.sh
```

```bash
./deploy/run_web.sh
```

本机快捷参数位于 config/robots/TH-T170C-0120/quick_params.yaml。网页保存写入这里；手改 YAML 后用 ./deploy/apply_config.sh 应用。导入时令牌一并保留，网页地址仍为 http://192.168.113.120:8080/，以启动摘要为准。

## 4. 新机器人首次部署

解压同一个包，不从其他机器人导入任务参考。以下以新机器人 No.125、3 kg 为例：

```bash
./deploy/setup_robot.sh TH-T170C-0125 --model 3 \
  --head-serial 实际头相机SN --left-serial 实际左手SN --right-serial 实际右手SN
./deploy/run_maintenance.sh
```

另开终端运行 ./deploy/run_web.sh，通过标定向导建立本机左右抓放参考。5 kg 改为 --model 5。上位机 IP 默认按编号生成 192.168.113.125；不同则增加 --web-host 实际IP。默认手相机方向为左 0°、右 180°。

详细参数位置及 SDK 安装见 [快速部署指南](V1_QUICK_START.md)。

## 5. 调度联调与自动运行

维护程序启动后，由网页手动进入调度；进入期间手动动作应被禁用。检查一次任务接收、实际执行、成功／失败反馈和退出后恢复维护状态。该环节未实机验证，尤其需确认现场 SDK 的阻塞收信能在退出时正常解除。

联调通过后停止维护主程序，使用既有自动入口：

```bash
./deploy/run_production.sh
```

该入口保持自动进入调度。已有 systemd 部署还需将 /etc/t170-factory/runtime.env 的 T170_INSTALL_ROOT 指向新包；不要让旧 T170_WORKSPACE 继续指向停用目录。

## 6. GitHub 发布、后续迭代与回退

在仓库 Releases 创建对应 v1.1.0 的发布，正文使用 packaging/RELEASE_NOTES_1.1.0.md，上传 TGZ 和 SHA256。调度未验收时可先保留为预发布；验收完成后再转正式发布。当前不需要额外建设 GitHub Packages 或 apt 软件源。

现场仍需调代码时，继续使用原来的源码循环：Mac 修改并推送 → 机器人源码目录执行 ./deploy/update_dev.sh → 启动维护验证。普通修复不必每次打包或改版本号；源码与已交付运行包是两条独立入口，运行包不会因 git pull 自动改变。

下一次稳定交付再升版、建立新标签、构建一次并分发，新包导入当前实际使用目录。只改快捷参数或重新标定不需要重新构建。

回退时停止新版主程序和网页，回到保留的旧源码目录或旧运行包启动；升级未修改旧目录。不要直接把新二进制覆盖到旧目录中。
