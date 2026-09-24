# T170C Web 调试台

该页面运行在机器人 Ubuntu 主机上，浏览器只是显示与下发高层命令。相机、CAN、夹爪和机械臂始终由本机 `t170c_debug` C++ 进程独占。

## 部署后启动

```bash
cd ~/T170Clean
cmake --build build_robot -j2
bash tools/start_web_debug.sh
```

脚本会先执行 `sudo -v`，按提示输入机器人用户密码即可；之后 SocketCAN 初始化不会在日志管道中隐藏密码提示。

启动完成后终端会打印带随机访问令牌的地址，例如：

```text
http://192.168.125.171:8080/?token=...
```

在同一局域网电脑的浏览器中打开该地址。不要把 8080 端口映射到公网。

如果 C++ 服务已经在另一个终端运行，可以只启动网页代理：

```bash
python3 web_debug/server.py
```

固定访问令牌：

```bash
export T170_WEB_TOKEN='请换成足够长的随机字符串'
bash tools/start_web_debug.sh
```

## 页面模块

- 系统总览：C++ 服务、CAN、视觉管线、串口、夹爪、TCP、七关节角和腰部同步状态。
- 视觉检测：头/左右手相机单次或连续 YOLO 检测、头相机 ArUco 检测、原图/标注图和坐标结果。
- 动作流程：状态检查、只检测不运动、现有 home、完整抓取。
- 夹爪调试：左右或双侧张开、Soft 力控夹取和反馈。
- 参数配置：编辑 `config/move_box_params.yaml`，保存时备份并让 C++ 校验；失败自动恢复。
- 运行日志：合并 Web 服务日志、托管启动的 C++ 输出和现有 `startup.log`。

## 安全约束

- 同一时间只接受一个硬件动作；忙碌期间其他动作立即返回，不会排队后突然执行。
- STOP 绕过动作互斥锁，但仍依赖进程、操作系统和 CAN 通信正常，不能替代实体急停。
- `home` 当前仍是现有实现，页面明确标记其轨迹待优化。
- 页面不会把实时关节伺服数据经浏览器传输；浏览器只发送离散高层命令。
- C++ 控制端口继续只监听 `127.0.0.1:8099`，局域网只开放带令牌的 Web 端口。

## 接口

Web API 使用请求头 `X-T170-Token` 鉴权。主要端点：

- `GET /api/health`
- `GET /api/status`
- `GET /api/images`
- `GET /api/image/latest?slot=head&kind=processed`
- `GET /api/config`
- `GET /api/logs?since=0`
- `POST /api/command`
- `POST /api/config`

危险命令除令牌外还必须在 JSON 中携带 `confirm`，且值必须等于命令名。网页会通过安全确认弹窗生成该字段。
