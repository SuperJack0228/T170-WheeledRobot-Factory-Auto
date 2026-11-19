#pragma once

/**
 * @file Ti5_Seer.hpp
 * @brief 仙工（SEER）Robokit TCP 底盘控制 SDK — 唯一公开头文件。
 *
 * 协议为定长报文头 + JSON 数据区的 request/response；机器人作服务端。
 * 官方文档见飞书「Robokit API 通信协议」（`api_address.txt`）；
 * 本仓库对照见 `docs/seer_api/`（编号总表 / SDK 映射 / 字段摘录）。
 *
 * @par 用法
 * @code
 * #include "Ti5_Seer.hpp"
 * Seer robot("192.168.1.100");
 * if (!robot.Connect()) { ... }
 * double x, y, a, conf; int loc; bool forbidden;
 * std::string cur, last;
 * robot.GetLocation(x, y, a, cur, last, conf, loc, forbidden);
 * @endcode
 *
 * @par 返回值约定（多数接口返回 int）
 * - `0`：成功（车端 `ret_code` 为 0 或缺省）
 * - `>0`：机器人协议 `ret_code`（见协议附录错误码）
 * - `<0`：SDK 侧错误
 *   - `-1` 未连接 / 端口不可用
 *   - `-2` 发送失败
 *   - `-3` / `-6` 接收失败
 *   - `-4` 报文头错误
 *   - `-5` 报文过大
 *   - `-7` JSON 解析失败
 *   - `-8` 参数非法
 *   - `-9` 等待超时
 *
 * @par 常用状态枚举（出参整型）
 * - **reloc_status**（1021）：`0` FAILED 失败，`1` SUCCESS 成功（已确认），
 *   `2` RELOCING 定位中，`3` COMPLETED 匹配完成但未确认（需 2003）
 * - **task_status**（1020 导航）：`0` NONE，`1` WAITING，`2` RUNNING，
 *   `3` SUSPENDED，`4` COMPLETED，`5` FAILED，`6` CANCELED
 * - **task_type**（1020）：`0` 无导航，`1` 自由导航到坐标，`2` 自由导航到站点，
 *   `3` 路径导航到站点，`7` 平动/转动，`100` 其他
 * - **1110 task_status_list[].status** 另含：`7` OverTime，`404` NotFound
 *
 * @par TCP 端口
 * | 类别 | 端口 | 说明 |
 * |------|------|------|
 * | 状态 | 19204 | 查询（可多连接） |
 * | 控制 | 19205 | 速度 / 重定位 / 切图等（通常单连接） |
 * | 导航 | 19206 | 路径导航等（通常单连接） |
 * | 配置 | 19207 | 清错 / 地图上传下载等 |
 * | 内核 | 19208 | 本 SDK 默认不连接 |
 * | 外设 | 19210 | 音频 / DO / 软急停等 |
 *
 * @note 不含辊筒 / 顶升 / 货叉等专用能力。路径与区域见 docs/PATH_AND_AREA.md；
 *       未封装协议可用 `Call`，列表见 docs/UNIMPLEMENTED_APIS.md。
 * @note 字段与实车不一致时，以 `LastBody()` 为准。
 */

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

/**
 * @brief 仙工 Robokit 底盘控制客户端（速度 / 导航 / 重定位 / 地图 / 音频 / IO）。
 *
 * 同一 `Seer` 对象内部调用已加锁；勿跨线程共享同一连接对象而不加外部同步。
 * 拷贝构造 / 赋值已删除，请按值持有或使用智能指针管理生命周期。
 */
class Seer {
 public:
  /** @brief 默认构造，主机默认为 `127.0.0.1`，需再 `SetHost` 或构造时传入。 */
  Seer();
  /**
   * @brief 指定机器人 IP/主机名。
   * @param host 例如 `"192.168.90.194"`
   */
  explicit Seer(std::string host);
  ~Seer();

  Seer(const Seer&) = delete;
  Seer& operator=(const Seer&) = delete;

  /** @brief 设置机器人地址（下次 `Connect` 生效）。 */
  void SetHost(std::string host);
  /**
   * @brief 设置连接与收发超时。
   * @param connect_ms TCP 连接超时（毫秒）
   * @param recv_ms    单次收发超时（毫秒）
   */
  void SetTimeoutMs(int connect_ms, int recv_ms);

  /**
   * @brief 连接状态 / 控制 / 导航三主端口；配置 `19207` 与外设 `19210` 尽力连接（失败不阻断）。
   * @return true 表示三主端口均已连通
   */
  bool Connect();
  /** @brief 断开全部端口并关闭套接字。 */
  void Disconnect();
  /** @brief 三主端口（状态/控制/导航）是否均已连接。 */
  bool IsConnected() const;

  /** @brief 最近一次调用的返回码（同文件头返回值约定）。 */
  int LastCode() const { return last_code_; }
  /** @brief 最近一次错误信息或机器人应答中的 `err_msg`。 */
  const std::string& LastMessage() const { return last_message_; }
  /**
   * @brief 最近一次应答 JSON 原文。
   * @note 出参未覆盖的字段（如 `move_status_info`、`closest_target`、`create_on`）可从此解析。
   */
  const std::string& LastBody() const { return last_body_; }

  // ===========================================================================
  // 常用查询（已解析出参；括号内为协议编号）
  // ===========================================================================

  /**
   * @brief 查询机器人位置 (1004，`robot_status_loc_req`)。
   *
   * 单位：坐标 m，朝向 rad（世界坐标系）。
   *
   * @param[out] x,y,angle 当前位姿
   * @param[out] current_station 当前所在站点 id（可能为空）
   * @param[out] last_station    上一站点 id（可能为空）
   * @param[out] confidence      定位置信度，常见范围 [0,1]
   * @param[out] loc_state       定位相关状态（以车端定义为准）
   * @param[out] in_forbidden_area 是否在禁行区
   */
  int GetLocation(double& x, double& y, double& angle,
                  std::string& current_station, std::string& last_station,
                  double& confidence, int& loc_state, bool& in_forbidden_area);

  /**
   * @brief 查询机器人速度 (1005，`robot_status_speed_req`)。
   *
   * @param[out] vx,vy 机器人坐标系线速度 m/s；@param[out] w 角速度 rad/s
   * @param[out] is_stop     是否判定为静止
   * @param[out] detect_skid 是否检测到打滑
   */
  int GetSpeed(double& vx, double& vy, double& w, bool& is_stop, bool& detect_skid);

  /**
   * @brief 查询被阻挡状态 (1006，`robot_status_block_req`)。
   *
   * @param[out] blocked 是否被阻挡；@param[out] slowed 是否因近障减速
   * @param[out] block_x,block_y 阻挡相关坐标 m（无则多为 0）
   * @param[out] nearest_obstacles 近障条目；元素多为 JSON 片段字符串，便于上层自解析
   */
  int GetBlocked(bool& blocked, bool& slowed, double& block_x, double& block_y,
                 std::vector<std::string>& nearest_obstacles);

  /**
   * @brief 查询电池状态 (1007，`robot_status_battery_req`)。
   *
   * @param[out] level 电量，通常 0.0～1.0（如 0.69 表示约 69%）
   * @param[out] charging 是否在充电
   * @param[out] voltage 电压 V；@param[out] current 电流 A（放电常为负）
   * @param[out] battery_temp 电池温度 ℃；@param[out] battery_cycle 循环次数
   */
  int GetBattery(double& level, bool& charging, double& voltage, double& current,
                 double& battery_temp, int& battery_cycle);

  /**
   * @brief 查询机器人导航状态 (1020，`robot_status_task_req`)。
   *
   * 查询当前导航状态、目标站点/坐标、路径上已走与未走站点等。
   * 协议可选请求 `{"simple":true}` 只回 `task_status`；本接口固定拉完整应答。
   *
   * @param[out] task_status 见文件头：0 无 / 1 等待 / 2 运行 / 3 暂停 / 4 完成 / 5 失败 / 6 取消
   * @param[out] task_type   见文件头：0 无 / 1 自由到坐标 / 2 自由到站 / 3 路径到站 / 7 平动转动 / …
   * @param[out] target_id   目标站点 id；仅 `task_type` 为 2 或 3 时有意义
   * @param[out] target_x,y,angle 目标点（来自 `target_point`[x,y,r]）；仅自由导航到坐标（type=1）时有意义
   * @param[out] finished_path   已经过站点 id；路径导航（type=3）时有效
   * @param[out] unfinished_path 尚未经过站点 id；路径导航（type=3）时有效
   * @param[out] target_dist 距目标剩余距离 m（实车常见字段；文档未必列出）
   *
   * @note 未出参字段如 `move_status_info`、`containers`、`create_on` 见 `LastBody()`。
   * @see WaitNavDone, NavigateToStation
   */
  int GetNavStatus(int& task_status, int& task_type, std::string& target_id,
                   double& target_x, double& target_y, double& target_angle,
                   std::vector<std::string>& finished_path,
                   std::vector<std::string>& unfinished_path, double& target_dist);

  /**
   * @brief 查询机器人任务状态包 (1110，`robot_status_task_status_package_req`)。
   *
   * 缺省请求（空 body）时，车端返回「最近已完成的一个任务 + 所有未完成任务」。
   * 也可自行 `Call(19204, 1110, "{\"task_ids\":[\"...\"]}")` 按任务 id 查询。
   *
   * @param[out] source_name 当前导航任务的 source（如上一站）
   * @param[out] target_name 当前导航任务目标站 id
   * @param[out] percentage  当前路径已完成比例，常见 0.0～1.0
   * @param[out] task_status_list 任务列表摘要；每项格式为 `"status:type:task_id"`
   *             （status 含义见文件头；另可能出现 7=超时、404=未找到）
   *
   * @note 协议还有 `closest_target`（附近最近站）、`distance`（到路径投影距离）、
   *       `info`（提示文案，不作逻辑判断）等，本接口未出参，需要时读 `LastBody()`。
   */
  int GetTaskProgress(std::string& source_name, std::string& target_name, double& percentage,
                      std::vector<std::string>& task_status_list);

  /**
   * @brief 查询重定位/定位状态 (1021，`robot_status_reloc_req`)。
   *
   * @param[out] status 见文件头 reloc_status。
   *   - 为 `3`（COMPLETED）时说明匹配已完成但用户未确认，应调用 `ConfirmRelocate`（2003），
   *     成功后变为 `1`（SUCCESS），此后才宜进入自动导航等模式。
   * @see Relocate, ConfirmRelocate, DoRelocate
   */
  int GetRelocStatus(int& status);

  /**
   * @brief 查询 I/O 数据 (1013，`robot_status_io_req`)。
   *
   * @param[out] di_ids,di_status 数字输入通道 id 与电平，等长一一对应
   * @param[out] do_ids,do_status 数字输出通道 id 与电平，等长一一对应
   * @note 写 DO 请用 `SetDo` / `SetDos`（6001/6002）。
   */
  int GetIo(std::vector<int>& di_ids, std::vector<bool>& di_status,
            std::vector<int>& do_ids, std::vector<bool>& do_status);

  /**
   * @brief 查询报警状态 (1050，`robot_status_alarm_req`)。
   *
   * @param[out] codes,messages,levels 等长；文案优先取车端 `desc`（兼容 msg/message）
   * @param[out] levels 多为 error / warning 等字符串（无则用默认级别）
   */
  int GetAlarm(std::vector<int>& codes, std::vector<std::string>& messages,
               std::vector<std::string>& levels);

  /**
   * @brief 查询机器人信息摘要 (1000，`robot_status_info_req`)。
   *
   * 完整应答体很大；本接口只抽取常用身份字段。
   *
   * @param[out] model 型号（如 AMB-300）；@param[out] vehicle_id 车号
   * @param[out] version 软件/固件版本；@param[out] ip 优先 `current_ip`，回退 IP/ip
   */
  int GetRobotInfo(std::string& model, std::string& vehicle_id, std::string& version,
                   std::string& ip);

  // ===========================================================================
  // 规划前查询 / 地图管理（不做几何判断；见 docs/PATH_AND_AREA.md）
  // ===========================================================================

  /**
   * @brief 查询机器人当前所在区域 (1011，`robot_status_area_req`)。
   *
   * @param[out] area_ids 区域 id 列表；地图区域可重叠，故可为多个；也可能为空
   * @note 仅反映「当前是否在某区域内」，不能代替路径与区域相交判断。
   */
  int GetAreaIds(std::vector<std::string>& area_ids);

  /**
   * @brief 查询已载入/存储的地图 (1300，`robot_status_map_req`)。
   *
   * @param[out] current_map 当前载入地图名
   * @param[out] maps 车端已存储地图名列表
   * @param[out] current_map_md5 当前地图 MD5（无则空）
   * @param[out] map_file_names/sizes/times 来自 `map_files_info`，三者等长（可能为空）
   */
  int GetMapInfo(std::string& current_map, std::vector<std::string>& maps,
                 std::string& current_map_md5, std::vector<std::string>& map_file_names,
                 std::vector<std::string>& map_file_sizes,
                 std::vector<std::string>& map_file_times);

  /**
   * @brief 查询当前载入地图中的站点 (1301，`robot_status_station_req`)。
   *
   * 用于获取站点坐标、朝向、类型等，供规划与出生点标定。
   *
   * @param[out] ids,types,xs,ys,rs,descs,spins 等长且按下标一一对应
   * @param[out] xs,ys 世界坐标 m；@param[out] rs 朝向 rad（亦兼容 angle）
   * @param[out] spins 是否允许自旋等；实车可能为字符串 `"true"`/`"false"`，已做兼容解析
   * @note 只提供节点位姿，不能替代 `.smap` 中的路径曲线几何。
   */
  int GetStations(std::vector<std::string>& ids, std::vector<std::string>& types,
                  std::vector<double>& xs, std::vector<double>& ys, std::vector<double>& rs,
                  std::vector<std::string>& descs, std::vector<bool>& spins);

  /**
   * @brief 获取路径导航的路径 (3053，`robot_task_target_path_req`)。
   *
   * 只返回「用路径导航去目标站」所规划的站点序列，**不执行导航**。
   *
   * @param station_id 目标站点 id（请求字段 `id`）
   * @param[out] path 按顺序的路径站点 id（应答字段多为 `path`）
   *
   * @warning 请在机器人**未进行导航**时调用；导航过程中调用可能导致停车。
   * @note 实际轨迹是地图上的贝塞尔/曲线，本接口只给出站点序列，不是曲线采样点。
   */
  int GetTargetPathStations(const std::string& station_id, std::vector<std::string>& path);

  /**
   * @brief 查询任意两点之间的路径 (1303，`robot_status_path_req` 一类)。
   *
   * @param source_id 起点站点 id（请求字段 `source_id`）
   * @param target_id 终点站点 id（请求字段 `target_id`）
   * @param[out] path 站点 id 序列；实车应答常见字段为 `list`（亦兼容 `path`）
   */
  int GetPathBetweenStations(const std::string& source_id, const std::string& target_id,
                             std::vector<std::string>& path);

  /**
   * @brief 查询自由导航路径点 (1010，`robot_status_path_req`)。
   *
   * @param[out] xs,ys 世界坐标点列，等长；依次连接即为规划折线
   * @note 固定路径导航下正文常为 `path:[]`，通常无业务意义。
   */
  int GetFreeNavPath(std::vector<double>& xs, std::vector<double>& ys);

  /**
   * @brief 从机器人下载地图 (4011，`robot_config_downloadmap_req`)。
   *
   * 成功时数据区为整张 JSON 格式地图（`.smap`），可用于解析区域、AdvancedCurve 等。
   *
   * @param map_name 地图名（请求字段 `map_name`）
   * @param[out] map_data 优先取应答中的地图字段；否则回退为整段 `LastBody`
   */
  int DownloadMapData(const std::string& map_name, std::string& map_data);

  /**
   * @brief 上传地图到机器人 (4010，`robot_config_uploadmap_req`)。
   *
   * 协议要求：**JSON 数据区即为整张 `.smap` 地图**，不是 `{map_name, map_data}` 包装。
   * 地图名取自地图内 `header.mapName`。
   *
   * @param map_data 完整 `.smap` JSON 文本
   * @note 上传后一般不会自动切换为当前图，需再 `SwitchMap`，或改用 `UploadAndLoadMap`。
   */
  int UploadMapData(const std::string& map_data);

  /**
   * @brief 上传并载入地图 (2025，控制口)。
   * @param map_data 同 `UploadMapData`，整张 `.smap` JSON。
   */
  int UploadAndLoadMap(const std::string& map_data);

  /**
   * @brief 删除机器人上的地图 (4012，`robot_config_removemap_req`)。
   * @param map_name 地图名（请求字段 `map_name`）
   * @note 通常不能删除当前已载入地图；先 `SwitchMap` 到其它图再删。
   */
  int RemoveMap(const std::string& map_name);

  // ===========================================================================
  // 常用控制 / 导航
  // ===========================================================================

  /**
   * @brief 开环运动 (2010，`robot_control_motion_req`)。
   * @param vx,vy 机器人坐标系线速度 m/s；@param w 角速度 rad/s
   * @note 开环持续下发直至 `Stop` 或其它运动/导航打断。
   */
  int SetSpeed(double vx, double vy, double w);

  /** @brief 停止开环运动 (2000，`robot_control_stop_req`)。 */
  int Stop();

  /**
   * @brief 软急停 (6004)。
   * @param enable `true` 触发软急停，`false` 解除（以车端实现为准）
   */
  int SoftEstop(bool enable);

  /** @brief 清除机器人当前所有报错 (4009)。 */
  int ClearErrors();

  /**
   * @brief 重定位 (2002，`robot_control_reloc_req`)。
   *
   * 给出世界坐标系中的近似位姿，由车端在附近做匹配。
   *
   * @param x,y 世界坐标 m；@param angle 朝向 rad（缺省角度时全域搜角更慢，本接口始终带 angle）
   * @note 只下发请求，不等待、不确认。完整流程请用 `DoRelocate`。
   * @note 不可所有参数都缺省；回原点请用 `RelocateHome`（`home=true`）。
   */
  int Relocate(double x, double y, double angle);

  /**
   * @brief 在已配置的 robotHome 上重定位 (2002，`home=true`)。
   *
   * 若 `home` 为 true，则 x/y/angle 无效；从 NetProtocol 中的 robotHome1～5 重定位。
   * 未配置 home 时车端可能不做任何操作。
   *
   * @note 只下发。完整流程请用 `DoRelocateHome`。车须已在 home 附近。
   */
  int RelocateHome();

  /**
   * @brief 确认定位正确 (2003，`robot_control_comfirmloc_req`)。
   *
   * 无 JSON 数据区。当 `reloc_status==3`（COMPLETED）时调用，使状态变为 `1`（SUCCESS）。
   * 对齐 Roboshop「确认定位」；确认前不宜当作定位已可靠。
   */
  int ConfirmRelocate();

  /**
   * @brief 取消重定位 (2004，`robot_control_cancelreloc_req`)。
   *
   * 仅当正在重定位（`reloc_status==2`）时有效：取消本次重定位，位姿回到下发前，
   * 状态变为 `3`（COMPLETED）。当前未在重定位时本指令无作用。
   */
  int CancelRelocate();

  /**
   * @brief 路径导航到站点 (3051，`robot_task_gotarget_req`)。
   *
   * 给定目标站点，车端沿地图贝塞尔路径运行，途经中间站但不停留。
   * 本接口**异步下发**，立即返回，不等待到达；请用 `GetNavStatus` / `WaitNavDone` 轮询。
   *
   * @param station_id 目标站点 id（请求字段 `id`）
   * @note 同一时刻通常只能有一个导航任务，新任务会打断旧任务。
   * @see NavigateToStation, GetNavStatus
   */
  int GoToStation(const std::string& station_id);

  /** @brief 暂停当前导航 (3001)。 */
  int PauseNav();
  /** @brief 继续当前导航 (3002)。 */
  int ResumeNav();
  /** @brief 取消当前导航 (3003)。 */
  int CancelNav();

  /**
   * @brief 平动 (3055)：以固定速度直线运动固定距离。
   * @param dist_m 距离 m；@param vx 线速度 m/s，默认 0.2
   */
  int Translate(double dist_m, double vx = 0.2);

  /**
   * @brief 转动 (3056)：以固定角速度旋转固定角度。
   * @param angle_rad 角度 rad；@param vw 角速度 rad/s
   */
  int Turn(double angle_rad, double vw);

  /**
   * @brief 切换载入的地图 (2022，`robot_control_loadmap_req`)。
   * @param map_name 要载入的地图名
   * @note 切图后通常需要重新定位（重定位 / 出生点流程）。
   */
  int SwitchMap(const std::string& map_name);

  // ===========================================================================
  // 音频 / IO（外设端口 19210；读 IO 走状态口 19204）
  // ===========================================================================

  /** @brief 播放音频 (6000)；`name` 为车端已有音频资源名（可用 `QueryAudioList` 查询）。 */
  int PlayAudio(const std::string& name);
  /** @brief 暂停音频 (6010)。 */
  int PauseAudio();
  /** @brief 继续音频 (6011)。 */
  int ResumeAudio();
  /** @brief 停止音频 (6012)。 */
  int StopAudio();
  /**
   * @brief 设置单个 DO (6001)。
   * @param id DO 编号；@param status 电平 true/false
   */
  int SetDo(int id, bool status);
  /**
   * @brief 设置继电器 (6003)。
   * @param id 继电器编号；@param status 吸合/断开
   */
  int SetRelay(int id, bool status);

  // ===========================================================================
  // 阻塞流程（内部轮询状态，注意超时）
  // ===========================================================================

  /**
   * @brief 等待重定位匹配结束（`reloc_status` 为 1 或 3）。默认超时 60s。
   * @note 若得到 3，表示匹配完成但尚未确认，完整流程应再 `ConfirmRelocate`；
   *       `DoRelocate` / `DoRelocateHome` 已包含确认步骤。
   */
  int WaitRelocateReady(int timeout_ms = 60000);

  /**
   * @brief 完整重定位：下发 2002 → 等待匹配 → 若仍为完成态则确认 2003。
   *
   * 对齐 Roboshop 手动重定位。典型「出生点」用法：在地图上标定固定位姿，
   * 把车推到该物理位置并对准朝向后调用 `DoRelocate(birth_x, birth_y, birth_angle)`。
   * 初值偏差过大易导致匹配失败（status=0）。
   *
   * @see Relocate, WaitRelocateReady, ConfirmRelocate
   */
  int DoRelocate(double x, double y, double angle, int timeout_ms = 60000);

  /**
   * @brief 完整回原点重定位：`RelocateHome` → 等待 → 确认。
   * @note 车须已在配置的 robotHome 附近，且车端已配置 home。
   */
  int DoRelocateHome(int timeout_ms = 60000);

  /**
   * @brief 路径导航到站点并阻塞等待结束（完成 / 失败 / 取消）。默认超时 600s。
   * @see GoToStation, WaitNavDone, GetNavStatus
   */
  int NavigateToStation(const std::string& station_id, int timeout_ms = 600000);

  /**
   * @brief 等待当前导航任务结束。
   *
   * 内部轮询 `GetNavStatus` 的 `task_status`：4 完成返回成功；5/6 失败或取消返回错误码。
   * 默认超时 600s。
   */
  int WaitNavDone(int timeout_ms = 600000);

  // ===========================================================================
  // 协议 API（返回码同上；正文多在 LastBody；带 Raw 表示自行组 JSON）
  // ===========================================================================

  /** @brief 查询机器人信息 (1000)；完整正文见 `LastBody`，摘要用 `GetRobotInfo`。 */
  int QueryRobotInfo();
  /** @brief 查询位置 (1004)；解析版见 `GetLocation`。 */
  int QueryLocation();
  /** @brief 查询速度 (1005)；解析版见 `GetSpeed`。 */
  int QuerySpeed();
  /** @brief 查询阻挡 (1006)；解析版见 `GetBlocked`。 */
  int QueryBlockStatus();
  /** @brief 查询电池 (1007)；解析版见 `GetBattery`。 */
  int QueryBattery();
  /** @brief 查询所在区域 (1011)；解析版见 `GetAreaIds`。 */
  int QueryArea();
  /** @brief 查询自由导航路径点 (1010)；解析版见 `GetFreeNavPath`。 */
  int QueryPath();
  /** @brief 查询 I/O (1013)；解析版见 `GetIo`。 */
  int QueryIo();
  /** @brief 查询导航状态 (1020)；解析版见 `GetNavStatus`。 */
  int QueryTaskStatus();
  /** @brief 查询重定位状态 (1021)；解析版见 `GetRelocStatus`。 */
  int QueryRelocStatus();
  /** @brief 查询报警 (1050)；解析版见 `GetAlarm`。 */
  int QueryAlarm();
  /** @brief 查询任务状态包 (1110)；解析版见 `GetTaskProgress`。 */
  int QueryTaskPackage();
  /** @brief 查询已载入/存储地图 (1300)；解析版见 `GetMapInfo`。 */
  int QueryMap();
  /** @brief 查询站点列表 (1301)；解析版见 `GetStations`。 */
  int QueryStation();
  /**
   * @brief 查询两点间路径 (1303)，自行组请求 JSON。
   * @param body 例如 `{"source_id":"AP33","target_id":"AP25"}`；解析版见 `GetPathBetweenStations`
   */
  int QueryPathBetween(const std::string& body = "");

  /** @brief 上传地图 (4010)，`map_data` 为整张 .smap JSON；高层见 `UploadMapData`。 */
  int UploadMap(const std::string& map_data);
  /** @brief 上传地图，原始 body (4010)。 */
  int UploadMapRaw(const std::string& body);
  /** @brief 下载地图 (4011)；解析内容见 `DownloadMapData`。 */
  int DownloadMap(const std::string& map_name);
  /** @brief 下载地图，原始 body (4011)。 */
  int DownloadMapRaw(const std::string& body);
  /** @brief 删除地图，原始 body (4012)；高层见 `RemoveMap`。 */
  int RemoveMapRaw(const std::string& body);
  /** @brief 上传并载入地图，原始 body (2025)；高层见 `UploadAndLoadMap`。 */
  int UploadAndLoadMapRaw(const std::string& body);

  /**
   * @brief 重定位原始 body (2002)。
   * @param body 例如 `{"x":1.0,"y":2.0,"angle":0}` 或 `{"home":true}`
   */
  int RelocateRaw(const std::string& body = "");
  /** @brief 开环运动原始 body (2010)。 */
  int MotionRaw(const std::string& body = "");

  /** @brief 路径导航到站点 (3051)，同 `GoToStation`。 */
  int GoToTarget(const std::string& station_id);
  /** @brief 路径导航原始 body (3051)，可带 angle / 速度等扩展字段。 */
  int GoToTargetRaw(const std::string& body);

  /** @brief 圆弧运动 (3058)，自行组 JSON。 */
  int Circular(const std::string& body = "");
  /** @brief 启用/禁用路径 (3059)，自行组 JSON。 */
  int SetPathEnable(const std::string& body = "");
  /** @brief 指定路径导航 (3066)，按站点序列导航，自行组 JSON。 */
  int GoToTargetList(const std::string& body = "");
  /** @brief 清除指定导航路径 (3067)。 */
  int ClearTargetList();
  /** @brief 按任务 ID 安全清除导航 (3068)，自行组 JSON。 */
  int SafeClearMovements(const std::string& body = "");
  /** @brief 查询任务链状态 (3101)。 */
  int QueryTaskListStatus();
  /** @brief 执行预存任务链 (3106)，自行组 JSON。 */
  int RunTaskList(const std::string& body = "");

  /** @brief 抢占/锁定控制权 (4005)，自行组 JSON。 */
  int LockControl(const std::string& body = "");
  /** @brief 释放控制权 (4006)。 */
  int UnlockControl();
  /** @brief 运行信息复位 / 清里程 (4450)。 */
  int ClearOdo();
  /** @brief 设置第三方错误 (4800)，自行组 JSON。 */
  int SetError(const std::string& body = "");
  /** @brief 清除第三方错误 (4801)。 */
  int ClearError();
  /** @brief 设置第三方警告 (4802)，自行组 JSON。 */
  int SetWarning(const std::string& body = "");
  /** @brief 清除第三方警告 (4803)。 */
  int ClearWarning();

  /** @brief 播放音频原始 body (6000)。 */
  int PlayAudioRaw(const std::string& body = "");
  /** @brief 设置 DO 原始 body (6001)。 */
  int SetDoRaw(const std::string& body = "");
  /** @brief 批量设置 DO (6002)，自行组 JSON。 */
  int SetDos(const std::string& body = "");
  /** @brief 设置继电器原始 body (6003)。 */
  int SetRelayRaw(const std::string& body = "");
  /** @brief 设置充电继电器 (6005)，自行组 JSON。 */
  int SetChargingRelay(const std::string& body = "");
  /** @brief 上传音频 (6030)，自行组 JSON。 */
  int UploadAudio(const std::string& body = "");
  /** @brief 下载音频 (6031)，自行组 JSON。 */
  int DownloadAudio(const std::string& body = "");
  /** @brief 查询音频文件列表 (6033)。 */
  int QueryAudioList();

  /**
   * @brief 通用协议调用（未封装 API 的逃生舱）。
   *
   * @param port   TCP 端口，如 19204 / 19205 / 19206 / 19207 / 19210
   * @param api_id 协议编号，见 docs/UNIMPLEMENTED_APIS.md 与官方 API 列表
   * @param body   JSON 文本，可空；应答在 `LastBody()`
   * @warning 无类型校验，调用前请核对端口与字段名。
   */
  int Call(int port, int api_id, const std::string& body = "");

 private:
  int* FdPtr(int port);
  int SocketForPort(int port) const;
  bool EnsurePort(int port);
  bool ConnectOne(int port, int& fd);
  void CloseAll();
  int CallUnlocked(int port, int api_id, const std::string& body);
  int SetLast(int code, std::string msg = {});
  /** 等待匹配结束后按需发送 2003 确认。 */
  int FinishRelocateWithConfirm(int timeout_ms);

  std::string host_ = "127.0.0.1";
  int connect_timeout_ms_ = 3000;
  int recv_timeout_ms_ = 5000;
  int state_fd_ = -1;
  int control_fd_ = -1;
  int nav_fd_ = -1;
  int config_fd_ = -1;
  int other_fd_ = -1;
  uint16_t seq_ = 1;
  int last_code_ = 0;
  std::string last_message_;
  std::string last_body_;
  mutable std::mutex mutex_;
};
