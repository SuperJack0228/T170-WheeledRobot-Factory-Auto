#pragma once

/**
 * @file Ti5_Seer.hpp
 * @brief 仙工（SEER）Robokit TCP 底盘控制 SDK — 唯一公开头文件。
 *
 * @par 用法
 * @code
 * #include "Ti5_Seer.hpp"
 * Seer robot("192.168.1.100");
 * if (!robot.Connect()) { ... }
 * @endcode
 *
 * @par 返回值约定（多数接口返回 int）
 * - `0`：成功
 * - `>0`：机器人协议 `ret_code`
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
 * @par 状态字段（查询结果中的整型含义）
 * - reloc_status：`0` 失败，`1` 成功，`2` 定位中，`3` 完成
 * - task_status：`0` 无，`1` 等待，`2` 运行，`3` 暂停，`4` 完成，`5` 失败，`6` 取消
 *
 * @par TCP 端口（Call / 内部连接使用）
 * 状态 `19204` · 控制 `19205` · 导航 `19206` · 配置 `19207` · 内核 `19208` · 外设 `19210`
 *
 * @note 不含辊筒 / 顶升 / 货叉等外设能力。路径与区域说明见 docs/PATH_AND_AREA.md。
 */

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

/**
 * @brief 仙工 Robokit 底盘控制客户端（速度 / 导航 / 重定位 / 音频 / IO）。
 *
 * 非线程安全跨实例共享连接；同一 `Seer` 对象内部调用已加锁。
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

  /** @brief 设置机器人地址（下次 Connect 生效）。 */
  void SetHost(std::string host);
  /**
   * @brief 设置连接与收发超时。
   * @param connect_ms TCP 连接超时（毫秒）
   * @param recv_ms    单次收发超时（毫秒）
   */
  void SetTimeoutMs(int connect_ms, int recv_ms);

  /**
   * @brief 连接状态 / 控制 / 导航端口；配置与外设端口尽力连接（失败不阻断）。
   * @return true 表示三主端口均已连通
   */
  bool Connect();
  /** @brief 断开全部端口并关闭套接字。 */
  void Disconnect();
  /** @brief 三主端口（状态/控制/导航）是否均已连接。 */
  bool IsConnected() const;

  /** @brief 最近一次调用的返回码（同接口返回值约定）。 */
  int LastCode() const { return last_code_; }
  /** @brief 最近一次错误或机器人 `err_msg`。 */
  const std::string& LastMessage() const { return last_message_; }
  /** @brief 最近一次应答 JSON 原文，字段异常时可据此排查。 */
  const std::string& LastBody() const { return last_body_; }

  // ===========================================================================
  // 常用查询（已解析出参；协议号见注释）
  // ===========================================================================

  /**
   * @brief 位姿与站点 (1004)。单位：m / rad。
   * @param[out] confidence 定位置信度；@param[out] loc_state 定位状态；
   * @param[out] in_forbidden_area 是否在禁行区
   */
  int GetLocation(double& x, double& y, double& angle,
                  std::string& current_station, std::string& last_station,
                  double& confidence, int& loc_state, bool& in_forbidden_area);
  /**
   * @brief 速度 (1005)。`vx/vy`：m/s，`w`：rad/s。
   * @param[out] is_stop 是否静止；@param[out] detect_skid 是否打滑
   */
  int GetSpeed(double& vx, double& vy, double& w, bool& is_stop, bool& detect_skid);
  /**
   * @brief 阻挡状态 (1006)。
   * @param[out] nearest_obstacles 近障条目（多为 JSON 片段字符串，便于上层自解析）
   */
  int GetBlocked(bool& blocked, bool& slowed, double& block_x, double& block_y,
                 std::vector<std::string>& nearest_obstacles);
  /**
   * @brief 电池 (1007)。`level` 通常 0.0～1.0。
   */
  int GetBattery(double& level, bool& charging, double& voltage, double& current,
                 double& battery_temp, int& battery_cycle);
  /**
   * @brief 导航/任务状态 (1020)。
   * @param[out] finished_path / unfinished_path 站点 id 序列（路径导航）
   * @param[out] target_dist 距目标剩余距离 m
   */
  int GetNavStatus(int& task_status, int& task_type, std::string& target_id,
                   double& target_x, double& target_y, double& target_angle,
                   std::vector<std::string>& finished_path,
                   std::vector<std::string>& unfinished_path, double& target_dist);
  /** @brief 任务包进度 (1110)。 */
  int GetTaskProgress(std::string& source_name, std::string& target_name,
                      double& percentage, double& distance,
                      std::vector<std::string>& task_status_list);
  /** @brief 重定位状态 (1021)，含义见文件头 reloc_status。 */
  int GetRelocStatus(int& status);
  /**
   * @brief 解析 I/O (1013)。DI/DO 的 id 与 status 分别等长一一对应。
   */
  int GetIo(std::vector<int>& di_ids, std::vector<bool>& di_status,
            std::vector<int>& do_ids, std::vector<bool>& do_status);
  /**
   * @brief 解析报警 (1050)。codes/messages/levels 等长；levels 多为 error/warning 等。
   */
  int GetAlarm(std::vector<int>& codes, std::vector<std::string>& messages,
               std::vector<std::string>& levels);
  /**
   * @brief 机器人身份摘要 (1000)：型号 / 车号 / 版本 / IP。
   */
  int GetRobotInfo(std::string& model, std::string& vehicle_id, std::string& version,
                   std::string& ip);

  // ===========================================================================
  // 规划前查询（不做几何判断；路径导航多为站点序列，见 docs/PATH_AND_AREA.md）
  // ===========================================================================

  /** @brief 当前所在区域 id 列表 (1011)，仅反映当前位置。 */
  int GetAreaIds(std::vector<std::string>& area_ids);
  /**
   * @brief 当前地图与已存地图 (1300)。
   * @param[out] map_file_names/sizes/times 来自 map_files_info，等长（可能为空）
   */
  int GetMapInfo(std::string& current_map, std::vector<std::string>& maps,
                 std::string& current_map_md5, std::vector<std::string>& map_file_names,
                 std::vector<std::string>& map_file_sizes,
                 std::vector<std::string>& map_file_times);
  /**
   * @brief 当前地图全部站点 (1301)。
   * @param[out] ids/types/xs/ys/rs/descs/spins 等长且按下标一一对应；`r` 为朝向 rad
   */
  int GetStations(std::vector<std::string>& ids, std::vector<std::string>& types,
                  std::vector<double>& xs, std::vector<double>& ys, std::vector<double>& rs,
                  std::vector<std::string>& descs, std::vector<bool>& spins);
  /**
   * @brief 路径导航预览：到目标站的站点序列 (3053)，不执行导航。
   * @param[out] length 规划路径长度 m（无则 0）
   * @warning 请在未导航时调用；导航中调用可能导致停车。
   */
  int GetTargetPathStations(const std::string& station_id, std::vector<std::string>& path,
                            double& length);
  /**
   * @brief 查询两点间路径站点序列 (1303)。
   * @param source_id 起点站点 id
   * @param target_id 终点站点 id
   * @note 请求 JSON 字段为 `source_id` / `target_id`（与实车协议一致）。
   */
  int GetPathBetweenStations(const std::string& source_id, const std::string& target_id,
                             std::vector<std::string>& path);
  /**
   * @brief 自由导航当前路径点列 (1010)；xs/ys 等长。固定路径导航下通常无意义。
   */
  int GetFreeNavPath(std::vector<double>& xs, std::vector<double>& ys);
  /**
   * @brief 下载地图内容 (4011)，用于解析 .smap（区域/曲线等）。
   * @param[out] map_data 优先取应答中的地图字段；否则为整段 LastBody
   */
  int DownloadMapData(const std::string& map_name, std::string& map_data);

  // ===========================================================================
  // 常用控制 / 导航
  // ===========================================================================

  /** @brief 开环速度 (2010)。`vx/vy`：m/s，`w`：rad/s。 */
  int SetSpeed(double vx, double vy, double w);
  /** @brief 停止开环运动 (2000)。 */
  int Stop();
  /** @brief 软急停 (6004)。`enable=true` 触发，`false` 解除（以车端为准）。 */
  int SoftEstop(bool enable);
  /** @brief 清除所有报错 (4009)。 */
  int ClearErrors();
  /** @brief 重定位到世界坐标 (2002)。单位：m / rad。 */
  int Relocate(double x, double y, double angle);
  /** @brief 回充/回原点式重定位 (2002, home=true)。 */
  int RelocateHome();
  /** @brief 取消重定位 (2004)。 */
  int CancelRelocate();
  /** @brief 固定路径导航到站点 (3051)，异步下发，不等待完成。 */
  int GoToStation(const std::string& station_id);
  /** @brief 暂停导航 (3001)。 */
  int PauseNav();
  /** @brief 继续导航 (3002)。 */
  int ResumeNav();
  /** @brief 取消导航 (3003)。 */
  int CancelNav();
  /**
   * @brief 平动固定距离 (3055)。
   * @param dist_m 距离 m；@param vx 线速度 m/s，默认 0.2
   */
  int Translate(double dist_m, double vx = 0.2);
  /** @brief 原地转动固定角度 (3056)。`angle_rad` / `vw`：rad、rad/s。 */
  int Turn(double angle_rad, double vw);
  /** @brief 切换已载入地图 (2022)。 */
  int SwitchMap(const std::string& map_name);

  // ===========================================================================
  // 音频 / IO（外设端口 19210；状态查询 IO 走 19204）
  // ===========================================================================

  /** @brief 播放音频 (6000)，`name` 为车端音频名。 */
  int PlayAudio(const std::string& name);
  /** @brief 暂停音频 (6010)。 */
  int PauseAudio();
  /** @brief 继续音频 (6011)。 */
  int ResumeAudio();
  /** @brief 停止音频 (6012)。 */
  int StopAudio();
  /** @brief 设置单个 DO (6001)。 */
  int SetDo(int id, bool status);
  /** @brief 设置继电器 (6003)。 */
  int SetRelay(int id, bool status);

  // ===========================================================================
  // 阻塞流程（内部轮询状态，注意超时）
  // ===========================================================================

  /** @brief 等待重定位完成（成功或完成态）。默认超时 60s。 */
  int WaitRelocateReady(int timeout_ms = 60000);
  /**
   * @brief 下发重定位并等待完成；不含「确认定位」。
   * @see Relocate, WaitRelocateReady
   */
  int DoRelocate(double x, double y, double angle, int timeout_ms = 60000);
  /**
   * @brief 路径导航到站点并阻塞等待结束。默认超时 600s。
   * @see GoToStation, WaitNavDone
   */
  int NavigateToStation(const std::string& station_id, int timeout_ms = 600000);
  /** @brief 等待当前导航任务结束（完成 / 失败 / 取消）。默认超时 600s。 */
  int WaitNavDone(int timeout_ms = 600000);

  // ===========================================================================
  // 协议 API（返回码同上；正文多在 LastBody，带 Raw 后缀表示自行组 JSON）
  // ===========================================================================

  /** @brief 查询机器人信息 (1000) */
  int QueryRobotInfo();
  /** @brief 查询位置 (1004) */
  int QueryLocation();
  /** @brief 查询速度 (1005) */
  int QuerySpeed();
  /** @brief 查询阻挡 (1006) */
  int QueryBlockStatus();
  /** @brief 查询电池 (1007) */
  int QueryBattery();
  /** @brief 查询所在区域 (1011) */
  int QueryArea();
  /** @brief 查询自由导航路径点 (1010) */
  int QueryPath();
  /** @brief 查询 I/O (1013) */
  int QueryIo();
  /** @brief 查询任务/导航状态 (1020) */
  int QueryTaskStatus();
  /** @brief 查询重定位状态 (1021) */
  int QueryRelocStatus();
  /** @brief 查询报警 (1050) */
  int QueryAlarm();
  /** @brief 查询任务状态包 (1110) */
  int QueryTaskPackage();
  /** @brief 查询已载入/存储地图名 (1300) */
  int QueryMap();
  /** @brief 查询站点列表 (1301) */
  int QueryStation();
  /** @brief 查询两点间路径 (1303)，`body` 为请求 JSON */
  int QueryPathBetween(const std::string& body = "");
  /** @brief 下载地图 (4011) */
  int DownloadMap(const std::string& map_name);
  /** @brief 下载地图，原始 body (4011) */
  int DownloadMapRaw(const std::string& body);
  /** @brief 重定位原始 body (2002) */
  int RelocateRaw(const std::string& body = "");
  /** @brief 开环运动原始 body (2010) */
  int MotionRaw(const std::string& body = "");
  /** @brief 路径导航到站点 (3051) */
  int GoToTarget(const std::string& station_id);
  /** @brief 路径导航原始 body (3051) */
  int GoToTargetRaw(const std::string& body);
  /**
   * @brief 获取路径导航规划路径 (3053)，不执行。
   * @warning 勿在导航过程中调用。
   */
  int GetTargetPath(const std::string& station_id);
  /** @brief 获取路径导航规划路径，原始 body (3053) */
  int GetTargetPathRaw(const std::string& body = "");
  /** @brief 圆弧运动 (3058) */
  int Circular(const std::string& body = "");
  /** @brief 启用/禁用路径 (3059) */
  int SetPathEnable(const std::string& body = "");
  /** @brief 指定路径导航 (3066) */
  int GoToTargetList(const std::string& body = "");
  /** @brief 清除指定导航路径 (3067) */
  int ClearTargetList();
  /** @brief 按任务 ID 安全清除导航 (3068) */
  int SafeClearMovements(const std::string& body = "");
  /** @brief 查询任务链状态 (3101) */
  int QueryTaskListStatus();
  /** @brief 执行预存任务链 (3106) */
  int RunTaskList(const std::string& body = "");
  /** @brief 抢占控制权 (4005) */
  int LockControl(const std::string& body = "");
  /** @brief 释放控制权 (4006) */
  int UnlockControl();
  /** @brief 运行信息复位 / 清里程 (4450) */
  int ClearOdo();
  /** @brief 设置第三方错误 (4800) */
  int SetError(const std::string& body = "");
  /** @brief 清除第三方错误 (4801) */
  int ClearError();
  /** @brief 设置第三方警告 (4802) */
  int SetWarning(const std::string& body = "");
  /** @brief 清除第三方警告 (4803) */
  int ClearWarning();
  /** @brief 播放音频原始 body (6000) */
  int PlayAudioRaw(const std::string& body = "");
  /** @brief 设置 DO 原始 body (6001) */
  int SetDoRaw(const std::string& body = "");
  /** @brief 批量设置 DO (6002) */
  int SetDos(const std::string& body = "");
  /** @brief 设置继电器原始 body (6003) */
  int SetRelayRaw(const std::string& body = "");
  /** @brief 设置充电继电器 (6005) */
  int SetChargingRelay(const std::string& body = "");
  /** @brief 上传音频 (6030) */
  int UploadAudio(const std::string& body = "");
  /** @brief 下载音频 (6031) */
  int DownloadAudio(const std::string& body = "");
  /** @brief 音频文件列表 (6033) */
  int QueryAudioList();

  /**
   * @brief 通用协议调用（未封装 API 的逃生舱）。
   * @param port   TCP 端口，如 19204
   * @param api_id 协议编号，见 docs/UNIMPLEMENTED_APIS.md
   * @param body   JSON 文本，可空
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
