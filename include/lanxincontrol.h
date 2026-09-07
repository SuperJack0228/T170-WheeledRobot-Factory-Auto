#ifndef LANXIN_CONTROL_H
#define LANXIN_CONTROL_H

#include <cstdint>
#include <string>

#include "VmrSDKApi.h"

// LanxinControl:
// - VMR 普通 SDK：Connect(ip, port) → VMR_initWithServer，常见 port=9000。
// - apiv2 RPC（topo_move_task / charge_task 等）：InitClient 使用同一 ip，端口为 rpc_port_（默认 9000，
//   与 VMR_AMR_SDK/apiv2/topo_move.cpp 一致），与 VMR 端口分开。
class LanxinControl {
 public:
  LanxinControl();
  ~LanxinControl();

  // 连接底盘服务端并初始化 SDK 句柄（仅 VMR，一般为 9000）。
  // ip: 底盘 IP；port: VMR 服务端口（通常 9000）。
  bool Connect(const std::string& ip, int port);
  // 设置 apiv2 RPC 端口（topo / 充电等），默认 9000；与 Connect 的 VMR 端口独立。
  // 若已建立过 RPC，改端口会要求下次重新 InitClient。
  void SetRpcPort(int port);
  int GetRpcPort() const;
  // 拓扑 RPC：InitClient + HasHandleRobotTaskDev（VMR_AMR_SDK 新接口，无 handle）
  bool SetTopoEndpoint(const std::string& ip, int rpc_port = 9000);
  // 释放句柄并清理内部状态（建议退出前调用）。
  void Disconnect();
  // 判断当前是否已建立 SDK 连接。
  bool IsConnected() const;

  // 1. 导航到点 VMR_moveTasks (x, y 单位米，theta 单位弧度)
  bool MoveToPose(double x, double y, double theta);
  // 2. 导航任务 VmrNavTask（可选前进/倒车/全向/无头 + 导航模式）
  bool MoveWithNavTask(double x, double y, double theta, int move_mode,
                       int navi_mode, bool forbid_rotation_on_start);
  // 3. 相对前进/后退，distance 单位米，forward=true 前进，false 后退
  bool MoveForwardOrBackward(double distance, bool forward);
  // 4. 原地旋转，deg 单位度（左正右负）
  bool RotateInPlace(double deg);
  /** 失败则重试；连续 max_attempts 次仍失败返回 false */
  bool RotateInPlaceRetry(double deg, int max_attempts = 10, int retry_interval_sec = 2);

  // 5. 拓扑导航：仅 topo_move_task + 等待结果（与 topo_move.cpp 47 行起一致）；连接须在 SetTopoEndpoint。
  // pose_name: 地图点位名；action: 0 仅移动，3 充电，4 停止充电
  bool MoveTopo(const std::string& pose_name, int32_t action = 0);
  // 取消最近一次 MoveTopo 对应的任务（若仍在进行）
  bool CancelTopoTask();

  // 充电接口（apiv2）
  bool StartChargeTask(double x, double y, double theta);
  bool CancelChargeTask();  // 取消最近一次 StartChargeTask 任务
  bool ChargeRelay(bool on);

  // 查询电量（读取回调缓存）
  // 返回 true 表示 out_battery 已填充有效数据。
  bool GetBattery(VmrBatteryInfo& out_battery) const;
  // 打印最近一次电池回调缓存。
  void PrintBattery() const;

  // 速度相关接口
  // 设置整体速度比例（0.0~1.0，1.0 为最大速度）
  bool SetSpeedFactor(double factor);
  // 开启或关闭 SDK 控速模式（true 开启，false 关闭）
  bool EnableSdkCtrlSpeed(bool enable);
  // 发送实时速度（手柄/遥操作常用），vx/vy 单位 m/s，wz 单位 rad/s
  bool SendTwist(double vx, double vy, double wz);
  // 发送 0 速度停车
  bool StopTwist();

 private:
  // 电池回调静态桥接函数（转发到对象实例）。
  static void BatteryCallbackBridge(const VmrBatteryInfo& info);
  // 电池回调实例处理函数（更新缓存）。
  void OnBattery(const VmrBatteryInfo& info);

  // 建立 apiv2（InitClient + HasHandleRobotTaskDev）；SetTopoEndpoint / EnsureRpcReady 内部用。
  bool OpenRpcClient();
  // 充电等：若尚未 RPC 则 OpenRpcClient（依赖已设置的 ip_、rpc_port_，如仅 Connect 后需 SetRpcPort）。
  bool EnsureRpcReady();
  // 轮询任务状态直到完成，返回是否成功。
  bool WaitTaskDone(const std::string& task_id) const;

 private:
  // 当前连接目标
  std::string ip_;
  int port_ = 9000;       // VMR_initWithServer
  int rpc_port_ = 9000;   // rpc_client::InitClient（与 topo_move.cpp 一致）
  // 普通 SDK 句柄（来自 VMR_Handle_Create）
  VMR_Handle vmr_handle_ = 0;

  // apiv2 RPC 状态
  bool rpc_ready_ = false;
  // 最近一次 charge_task 的任务 ID，用于取消充电
  uint64_t last_charge_task_id_ = 0;
  bool has_last_charge_task_ = false;
  // 最近一次 topo_move_task 的任务 ID
  uint64_t last_topo_task_id_ = 0;
  bool has_last_topo_task_ = false;

  // 电池回调缓存
  mutable bool has_battery_ = false;
  mutable VmrBatteryInfo battery_{};

  // 当前活动实例（用于静态回调桥接）
  static LanxinControl* g_instance_;
};

int lanxin_point_move(LanxinControl &ctrl,double *point);

void move_topo_until_ok(LanxinControl &ctrl, const std::string &pose_name,int a);

int chargeInMobileZone(LanxinControl &ctrl);
int cancelCharging(LanxinControl &ctrl);
double getBatteryLevel(LanxinControl &ctrl);


#endif
