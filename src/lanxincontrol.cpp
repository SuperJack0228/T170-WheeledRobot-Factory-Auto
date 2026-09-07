#include "lanxincontrol.h"

#include <chrono>
#include <ctime>
#include <exception>
#include <iostream>
#include <thread>

#include <unistd.h>
#include "client/common/client_common.h"
#include "robot_task.h"

LanxinControl* LanxinControl::g_instance_ = nullptr;

LanxinControl::LanxinControl() { g_instance_ = this; }

LanxinControl::~LanxinControl() { Disconnect(); }

bool LanxinControl::Connect(const std::string& ip, int port) {
  ip_ = ip;
  port_ = port;
  int ret = VMR_initWithServer(ip_.c_str(), port_);
  if (ret != 0) {
    std::cerr << "VMR_initWithServer failed, ret=" << ret << std::endl;
    return false;
  }

  vmr_handle_ = VMR_Handle_Create();
  if (!vmr_handle_) {
    std::cerr << "VMR_Handle_Create failed" << std::endl;
    return false;
  }

  VMR_registerBatteryCallback(static_cast<int>(vmr_handle_),
                              BatteryCallbackBridge);
  std::cout << "Connected to " << ip_ << ":" << port_ << " (VMR), apiv2 RPC port: "
            << rpc_port_ << std::endl;
  return true;
}

void LanxinControl::SetRpcPort(int port) {
  if (port <= 0) {
    return;
  }
  if (rpc_port_ != port) {
    rpc_ready_ = false;
  }
  rpc_port_ = port;
}

int LanxinControl::GetRpcPort() const { return rpc_port_; }

bool LanxinControl::OpenRpcClient() {
  if (ip_.empty()) {
    std::cerr << "OpenRpcClient: ip is empty" << std::endl;
    return false;
  }
  try {
    rpc_client::ClientConfig conf;
    conf.ip = ip_;
    conf.port = rpc_port_;
    conf.connect_timeout_ms = 3000;
    conf.call_timeout_ms = 60000;
    conf.error_callback = [](const std::error_code& ec,
                             const std::string& message) {
      std::cerr << "RPC error: " << ec.message() << ", " << message
                << std::endl;
    };
    if (!rpc_client::InitClient(conf)) {
      std::cerr << "InitClient failed" << std::endl;
      rpc_ready_ = false;
      return false;
    }
    if (!robot_task::HasHandleRobotTaskDev()) {
      std::cerr << "no robot_task device (HasHandleRobotTaskDev=false)" << std::endl;
      rpc_ready_ = false;
      return false;
    }
    rpc_ready_ = true;
    std::cout << "RPC connected " << ip_ << ":" << rpc_port_
              << " robot_task=available" << std::endl;
    return true;
  } catch (const std::exception& e) {
    std::cerr << "OpenRpcClient 异常: " << e.what() << std::endl;
    rpc_ready_ = false;
    return false;
  }
}

bool LanxinControl::SetTopoEndpoint(const std::string& ip, int rpc_port) {
  if (ip.empty()) {
    return false;
  }
  ip_ = ip;
  SetRpcPort(rpc_port);
  return OpenRpcClient();
}

void LanxinControl::Disconnect() {
  if (vmr_handle_) {
    VMR_Handle_Destroy(vmr_handle_);
    vmr_handle_ = 0;
  }
  rpc_ready_ = false;
  has_last_charge_task_ = false;
  has_last_topo_task_ = false;
}

bool LanxinControl::IsConnected() const { return vmr_handle_ != 0; }

bool LanxinControl::WaitTaskDone(const std::string& task_id) const {
  while (true) {
    VmrTaskResult r = VMR_checkTaskStatus(vmr_handle_, task_id.c_str());
    if (r.task_result == -1) {
      std::cout << "task running..." << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
      continue;
    }
    std::cout << "task_result=" << r.task_result << std::endl;
    return r.task_result == 0;
  }
}

bool LanxinControl::MoveToPose(double x, double y, double theta) {
  VmrPose pose;
  pose.x = x;
  pose.y = y;
  pose.theta = theta;
  std::string task_id = VMR_moveTasks(vmr_handle_, pose);
  if (task_id.empty()) {
    std::cerr << "VMR_moveTasks failed" << std::endl;
    return false;
  }
  std::cout << "move task_id=" << task_id << std::endl;
  return WaitTaskDone(task_id);
}

bool LanxinControl::MoveWithNavTask(double x, double y, double theta,
                                    int move_mode, int navi_mode,
                                    bool forbid_rotation_on_start) {
  VmrNavTask task;
  task.target_pose.x = x;
  task.target_pose.y = y;
  task.target_pose.theta = theta;
  task.move_mode = static_cast<VmrMoveMode>(move_mode);
  task.navi_mode = static_cast<VmrNaviMode>(navi_mode);
  task.forbid_rotation_on_start = forbid_rotation_on_start;

  std::string task_id = VMR_moveTasks(vmr_handle_, task);
  if (task_id.empty()) {
    std::cerr << "VMR_moveTasks(VmrNavTask) failed" << std::endl;
    return false;
  }
  std::cout << "nav task_id=" << task_id << std::endl;
  return WaitTaskDone(task_id);
}

bool LanxinControl::MoveForwardOrBackward(double distance, bool forward) {
  float angle = forward ? 0.0f : 180.0f;
  std::string task_id =
      VMR_moveRelative(vmr_handle_, static_cast<float>(distance), angle);
  if (task_id.empty()) {
    std::cerr << "VMR_moveRelative failed" << std::endl;
    return false;
  }
  std::cout << (forward ? "forward" : "backward") << " task_id=" << task_id
            << std::endl;
  return WaitTaskDone(task_id);
}

bool LanxinControl::RotateInPlace(double deg) {
  std::string task_id = VMR_rotateInPlace(vmr_handle_, static_cast<float>(deg));
  if (task_id.empty()) {
    std::cerr << "VMR_rotateInPlace failed" << std::endl;
    return false;
  }
  std::cout << "rotate task_id=" << task_id << std::endl;
  return WaitTaskDone(task_id);
}

bool LanxinControl::RotateInPlaceRetry(double deg, int max_attempts, int retry_interval_sec) {
  if (max_attempts <= 0)
    max_attempts = 1;
  if (retry_interval_sec < 0)
    retry_interval_sec = 0;
  for (int attempt = 1; attempt <= max_attempts; ++attempt) {
    if (RotateInPlace(deg)) {
      if (attempt > 1)
        std::cout << "[chassis] RotateInPlace(" << deg << ") 第 " << attempt
                  << " 次成功" << std::endl;
      return true;
    }
    std::cerr << "[chassis] RotateInPlace(" << deg << ") 第 " << attempt << "/"
              << max_attempts << " 次失败";
    if (attempt < max_attempts) {
      std::cerr << "，" << retry_interval_sec << "s 后重试..." << std::endl;
      if (retry_interval_sec > 0)
        sleep(static_cast<unsigned>(retry_interval_sec));
    } else {
      std::cerr << std::endl;
    }
  }
  return false;
}

bool LanxinControl::MoveTopo(const std::string& pose_name, int32_t action) {
  if (!rpc_ready_) {
    std::cerr << "MoveTopo: call SetTopoEndpoint first (RPC not connected)"
              << std::endl;
    return false;
  }
  try {
    robot_task::TopoPose pose;
    pose.pose_name = pose_name;
    pose.action = action;
    auto cmd = robot_task::topo_move_task(pose);
    last_topo_task_id_ = cmd.id;
    has_last_topo_task_ = true;
    std::cout << "cmd task id: " << cmd.id << std::endl;

    auto status = cmd.wait_for(std::chrono::seconds(300));
    if (status == std::future_status::ready) {
      const auto result = cmd.get();
      std::cout << "err_code=" << result.err_code
                << " err_message=" << result.err_message
                << " data.result=" << result.data.result << std::endl;
      return result.err_code == 0 && result.data.result == 1;
    }
    std::cout << "cmd timeout" << std::endl;
    if (cmd.cancel) {
      cmd.cancel();
    }
    return false;
  } catch (const std::exception& e) {
    std::cerr << "MoveTopo(\"" << pose_name << "\") 异常: " << e.what()
              << std::endl;
    return false;
  }
}

bool LanxinControl::CancelTopoTask() {
  if (!rpc_ready_) {
    std::cerr << "CancelTopoTask: RPC not connected" << std::endl;
    return false;
  }
  if (!has_last_topo_task_) {
    std::cerr << "no cached topo task id, run MoveTopo first" << std::endl;
    return false;
  }
  robot_task::cancel_topo_move_task(last_topo_task_id_);
  std::cout << "cancel_topo_move_task(" << last_topo_task_id_ << ") sent"
            << std::endl;
  return true;
}

bool LanxinControl::EnsureRpcReady() {
  if (rpc_ready_) {
    return true;
  }
  return OpenRpcClient();
}

bool LanxinControl::StartChargeTask(double x, double y, double theta) {
  if (!EnsureRpcReady()) {
    return false;
  }
  try {
    robot_task::RobotPose pose{x, y, theta};
    auto cmd = robot_task::charge_task(pose);
    last_charge_task_id_ = cmd.id;
    has_last_charge_task_ = true;
    std::cout << "charge task id=" << cmd.id << std::endl;

    auto status = cmd.wait_for(std::chrono::seconds(120));
    if (status == std::future_status::ready) {
      const auto result = cmd.get();
      std::cout << "err_code=" << result.err_code
                << " err_message=" << result.err_message
                << " data.result=" << result.data.result << std::endl;
      return result.err_code == 0 && result.data.result == 1;
    }
    std::cout << "charge timeout, canceling..." << std::endl;
    robot_task::cancel_charge_task(cmd.id);
    return false;
  } catch (const std::exception& e) {
    std::cerr << "StartChargeTask 异常: " << e.what() << std::endl;
    return false;
  }
}

bool LanxinControl::CancelChargeTask() {
  if (!EnsureRpcReady()) {
    return false;
  }
  if (!has_last_charge_task_) {
    std::cerr << "no cached charge task id, run StartChargeTask first"
              << std::endl;
    return false;
  }
  robot_task::cancel_charge_task(last_charge_task_id_);
  std::cout << "cancel_charge_task(" << last_charge_task_id_ << ") sent"
            << std::endl;
  return true;
}

bool LanxinControl::ChargeRelay(bool on) {
  if (!EnsureRpcReady()) {
    return false;
  }
  const bool ok = robot_task::charge_relay(on);
  std::cout << "charge_relay(" << (on ? "true" : "false") << "): " << ok
            << std::endl;
  return ok;
}

bool LanxinControl::SetSpeedFactor(double factor) {
  int ret = VMR_setSpeedFactor(vmr_handle_, factor);
  std::cout << "VMR_setSpeedFactor(" << factor << ") ret=" << ret << std::endl;
  return ret == 0;
}

bool LanxinControl::EnableSdkCtrlSpeed(bool enable) {
  std::string task_id = VMR_enableSdkCtrlSpeed(vmr_handle_, enable);
  std::cout << "VMR_enableSdkCtrlSpeed(" << (enable ? "true" : "false")
            << ") -> \"" << task_id << "\"" << std::endl;
  return !task_id.empty();
}

bool LanxinControl::SendTwist(double vx, double vy, double wz) {
  VmrTwistInfo tw{};
  tw.linear.x = vx;
  tw.linear.y = vy;
  tw.angular.z = wz;
  VMR_setRobotTwist(vmr_handle_, tw);
  return true;
}

bool LanxinControl::StopTwist() { return SendTwist(0.0, 0.0, 0.0); }

void LanxinControl::BatteryCallbackBridge(const VmrBatteryInfo& info) {
  if (g_instance_) {
    g_instance_->OnBattery(info);
  }
}

void LanxinControl::OnBattery(const VmrBatteryInfo& info) {
  battery_ = info;
  has_battery_ = true;
}

bool LanxinControl::GetBattery(VmrBatteryInfo& out_battery) const {
  if (!has_battery_) {
    return false;
  }
  out_battery = battery_;
  return true;
}

void LanxinControl::PrintBattery() const {
  VmrBatteryInfo b;
  if (!GetBattery(b)) {
    std::cout << "no battery callback yet, wait 1-2s and retry" << std::endl;
    return;
  }
  std::cout << "battery: " << b.percentage << "%, V=" << b.voltage
            << ", I=" << b.current << ", status="
            << static_cast<int>(b.power_supply_status)
            << " (0 unknown,1 charging,2 discharging)" << std::endl;
}


int lanxin_point_move(LanxinControl &ctrl,double *point)
{

  ctrl.MoveToPose(point[0], point[1], point[2]);

  return 0;
}


void move_topo_until_ok(LanxinControl &ctrl, const std::string &pose_name,int a)
{
    for (int n = 1;; ++n)
    {
        if (ctrl.MoveTopo(pose_name, a))
            return;
        std::cerr << "[MoveTopo] 第 " << n << " 次失败 pose=" << pose_name << "，"
                  << 1 << " 秒后重试..." << std::endl;
        sleep(1);
    }
}

int chargeInMobileZone(LanxinControl &ctrl)
{
    if (!ctrl.MoveTopo("c01", 0))
        return -1;
    if (!ctrl.MoveTopo("c01", 0))
        return -1;
    if (!ctrl.MoveTopo("c1", 3))   // action=3 充电，这个对
        return -1;
    return 0;
}
int cancelCharging(LanxinControl &ctrl)
{
    if (!ctrl.MoveTopo("c1", 4))   // action=4 停充，这个对
        return -1;
    sleep(1);
    if (!ctrl.MoveTopo("c01", 0))
        return -1;
    return 0;
}

double getBatteryLevel(LanxinControl &ctrl)
{

    sleep(1);
    VmrBatteryInfo b;
    if (!ctrl.GetBattery(b)) {
        return -1;
    }
 

    return b.percentage;
}
