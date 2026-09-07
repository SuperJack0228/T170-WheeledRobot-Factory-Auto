#include "gripper_interface.hpp"

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <unistd.h>

namespace gripper {
namespace {

constexpr int kRightMotorId = 0;
constexpr int kLeftMotorId = 1;

double clamp(double v, double lo, double hi) {
  return std::clamp(v, std::min(lo, hi), std::max(lo, hi));
}

Side sideFromMotorId(int id) {
  return id == kRightMotorId ? Side::Right : Side::Left;
}

double angleToRad(double angle_deg, const Config &cfg) {
  const double ratio =
      clamp(angle_deg / std::max(1e-6, cfg.full_close_angle_deg), 0.0, cfg.max_close_ratio);
  return cfg.open_position_rad + ratio * (cfg.close_position_rad - cfg.open_position_rad);
}

double radToAngle(double q, const Config &cfg) {
  const double denom = cfg.close_position_rad - cfg.open_position_rad;
  if (std::fabs(denom) < 1e-6) {
    return 0.0;
  }
  const double ratio = clamp((q - cfg.open_position_rad) / denom, 0.0, 1.0);
  return ratio * cfg.full_close_angle_deg;
}

bool probeMotor(const std::shared_ptr<SerialPort> &serial, int motor_id) {
  MotorCmd cmd;
  MotorData data;
  cmd.motorType = MotorType::M4010;
  cmd.id = motor_id;
  cmd.mode = queryMotorMode(cmd.motorType, MotorMode::FOC);
  data.motorType = cmd.motorType;
  usleep(200);
  return serial->sendRecv(&cmd, &data);
}

}  // namespace

struct Gripper::SideImpl {
  Side side = Side::Left;
  int motor_id = 0;
  std::string port;
  std::shared_ptr<SerialPort> serial;
  float gear_ratio = 1.0f;

  double target_q = 0.0;
  double q_cmd = 0.0;
  double q = 0.0;
  double dq = 0.0;
  double tau = 0.0;
  bool initialized = false;

  double teleop_ratio = 0.0;
  double effective_target_q = 0.0;
  bool torque_limited = false;

  // ros2_170D 软力矩路径（Soft* / setTeleopSoftRatio）
  bool soft_torque_active = false;
  bool soft_torque_limited = false;
  double q_anchor = 0.0;
  double dq_filt = 0.0;
  bool cmd_initialized = false;
};

namespace {

bool readMotorPosition(const std::shared_ptr<SerialPort> &serial, int motor_id,
                       float gear_ratio, double *q_out) {
  if (!serial || !q_out) {
    return false;
  }
  MotorCmd cmd;
  MotorData data;
  cmd.motorType = MotorType::M4010;
  cmd.id = motor_id;
  cmd.mode = queryMotorMode(cmd.motorType, MotorMode::FOC);
  data.motorType = cmd.motorType;
  if (!serial->sendRecv(&cmd, &data)) {
    return false;
  }
  *q_out = data.q / gear_ratio;
  return true;
}

}  // namespace

std::vector<std::string> scanSerialPorts() {
  std::vector<std::string> ports;
  constexpr const char *kPrefixes[] = {"/dev/ttyUSB", "/dev/ttyCH343USB"};

  if (!std::filesystem::exists("/dev")) {
    return ports;
  }

  for (const auto &entry : std::filesystem::directory_iterator("/dev")) {
    const std::string path = entry.path().string();
    for (const char *prefix : kPrefixes) {
      if (path.rfind(prefix, 0) == 0) {
        ports.push_back(path);
        break;
      }
    }
  }
  std::sort(ports.begin(), ports.end());
  return ports;
}

Gripper::Gripper(const Config &config) : config_(config) {}

Gripper::~Gripper() { stop(); }

bool Gripper::isConnected() const { return connected_.load(); }

std::vector<DetectedMotor> Gripper::detectedMotors() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<DetectedMotor> out;
  for (const auto &[side, impl] : sides_) {
    DetectedMotor m;
    m.side = side;
    m.motor_id = impl->motor_id;
    m.port = impl->port;
    out.push_back(m);
  }
  return out;
}

bool Gripper::connect() {
  if (connected_.load()) {
    return true;
  }

  const auto ports = scanSerialPorts();
  if (ports.empty()) {
    std::cerr << "[gripper] 未找到串口 (/dev/ttyUSB* 或 /dev/ttyCH343USB*)\n";
    return false;
  }

  std::cout << "[gripper] 扫描串口: ";
  for (const auto &p : ports) {
    std::cout << p << " ";
  }
  std::cout << "\n";

  std::map<int, std::pair<std::shared_ptr<SerialPort>, std::string>> found;

  for (int attempt = 0; attempt < config_.detect_retries && found.size() < 2; ++attempt) {
    for (const auto &port : ports) {
      auto serial = std::make_shared<SerialPort>(port.c_str());
      for (int id : {kRightMotorId, kLeftMotorId}) {
        if (found.count(id) > 0) {
          continue;
        }
        if (probeMotor(serial, id)) {
          found[id] = {serial, port};
          std::cout << "[gripper] 检测到 motor_id=" << id << " side="
                    << (id == kRightMotorId ? "right" : "left") << " port=" << port << "\n";
        }
      }
    }
    if (found.size() < 2) {
      usleep(50000);
    }
  }

  if (found.empty()) {
    std::cerr << "[gripper] 未检测到夹爪电机，请检查 USB 与电源\n";
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  sides_.clear();
  for (const auto &[id, info] : found) {
    const Side side = sideFromMotorId(id);
    auto impl = std::make_unique<SideImpl>();
    impl->side = side;
    impl->motor_id = id;
    impl->port = info.second;
    impl->serial = info.first;
    impl->gear_ratio = queryGearRatio(MotorType::M4010);
    double current_q = config_.open_position_rad;
    if (readMotorPosition(impl->serial, impl->motor_id, impl->gear_ratio, &current_q)) {
      impl->q = current_q;
      impl->initialized = true;
      std::cout << "[gripper] motor_id=" << id << " 当前位置 q=" << current_q << " rad\n";
    }
    impl->target_q = current_q;
    impl->q_cmd = current_q;
    impl->q_anchor = current_q;
    impl->dq_filt = 0.0;
    sides_[side] = std::move(impl);
  }

  connected_ = true;
  return true;
}

bool Gripper::start() {
  if (running_.load()) {
    return true;
  }
  if (!connected_.load() && !connect()) {
    return false;
  }

  stop_requested_ = false;
  running_ = true;
  control_thread_ = std::thread(&Gripper::controlLoop, this);
  return true;
}

void Gripper::stop() {
  if (running_.load()) {
    stop_requested_ = true;
    if (control_thread_.joinable()) {
      control_thread_.join();
    }
    running_ = false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  sides_.clear();
  connected_ = false;
}

void Gripper::setTargetRad(Side side, double target_rad) {
  std::lock_guard<std::mutex> lock(mutex_);
  teleop_mode_ = false;
  teleop_soft_mode_ = false;
  const auto it = sides_.find(side);
  if (it == sides_.end()) {
    return;
  }
  const double q =
      clamp(target_rad, config_.open_position_rad, config_.close_position_rad);
  it->second->soft_torque_active = false;
  it->second->soft_torque_limited = false;
  it->second->torque_limited = false;
  it->second->target_q = q;
  it->second->q_cmd = q;
  if (it->second->initialized) {
    it->second->q_anchor = it->second->q;
    it->second->dq_filt = it->second->dq;
  }
}

void Gripper::setSoftTargetRad(Side side, double target_rad) {
  std::lock_guard<std::mutex> lock(mutex_);
  teleop_mode_ = false;
  teleop_soft_mode_ = false;
  const auto it = sides_.find(side);
  if (it == sides_.end()) {
    return;
  }
  const double q =
      clamp(target_rad, config_.open_position_rad, config_.close_position_rad);
  it->second->soft_torque_active = true;
  it->second->soft_torque_limited = false;
  it->second->target_q = q;
  syncSoftMotionState(*it->second);
}

void Gripper::setAngle(Side side, double angle_deg) {
  setTargetRad(side, angleToRad(clamp(angle_deg, 0.0, config_.full_close_angle_deg), config_));
}

void Gripper::setBothAngles(double left_deg, double right_deg) {
  setAngle(Side::Left, left_deg);
  setAngle(Side::Right, right_deg);
}

void Gripper::open(Side side) { setTargetRad(side, config_.open_position_rad); }
void Gripper::openBoth() {
  setTargetRad(Side::Left, config_.open_position_rad);
  setTargetRad(Side::Right, config_.open_position_rad);
}

void Gripper::close(Side side, double ratio) {
  const double r = clamp(ratio, 0.0, config_.max_close_ratio);
  const double q =
      config_.open_position_rad + r * (config_.close_position_rad - config_.open_position_rad);
  setTargetRad(side, q);
}

void Gripper::closeBoth(double ratio) {
  const double r = clamp(ratio, 0.0, config_.max_close_ratio);
  const double q =
      config_.open_position_rad + r * (config_.close_position_rad - config_.open_position_rad);
  setTargetRad(Side::Left, q);
  setTargetRad(Side::Right, q);
}

bool Gripper::waitBothRad(double target_rad, double tolerance_rad,
                          std::chrono::milliseconds timeout) {
  bool wait_left = false;
  bool wait_right = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    wait_left = sides_.count(Side::Left) > 0;
    wait_right = sides_.count(Side::Right) > 0;
  }
  bool ok = true;
  if (wait_left) {
    ok &= waitForRad(Side::Left, target_rad, tolerance_rad, timeout);
  }
  if (wait_right) {
    ok &= waitForRad(Side::Right, target_rad, tolerance_rad, timeout);
  }
  return ok;
}

bool Gripper::openBothAndWait(double tolerance_rad, std::chrono::milliseconds timeout) {
  openBoth();
  return waitBothRad(config_.open_position_rad, tolerance_rad, timeout);
}

bool Gripper::closeBothAndWait(double ratio, double tolerance_rad,
                               std::chrono::milliseconds timeout) {
  closeBoth(ratio);
  const double r = clamp(ratio, 0.0, config_.max_close_ratio);
  const double q =
      config_.open_position_rad + r * (config_.close_position_rad - config_.open_position_rad);
  return waitBothRad(q, tolerance_rad, timeout);
}

bool Gripper::openAndWait(Side side, double tolerance_rad, std::chrono::milliseconds timeout) {
  open(side);
  return waitForRad(side, config_.open_position_rad, tolerance_rad, timeout);
}

bool Gripper::closeAndWait(Side side, double ratio, double tolerance_rad,
                           std::chrono::milliseconds timeout) {
  close(side, ratio);
  const double r = clamp(ratio, 0.0, config_.max_close_ratio);
  const double q =
      config_.open_position_rad + r * (config_.close_position_rad - config_.open_position_rad);
  return waitForRad(side, q, tolerance_rad, timeout);
}

void Gripper::openSoft(Side side) {
  setSoftTargetRad(side, config_.open_position_rad);
}

void Gripper::openSoftBoth() {
  setSoftTargetRad(Side::Left, config_.open_position_rad);
  setSoftTargetRad(Side::Right, config_.open_position_rad);
}

void Gripper::closeSoft(Side side, double ratio) {
  const double r = clamp(ratio, 0.0, config_.max_close_ratio);
  const double q =
      config_.open_position_rad + r * (config_.close_position_rad - config_.open_position_rad);
  setSoftTargetRad(side, q);
}

void Gripper::closeSoftBoth(double ratio) {
  const double r = clamp(ratio, 0.0, config_.max_close_ratio);
  const double q =
      config_.open_position_rad + r * (config_.close_position_rad - config_.open_position_rad);
  setSoftTargetRad(Side::Left, q);
  setSoftTargetRad(Side::Right, q);
}

bool Gripper::openSoftAndWait(Side side, double tolerance_rad,
                              std::chrono::milliseconds timeout) {
  openSoft(side);
  return waitForRad(side, config_.open_position_rad, tolerance_rad, timeout);
}

bool Gripper::openSoftBothAndWait(double tolerance_rad, std::chrono::milliseconds timeout) {
  openSoftBoth();
  return waitBothRad(config_.open_position_rad, tolerance_rad, timeout);
}

bool Gripper::closeSoftAndWait(Side side, double ratio, double tolerance_rad,
                               std::chrono::milliseconds timeout) {
  closeSoft(side, ratio);
  const double r = clamp(ratio, 0.0, config_.max_close_ratio);
  const double q =
      config_.open_position_rad + r * (config_.close_position_rad - config_.open_position_rad);
  return waitForRad(side, q, tolerance_rad, timeout);
}

bool Gripper::closeSoftBothAndWait(double ratio, double tolerance_rad,
                                   std::chrono::milliseconds timeout) {
  closeSoftBoth(ratio);
  const double r = clamp(ratio, 0.0, config_.max_close_ratio);
  const double q =
      config_.open_position_rad + r * (config_.close_position_rad - config_.open_position_rad);
  return waitBothRad(q, tolerance_rad, timeout);
}

double Gripper::resolveGraspTorque(double max_torque_nm) const {
  return max_torque_nm > 0.0 ? max_torque_nm : config_.grasp_torque_limit_nm;
}

double Gripper::resolveSoftTorque(double max_torque_nm) const {
  return max_torque_nm > 0.0 ? max_torque_nm : config_.soft_torque_limit_nm;
}

GraspFeedback Gripper::graspSideUntil(Side side, double max_torque_nm,
                                      double position_tolerance_rad,
                                      std::chrono::milliseconds timeout) {
  GraspFeedback out;
  const double torque_limit = resolveGraspTorque(max_torque_nm);
  const double close_q = config_.close_position_rad;
  const double pos_tol = std::max(0.0, position_tolerance_rad);

  close(side, 1.0);

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto fb = feedback(side);
    if (!fb.have_feedback) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    out.position_rad = fb.position_rad;
    out.torque = fb.torque;

    if (torque_limit > 0.0 && std::fabs(fb.torque) >= torque_limit) {
      setTargetRad(side, fb.position_rad);
      out.result = GraspResult::TorqueLimit;
      return out;
    }
    if (std::fabs(fb.position_rad - close_q) <= pos_tol) {
      out.result = GraspResult::PositionReached;
      return out;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  const auto fb = feedback(side);
  out.position_rad = fb.position_rad;
  out.torque = fb.torque;
  out.result = GraspResult::Timeout;
  return out;
}

GraspFeedback Gripper::graspAndWait(Side side, double max_torque_nm,
                                    std::chrono::milliseconds timeout) {
  return graspSideUntil(side, max_torque_nm, 0.2, timeout);
}

GraspFeedback Gripper::graspSoftSideUntil(Side side, double max_torque_nm,
                                          double position_tolerance_rad,
                                          std::chrono::milliseconds timeout) {
  GraspFeedback out;
  const double tau_limit = resolveSoftTorque(max_torque_nm);
  const double close_q = config_.close_position_rad;
  const double pos_tol = std::max(0.0, position_tolerance_rad);
  const double tau_contact = tau_limit > 0.0 ? tau_limit * 0.75 : 0.0;
  constexpr double kVelTol = 0.35;
  constexpr int kContactSamples = 15;

  closeSoft(side, 1.0);

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  int contact_samples = 0;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto fb = feedback(side);
    if (!fb.have_feedback) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    out.position_rad = fb.position_rad;
    out.torque = fb.torque;

    if (std::fabs(fb.position_rad - close_q) <= pos_tol) {
      out.result = GraspResult::PositionReached;
      return out;
    }

    if (tau_contact > 0.0 && fb.soft_torque_limited &&
        std::fabs(fb.torque) >= tau_contact && std::fabs(fb.velocity_rad_s) < kVelTol &&
        fb.position_rad > close_q + pos_tol) {
      ++contact_samples;
      if (contact_samples >= kContactSamples) {
        setSoftTargetRad(side, fb.position_rad);
        out.result = GraspResult::TorqueLimit;
        return out;
      }
    } else {
      contact_samples = 0;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  const auto fb = feedback(side);
  out.position_rad = fb.position_rad;
  out.torque = fb.torque;
  out.result = GraspResult::Timeout;
  return out;
}

GraspFeedback Gripper::graspSoftAndWait(Side side, double max_torque_nm,
                                        std::chrono::milliseconds timeout) {
  return graspSoftSideUntil(side, max_torque_nm, 0.2, timeout);
}

GraspFeedback Gripper::graspSoftBothAndWait(double max_torque_nm,
                                            std::chrono::milliseconds timeout) {
  GraspFeedback out;
  closeSoftBoth(1.0);

  const double tau_limit = resolveSoftTorque(max_torque_nm);
  const double close_q = config_.close_position_rad;
  constexpr double kPosTol = 0.2;
  const double tau_contact = tau_limit > 0.0 ? tau_limit * 0.75 : 0.0;
  constexpr double kVelTol = 0.35;
  constexpr int kContactSamples = 15;

  bool left_done = !hasSide(Side::Left);
  bool right_done = !hasSide(Side::Right);
  GraspFeedback left_fb;
  GraspFeedback right_fb;
  int left_contact = 0;
  int right_contact = 0;

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (!left_done && hasSide(Side::Left)) {
      const auto fb = feedback(Side::Left);
      if (fb.have_feedback) {
        left_fb.position_rad = fb.position_rad;
        left_fb.torque = fb.torque;
        if (std::fabs(fb.position_rad - close_q) <= kPosTol) {
          left_fb.result = GraspResult::PositionReached;
          left_done = true;
        } else if (tau_contact > 0.0 && fb.soft_torque_limited &&
                   std::fabs(fb.torque) >= tau_contact &&
                   std::fabs(fb.velocity_rad_s) < kVelTol &&
                   fb.position_rad > close_q + kPosTol) {
          ++left_contact;
          if (left_contact >= kContactSamples) {
            setSoftTargetRad(Side::Left, fb.position_rad);
            left_fb.result = GraspResult::TorqueLimit;
            left_done = true;
          }
        } else {
          left_contact = 0;
        }
      }
    }
    if (!right_done && hasSide(Side::Right)) {
      const auto fb = feedback(Side::Right);
      if (fb.have_feedback) {
        right_fb.position_rad = fb.position_rad;
        right_fb.torque = fb.torque;
        if (std::fabs(fb.position_rad - close_q) <= kPosTol) {
          right_fb.result = GraspResult::PositionReached;
          right_done = true;
        } else if (tau_contact > 0.0 && fb.soft_torque_limited &&
                   std::fabs(fb.torque) >= tau_contact &&
                   std::fabs(fb.velocity_rad_s) < kVelTol &&
                   fb.position_rad > close_q + kPosTol) {
          ++right_contact;
          if (right_contact >= kContactSamples) {
            setSoftTargetRad(Side::Right, fb.position_rad);
            right_fb.result = GraspResult::TorqueLimit;
            right_done = true;
          }
        } else {
          right_contact = 0;
        }
      }
    }
    if (left_done && right_done) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  if (!left_done) {
    left_fb.result = GraspResult::Timeout;
    const auto fb = feedback(Side::Left);
    left_fb.position_rad = fb.position_rad;
    left_fb.torque = fb.torque;
  }
  if (!right_done) {
    right_fb.result = GraspResult::Timeout;
    const auto fb = feedback(Side::Right);
    right_fb.position_rad = fb.position_rad;
    right_fb.torque = fb.torque;
  }

  if (left_fb.result == GraspResult::TorqueLimit ||
      right_fb.result == GraspResult::TorqueLimit) {
    out.result = GraspResult::TorqueLimit;
  } else if (left_fb.result == GraspResult::PositionReached &&
             right_fb.result == GraspResult::PositionReached) {
    out.result = GraspResult::PositionReached;
  } else {
    out.result = GraspResult::Timeout;
  }
  out.position_rad = (left_fb.position_rad + right_fb.position_rad) * 0.5;
  out.torque = std::max(std::fabs(left_fb.torque), std::fabs(right_fb.torque));
  return out;
}

bool Gripper::isSoftTorqueActive(Side side) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = sides_.find(side);
  if (it == sides_.end()) {
    return false;
  }
  return it->second->soft_torque_active;
}

GraspFeedback Gripper::graspBothAndWait(double max_torque_nm,
                                        std::chrono::milliseconds timeout) {
  GraspFeedback out;
  closeBoth(1.0);

  const double torque_limit = resolveGraspTorque(max_torque_nm);
  const double close_q = config_.close_position_rad;
  constexpr double kPosTol = 0.2;

  bool left_done = !hasSide(Side::Left);
  bool right_done = !hasSide(Side::Right);
  GraspFeedback left_fb;
  GraspFeedback right_fb;

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (!left_done && hasSide(Side::Left)) {
      const auto fb = feedback(Side::Left);
      if (fb.have_feedback) {
        left_fb.position_rad = fb.position_rad;
        left_fb.torque = fb.torque;
        if (torque_limit > 0.0 && std::fabs(fb.torque) >= torque_limit) {
          setTargetRad(Side::Left, fb.position_rad);
          left_fb.result = GraspResult::TorqueLimit;
          left_done = true;
        } else if (std::fabs(fb.position_rad - close_q) <= kPosTol) {
          left_fb.result = GraspResult::PositionReached;
          left_done = true;
        }
      }
    }
    if (!right_done && hasSide(Side::Right)) {
      const auto fb = feedback(Side::Right);
      if (fb.have_feedback) {
        right_fb.position_rad = fb.position_rad;
        right_fb.torque = fb.torque;
        if (torque_limit > 0.0 && std::fabs(fb.torque) >= torque_limit) {
          setTargetRad(Side::Right, fb.position_rad);
          right_fb.result = GraspResult::TorqueLimit;
          right_done = true;
        } else if (std::fabs(fb.position_rad - close_q) <= kPosTol) {
          right_fb.result = GraspResult::PositionReached;
          right_done = true;
        }
      }
    }
    if (left_done && right_done) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  if (!left_done) {
    left_fb.result = GraspResult::Timeout;
    const auto fb = feedback(Side::Left);
    left_fb.position_rad = fb.position_rad;
    left_fb.torque = fb.torque;
  }
  if (!right_done) {
    right_fb.result = GraspResult::Timeout;
    const auto fb = feedback(Side::Right);
    right_fb.position_rad = fb.position_rad;
    right_fb.torque = fb.torque;
  }

  if (left_fb.result == GraspResult::TorqueLimit ||
      right_fb.result == GraspResult::TorqueLimit) {
    out.result = GraspResult::TorqueLimit;
  } else if (left_fb.result == GraspResult::PositionReached &&
             right_fb.result == GraspResult::PositionReached) {
    out.result = GraspResult::PositionReached;
  } else {
    out.result = GraspResult::Timeout;
  }
  out.position_rad = (left_fb.position_rad + right_fb.position_rad) * 0.5;
  out.torque = std::max(std::fabs(left_fb.torque), std::fabs(right_fb.torque));
  return out;
}

bool Gripper::hasSide(Side side) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return sides_.count(side) > 0;
}

SideFeedback Gripper::feedback(Side side) const {
  std::lock_guard<std::mutex> lock(mutex_);
  SideFeedback fb;
  const auto it = sides_.find(side);
  if (it == sides_.end()) {
    return fb;
  }
  const auto &s = *it->second;
  fb.position_rad = s.q;
  fb.velocity_rad_s = s.dq;
  fb.torque = s.tau;
  fb.angle_deg = radToAngle(s.q, config_);
  fb.have_feedback = s.initialized;
  fb.command_rad = s.q_cmd;
  fb.soft_torque_limited = s.soft_torque_limited;
  return fb;
}

bool Gripper::waitFor(Side side, double target_deg, double tolerance_deg,
                      std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  const double tol = std::max(0.0, tolerance_deg);
  while (std::chrono::steady_clock::now() < deadline) {
    const auto fb = feedback(side);
    if (fb.have_feedback && std::fabs(fb.angle_deg - target_deg) <= tol) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

bool Gripper::waitForRad(Side side, double target_rad, double tolerance_rad,
                         std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  const double tol = std::max(0.0, tolerance_rad);
  while (std::chrono::steady_clock::now() < deadline) {
    const auto fb = feedback(side);
    if (fb.have_feedback && std::fabs(fb.position_rad - target_rad) <= tol) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

double Gripper::ratioToRad(double ratio) const {
  const double r = clamp(ratio, 0.0, config_.max_close_ratio);
  return config_.open_position_rad + r * (config_.close_position_rad - config_.open_position_rad);
}

double Gripper::radToRatio(double q) const {
  const double denom = config_.close_position_rad - config_.open_position_rad;
  if (std::fabs(denom) < 1e-6) {
    return 0.0;
  }
  return clamp((q - config_.open_position_rad) / denom, 0.0, 1.0);
}

void Gripper::syncSoftMotionState(SideImpl &impl) {
  if (!impl.initialized) {
    return;
  }
  impl.q_anchor = impl.q;
  impl.dq_filt = impl.dq;
  impl.q_cmd = impl.q;
  impl.cmd_initialized = true;
}

void Gripper::updateMotionFilter(SideImpl &impl) {
  if (!impl.initialized) {
    return;
  }
  const double pf = clamp(config_.pos_filter, 1e-3, 1.0);
  impl.q_anchor += pf * (impl.q - impl.q_anchor);
  impl.dq_filt += pf * (impl.dq - impl.dq_filt);
}

void Gripper::applySoftTorqueLimit(SideImpl &impl) {
  const double tau_lim = config_.soft_torque_limit_nm;
  if (tau_lim <= 0.0 || config_.kp <= 1e-6 || !impl.initialized) {
    impl.soft_torque_limited = false;
    return;
  }

  updateMotionFilter(impl);

  // 大行程：只按斜率走，不钳位力矩；同步 anchor，避免夹取后张开时被旧 anchor 锁死
  const double dist_to_target = std::fabs(impl.target_q - impl.q);
  if (dist_to_target > config_.soft_coast_margin_rad) {
    impl.soft_torque_limited = false;
    impl.q_anchor = impl.q;
    return;
  }

  // 接触段用当前实测位置做参考，不用滞后 filter 的 q_anchor
  const double ref_q = impl.q;

  // ros2_170D dex1_gripper_driver_node: tau ≈ kp*(q_cmd-q) - kd*dq
  const double lo = ref_q + (-tau_lim + config_.kd * impl.dq_filt) / config_.kp;
  const double hi = ref_q + (+tau_lim + config_.kd * impl.dq_filt) / config_.kp;

  const double before = impl.q_cmd;
  impl.q_cmd = clamp(impl.q_cmd, std::min(lo, hi), std::max(lo, hi));
  impl.soft_torque_limited = std::fabs(impl.q_cmd - before) > 1e-4;
}

void Gripper::runSoftControlStep(SideImpl &impl, double dt) {
  if (!impl.cmd_initialized) {
    impl.q_cmd = impl.initialized ? impl.q : impl.target_q;
    impl.q_anchor = impl.q_cmd;
    impl.dq_filt = impl.initialized ? impl.dq : 0.0;
    impl.cmd_initialized = true;
  }

  const double dist_to_target = std::fabs(impl.target_q - impl.q);
  const double coast_margin = std::max(0.0, config_.soft_coast_margin_rad);

  if (coast_margin <= 0.0 || dist_to_target > coast_margin) {
    // 大行程：指令直接跟目标，速度与经典一致
    impl.q_cmd = impl.target_q;
  } else {
    const double max_step = std::max(0.0, config_.soft_slew_rate) * dt;
    if (max_step > 0.0) {
      const double err = impl.target_q - impl.q_cmd;
      impl.q_cmd += clamp(err, -max_step, max_step);
    } else {
      impl.q_cmd = impl.target_q;
    }
  }

  applySoftTorqueLimit(impl);
  impl.q_cmd = clamp(impl.q_cmd, config_.close_position_rad, config_.open_position_rad);
}

void Gripper::applyTeleopTarget(SideImpl &impl) {
  const double ratio = clamp(impl.teleop_ratio, 0.0, 1.0);
  double q_des = ratioToRad(ratio);

  const double tau_lim = config_.grasp_torque_limit_nm;
  if (tau_lim > 0.0 && q_des < impl.q - 1e-4 && std::fabs(impl.tau) >= tau_lim) {
    q_des = impl.q;
    impl.torque_limited = true;
  } else {
    impl.torque_limited = false;
  }

  impl.effective_target_q = q_des;
  impl.target_q = q_des;
  impl.q_cmd = q_des;
}

void Gripper::setTeleopMode(bool enabled) {
  teleop_mode_ = enabled;
  if (!enabled) {
    teleop_soft_mode_ = false;
  }
}

void Gripper::setTeleopRatio(Side side, double ratio) {
  std::lock_guard<std::mutex> lock(mutex_);
  teleop_mode_ = true;
  teleop_soft_mode_ = false;
  const auto it = sides_.find(side);
  if (it == sides_.end()) {
    return;
  }
  it->second->teleop_ratio = clamp(ratio, 0.0, 1.0);
  it->second->soft_torque_active = false;
}

void Gripper::setTeleopBoth(double left_ratio, double right_ratio) {
  std::lock_guard<std::mutex> lock(mutex_);
  teleop_mode_ = true;
  teleop_soft_mode_ = false;
  if (const auto it = sides_.find(Side::Left); it != sides_.end()) {
    it->second->teleop_ratio = clamp(left_ratio, 0.0, 1.0);
    it->second->soft_torque_active = false;
  }
  if (const auto it = sides_.find(Side::Right); it != sides_.end()) {
    it->second->teleop_ratio = clamp(right_ratio, 0.0, 1.0);
    it->second->soft_torque_active = false;
  }
}

TeleopFeedback Gripper::teleopFeedback(Side side) const {
  std::lock_guard<std::mutex> lock(mutex_);
  TeleopFeedback out;
  const auto it = sides_.find(side);
  if (it == sides_.end()) {
    return out;
  }
  const auto &s = *it->second;
  out.command_ratio = s.teleop_ratio;
  out.effective_ratio = radToRatio(s.effective_target_q);
  out.position_rad = s.q;
  out.torque = s.tau;
  out.torque_limited = s.torque_limited;
  return out;
}

void Gripper::setTeleopSoftMode(bool enabled) {
  teleop_soft_mode_ = enabled;
  if (enabled) {
    teleop_mode_ = true;
  }
}

void Gripper::setTeleopSoftRatio(Side side, double ratio) {
  std::lock_guard<std::mutex> lock(mutex_);
  teleop_mode_ = true;
  teleop_soft_mode_ = true;
  const auto it = sides_.find(side);
  if (it == sides_.end()) {
    return;
  }
  it->second->teleop_ratio = clamp(ratio, 0.0, 1.0);
  it->second->soft_torque_active = true;
  syncSoftMotionState(*it->second);
}

void Gripper::setTeleopSoftBoth(double left_ratio, double right_ratio) {
  std::lock_guard<std::mutex> lock(mutex_);
  teleop_mode_ = true;
  teleop_soft_mode_ = true;
  if (const auto it = sides_.find(Side::Left); it != sides_.end()) {
    it->second->teleop_ratio = clamp(left_ratio, 0.0, 1.0);
    it->second->soft_torque_active = true;
  }
  if (const auto it = sides_.find(Side::Right); it != sides_.end()) {
    it->second->teleop_ratio = clamp(right_ratio, 0.0, 1.0);
    it->second->soft_torque_active = true;
  }
}

TeleopFeedback Gripper::teleopSoftFeedback(Side side) const {
  std::lock_guard<std::mutex> lock(mutex_);
  TeleopFeedback out;
  const auto it = sides_.find(side);
  if (it == sides_.end()) {
    return out;
  }
  const auto &s = *it->second;
  out.command_ratio = s.teleop_ratio;
  out.effective_ratio = radToRatio(s.q_cmd);
  out.position_rad = s.q;
  out.torque = s.tau;
  out.torque_limited = s.soft_torque_limited;
  return out;
}

bool Gripper::calibrate(Side side) {
  if (!connected_.load() && !connect()) {
    return false;
  }

  std::shared_ptr<SerialPort> serial;
  int motor_id = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = sides_.find(side);
    if (it == sides_.end()) {
      std::cerr << "[gripper] 未找到 side=" << (side == Side::Left ? "left" : "right") << "\n";
      return false;
    }
    serial = it->second->serial;
    motor_id = it->second->motor_id;
  }

  const bool ok = serial->calibration(MotorType::M4010, motor_id, 0.0f, config_.calibration_limit_rad);
  if (ok) {
    std::cout << "[gripper] motor_id=" << motor_id << " 标定成功\n";
  } else {
    std::cerr << "[gripper] motor_id=" << motor_id << " 标定失败\n";
  }
  return ok;
}

bool Gripper::calibrateInteractive() {
  if (!connect()) {
    return false;
  }

  const auto motors = detectedMotors();
  if (motors.empty()) {
    return false;
  }

  std::cout << "\n=== DEX1 夹爪标定 ===\n";
  std::cout << "步骤：手动将夹爪紧闭到极限，然后输入 s 并回车。\n";
  std::cout << "参考: https://support.unitree.com/home/zh/dex1-1_gripper/dex1_1\n\n";

  int index = 1;
  for (const auto &m : motors) {
    const char *side_name = m.side == Side::Right ? "右" : "左";
    std::cout << "--- 标定 " << index << "/" << motors.size() << " ---\n";
    std::cout << "Motor ID: " << m.motor_id << "  " << side_name << "夹爪  串口: " << m.port << "\n";
    std::cout << "请手动紧闭夹爪，完成后输入 s 回车（其他键跳过）: ";
    char key = 0;
    std::cin >> key;
    if (key == 's' || key == 'S') {
      if (!calibrate(m.side)) {
        return false;
      }
    } else {
      std::cout << "已跳过 motor_id=" << m.motor_id << "\n";
    }
    ++index;
  }

  std::cout << "\n标定流程结束。\n";
  return true;
}

void Gripper::controlLoop() {
  using clock = std::chrono::steady_clock;
  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, config_.control_hz));
  auto next_tick = clock::now();

  const double kp_scale = config_.kp;
  const double kd_scale = config_.kd;

  while (!stop_requested_.load()) {
    const auto tick_start = clock::now();
    const double dt = std::max(0.0, std::chrono::duration<double>(tick_start - next_tick).count());

    std::lock_guard<std::mutex> lock(mutex_);
    const bool teleop = teleop_mode_.load();
    const bool teleop_soft = teleop_soft_mode_.load();

    for (auto &[side, impl] : sides_) {
      (void)side;
      if (!impl->serial) {
        continue;
      }

      if (teleop && teleop_soft) {
        impl->soft_torque_active = true;
        impl->target_q = ratioToRad(clamp(impl->teleop_ratio, 0.0, 1.0));
        runSoftControlStep(*impl, dt);
        impl->effective_target_q = impl->q_cmd;
        impl->torque_limited = impl->soft_torque_limited;
      } else if (teleop) {
        applyTeleopTarget(*impl);
      } else if (impl->soft_torque_active) {
        runSoftControlStep(*impl, dt);
      } else {
        const double slew_rate = config_.default_slew_rate;
        const double max_step = std::max(0.0, slew_rate) * dt;
        if (max_step > 0.0) {
          const double err = impl->target_q - impl->q_cmd;
          impl->q_cmd += clamp(err, -max_step, max_step);
        } else {
          impl->q_cmd = impl->target_q;
        }
        impl->soft_torque_limited = false;
        updateMotionFilter(*impl);
        impl->q_cmd =
            clamp(impl->q_cmd, config_.close_position_rad, config_.open_position_rad);
      }

      MotorCmd cmd;
      MotorData data;
      cmd.motorType = MotorType::M4010;
      cmd.id = impl->motor_id;
      cmd.mode = queryMotorMode(cmd.motorType, MotorMode::FOC);
      data.motorType = cmd.motorType;

      const float gr = impl->gear_ratio;
      const float gr2 = gr * gr;
      cmd.kp = static_cast<float>(kp_scale / gr2);
      cmd.kd = static_cast<float>(kd_scale / gr2);
      cmd.q = static_cast<float>(impl->q_cmd * gr);
      cmd.dq = 0.0f;
      cmd.tau = 0.0f;
      cmd.timeout = 0;

      if (impl->serial->sendRecv(&cmd, &data)) {
        impl->q = data.q / gr;
        impl->dq = data.dq / gr;
        impl->tau = data.tau * gr;
        impl->initialized = true;
      }
    }

    next_tick += std::chrono::duration_cast<clock::duration>(period);
    std::this_thread::sleep_until(next_tick);
  }
}

}  // namespace gripper
