#include "gripper_interface.hpp"

#include "Ti5_Device_SDK.hpp"
#include "Ti5_socketcan.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iostream>
#include <thread>
#include <unistd.h>
#include <vector>

namespace gripper {
namespace {

constexpr int kRightMotorId = 0;
constexpr int kLeftMotorId = 1;
constexpr int kDex1RecvMs = 20;
constexpr std::uint32_t kMerrorTimeout = 512u;

double clamp(double v, double lo, double hi) {
  return std::clamp(v, std::min(lo, hi), std::max(lo, hi));
}

Side sideFromMotorId(int id) {
  return id == kRightMotorId ? Side::Right : Side::Left;
}

const char *sideName(Side side) { return side == Side::Right ? "右" : "左"; }

void log_merror(const char *tag, int motor_id, std::uint32_t merror, bool io_ok) {
  std::cout << "[gripper] " << tag << " id=" << motor_id << " io=" << (io_ok ? "ok" : "fail")
            << " merror=" << merror;
  if (merror & kMerrorTimeout)
    std::cout << "(超时保护,需FOC且timeout位=0清除)";
  else if (merror != 0)
    std::cout << "(故障)";
  std::cout << "\n";
}

// SDK 的 get_Gripper_State 实际是 Brake：会停机，空闲后 MError=512。
// 探活/读位置改走 FOC + kp=0，只问状态不拉位置。
bool dex1_read(uint8_t id, float q_hint, float &q, float &dq, float &tau, std::uint32_t &merror) {
  int temp_shell = 0;
  int temp_winding = 0;
  float voltage = 0.f;
  merror = 0;
  return set_Gripper_Pos_get_State(id, q_hint, 0.f, 0.f, 0.f, 0.f, q, dq, tau, temp_shell,
                                   temp_winding, voltage, merror, kDex1RecvMs);
}

bool dex1_set_get(uint8_t id, float q, float dq, float tau, float kp, float kd, float &q_fb,
                  float &dq_fb, float &tau_fb, std::uint32_t &merror) {
  int temp_shell = 0;
  int temp_winding = 0;
  float voltage = 0.f;
  merror = 0;
  return set_Gripper_Pos_get_State(id, q, dq, tau, kp, kd, q_fb, dq_fb, tau_fb, temp_shell,
                                   temp_winding, voltage, merror, kDex1RecvMs);
}

bool moved_toward(double q0, double q, double target, double min_delta) {
  if (std::fabs(q - q0) < min_delta)
    return false;
  const double want = target - q0;
  if (std::fabs(want) < min_delta)
    return true;
  return (q - q0) * want > 0.0;
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

}  // namespace

struct Gripper::SideImpl {
  Side side = Side::Left;
  int motor_id = 0;
  std::string port = "ti5-dex1";
  bool present = false;

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

  std::uint32_t merror = 0;
  bool io_ok = false;
  int io_fail_streak = 0;
  std::uint32_t last_logged_merror = 0;
};

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

  if (right_arm_motors_locked())
    std::cout << "[gripper] 右臂锁定，只用左夹爪 id=1（Ti5_Dex1）\n";

  std::vector<int> required;
  if (!right_arm_motors_locked())
    required.push_back(kRightMotorId);
  required.push_back(kLeftMotorId);

  std::vector<int> found;
  for (int attempt = 0; attempt < config_.detect_retries; ++attempt) {
    found.clear();
    for (int id : required) {
      if (!gripper_paired(static_cast<std::uint8_t>(id)))
        continue;
      float q = 0.f, dq = 0.f, tau = 0.f;
      std::uint32_t merror = 0;
      if (!dex1_read(static_cast<std::uint8_t>(id), 0.f, q, dq, tau, merror))
        continue;
      found.push_back(id);
      log_merror("探活", id, merror, true);
    }
    if (found.size() == required.size())
      break;
    std::cerr << "[gripper] 第 " << (attempt + 1) << "/" << config_.detect_retries
              << " 次未齐，缺";
    for (int id : required) {
      if (std::find(found.begin(), found.end(), id) == found.end())
        std::cerr << ' ' << id << (id == kRightMotorId ? "(右)" : "(左)");
    }
    std::cerr << "\n";
    usleep(200000);
  }

    if (found.empty()) {
      std::cerr << "[gripper] 一个夹爪都没有，拒绝启动\n";
      return false;
    }
    if (found.size() != required.size()) {
      std::cerr << "[gripper] 少一个夹爪，仍启动，手臂照常运动。缺";
      for (int id : required) {
        if (std::find(found.begin(), found.end(), id) == found.end())
          std::cerr << ' ' << id << (id == kRightMotorId ? "(右)" : "(左)");
      }
      std::cerr << "\n";
    }

  std::lock_guard<std::mutex> lock(mutex_);
  sides_.clear();
  for (int id : found) {
    const Side side = sideFromMotorId(id);
    auto impl = std::make_unique<SideImpl>();
    impl->side = side;
    impl->motor_id = id;
    impl->port = "ti5-dex1";
    impl->present = true;
    float q = 0.f, dq = 0.f, tau = 0.f;
    std::uint32_t merror = 0;
    double current_q = config_.open_position_rad;
    if (dex1_read(static_cast<std::uint8_t>(id), 0.f, q, dq, tau, merror)) {
      current_q = q;
      impl->q = q;
      impl->dq = dq;
      impl->tau = tau;
      impl->merror = merror;
      impl->io_ok = true;
      impl->initialized = true;
      std::cout << "[gripper] motor_id=" << id << " 当前位置 q=" << current_q << " rad\n";
      log_merror("读位置", id, merror, true);
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
  if (!wait_left && !wait_right)
    return true;

  const auto t0 = std::chrono::steady_clock::now();
  const auto deadline = t0 + timeout;
  const auto stuck_after = t0 + std::chrono::milliseconds(2500);
  const double tol = std::max(0.0, tolerance_rad);
  const double left0 = wait_left ? feedback(Side::Left).position_rad : 0.0;
  const double right0 = wait_right ? feedback(Side::Right).position_rad : 0.0;
  bool left_ok = !wait_left;
  bool right_ok = !wait_right;
  bool logged_stuck = false;

  while (std::chrono::steady_clock::now() < deadline) {
    if (wait_left && !left_ok) {
      const auto fb = feedback(Side::Left);
      if (fb.have_feedback && std::fabs(fb.position_rad - target_rad) <= tol)
        left_ok = true;
    }
    if (wait_right && !right_ok) {
      const auto fb = feedback(Side::Right);
      if (fb.have_feedback && std::fabs(fb.position_rad - target_rad) <= tol)
        right_ok = true;
    }
    if (left_ok && right_ok)
      return true;

    if (!logged_stuck && std::chrono::steady_clock::now() >= stuck_after) {
      bool moved = false;
      if (wait_left && !left_ok) {
        const auto fb = feedback(Side::Left);
        moved = moved || moved_toward(left0, fb.position_rad, target_rad, 0.03);
      }
      if (wait_right && !right_ok) {
        const auto fb = feedback(Side::Right);
        moved = moved || moved_toward(right0, fb.position_rad, target_rad, 0.03);
      }
      if (!moved) {
        const auto lf = wait_left ? feedback(Side::Left) : SideFeedback{};
        const auto rf = wait_right ? feedback(Side::Right) : SideFeedback{};
        std::cerr << "[gripper] 编码器未跟随目标 " << target_rad
                  << " rad，提前结束等待（避免左右各卡满超时）\n";
        if (wait_left)
          log_merror("左未动", kLeftMotorId, lf.merror, lf.io_ok);
        if (wait_right)
          log_merror("右未动", kRightMotorId, rf.merror, rf.io_ok);
        return false;
      }
      logged_stuck = true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
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
  if (!hasSide(side)) {
    std::cout << "[gripper] "
              << (side == Side::Right ? "右" : "左")
              << "夹爪未配对，跳过合爪，手臂照常运动\n";
    out.result = GraspResult::PositionReached;
    return out;
  }
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
  if (!hasSide(side)) {
    std::cout << "[gripper] "
              << (side == Side::Right ? "右" : "左")
              << "夹爪未配对，跳过合爪，手臂照常运动\n";
    out.result = GraspResult::PositionReached;
    return out;
  }
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
  fb.merror = s.merror;
  fb.io_ok = s.io_ok;
  return fb;
}

bool Gripper::waitFor(Side side, double target_deg, double tolerance_deg,
                      std::chrono::milliseconds timeout) {
  if (!hasSide(side))
    return true;
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
  if (!hasSide(side))
    return true;
  const auto t0 = std::chrono::steady_clock::now();
  const auto deadline = t0 + timeout;
  const auto stuck_after = t0 + std::chrono::milliseconds(2500);
  const double tol = std::max(0.0, tolerance_rad);
  const double q0 = feedback(side).position_rad;
  bool logged_stuck = false;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto fb = feedback(side);
    if (fb.have_feedback && std::fabs(fb.position_rad - target_rad) <= tol) {
      return true;
    }
    if (!logged_stuck && std::chrono::steady_clock::now() >= stuck_after) {
      if (!moved_toward(q0, fb.position_rad, target_rad, 0.03)) {
        std::cerr << "[gripper] " << sideName(side) << "夹爪未跟随目标 " << target_rad
                  << " rad，提前结束等待\n";
        log_merror(sideName(side), side == Side::Right ? kRightMotorId : kLeftMotorId, fb.merror,
                   fb.io_ok);
        return false;
      }
      logged_stuck = true;
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

  int motor_id = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = sides_.find(side);
    if (it == sides_.end()) {
      std::cerr << "[gripper] 未找到 side=" << (side == Side::Left ? "left" : "right") << "\n";
      return false;
    }
    motor_id = it->second->motor_id;
  }

  try {
    set_Gripper_Calibrate(static_cast<std::uint8_t>(motor_id));
    std::cout << "[gripper] motor_id=" << motor_id << " 已下发 Dex1 标定\n";
    return true;
  } catch (const std::exception &ex) {
    std::cerr << "[gripper] motor_id=" << motor_id << " 标定失败: " << ex.what() << "\n";
    return false;
  }
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
  const float kp = static_cast<float>(config_.kp);
  const float kd = static_cast<float>(config_.kd);

  struct Pulse {
    Side side = Side::Left;
    int motor_id = 0;
    float q_cmd = 0.f;
  };

  try {
    while (!stop_requested_.load()) {
      const auto tick_start = clock::now();
      const double dt =
          std::max(0.0, std::chrono::duration<double>(tick_start - next_tick).count());

      std::vector<Pulse> pulses;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        const bool teleop = teleop_mode_.load();
        const bool teleop_soft = teleop_soft_mode_.load();
        for (auto &[side, impl] : sides_) {
          if (!impl->present)
            continue;
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
            impl->q_cmd = impl->target_q;
            impl->soft_torque_limited = false;
            updateMotionFilter(*impl);
            impl->q_cmd =
                clamp(impl->q_cmd, config_.close_position_rad, config_.open_position_rad);
          }
          pulses.push_back({side, impl->motor_id, static_cast<float>(impl->q_cmd)});
        }
      }

      for (const auto &p : pulses) {
        float q_fb = 0.f, dq_fb = 0.f, tau_fb = 0.f;
        std::uint32_t merror = 0;
        bool ok = false;
        try {
          ok = dex1_set_get(static_cast<std::uint8_t>(p.motor_id), p.q_cmd, 0.f, 0.f, kp, kd, q_fb,
                            dq_fb, tau_fb, merror);
        } catch (const std::exception &ex) {
          std::cerr << "[gripper] Dex1 IO 异常 id=" << p.motor_id << " " << ex.what() << "\n";
        }
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = sides_.find(p.side);
        if (it == sides_.end())
          continue;
        auto &impl = *it->second;
        impl.io_ok = ok;
        if (ok) {
          impl.q = q_fb;
          impl.dq = dq_fb;
          impl.tau = tau_fb;
          impl.merror = merror;
          impl.initialized = true;
          impl.io_fail_streak = 0;
        } else {
          ++impl.io_fail_streak;
        }
        if (ok && merror != impl.last_logged_merror) {
          log_merror("状态", p.motor_id, merror, true);
          impl.last_logged_merror = merror;
        } else if (!ok && impl.io_fail_streak == 1) {
          log_merror("无回包", p.motor_id, impl.merror, false);
        }
      }

      next_tick += std::chrono::duration_cast<clock::duration>(period);
      if (next_tick < clock::now())
        next_tick = clock::now();
      std::this_thread::sleep_until(next_tick);
    }
  } catch (const std::exception &ex) {
    std::cerr << "[gripper] 控制线程异常退出: " << ex.what() << "\n";
  }
}

}  // namespace gripper
