/**
 * DEX1 夹爪演示（直接串口，无需 DDS / 无需 dex1_1_gripper_server）
 *
 * 编译: ./build.sh
 * 运行: sudo ./build/gripper_demo
 */
#include "gripper_interface.hpp"

#include <atomic>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>

namespace {

std::atomic<bool> g_running{true};

void onSignal(int) { g_running = false; }

void printFeedback(const gripper::Gripper &g) {
  const auto left = g.feedback(gripper::Side::Left);
  const auto right = g.feedback(gripper::Side::Right);
  std::cout << "  left=" << left.position_rad << " rad, right=" << right.position_rad << " rad\n";
}

}  // namespace

int main() {
  std::signal(SIGINT, onSignal);

  std::cout << "DEX1 夹爪演示（直接串口）\n";
  std::cout << "请使用 sudo 运行。\n";
  std::cout << "首次使用请先运行: sudo ./build/gripper_calibrate\n\n";

  gripper::Gripper gripper;
  if (!gripper.start()) {
    std::cerr << "夹爪启动失败，请检查 USB 串口与电源。\n";
    return 1;
  }

  const auto motors = gripper.detectedMotors();
  for (const auto &m : motors) {
    std::cout << "  motor_id=" << m.motor_id << " port=" << m.port << "\n";
  }

  const double full_close = gripper.config().full_close_angle_deg;

  std::cout << "\n=== 步骤 1: 双夹爪张开 ===\n";
  gripper.openBoth();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  printFeedback(gripper);

  std::cout << "=== 步骤 2: 左夹爪闭合 ===\n";
  gripper.close(gripper::Side::Left);
  gripper.waitFor(gripper::Side::Left, full_close, 0.08);
  printFeedback(gripper);
  std::this_thread::sleep_for(std::chrono::milliseconds(800));

  std::cout << "=== 步骤 3: 右夹爪闭合 ===\n";
  gripper.close(gripper::Side::Right);
  gripper.waitFor(gripper::Side::Right, full_close, 0.08);
  printFeedback(gripper);
  std::this_thread::sleep_for(std::chrono::milliseconds(800));

  std::cout << "=== 步骤 4: 正弦往复（3 周期）===\n";
  for (int c = 0; c < 3 && g_running; ++c) {
    for (int i = 0; i <= 40 && g_running; ++i) {
      const double phase = 2.0 * M_PI * static_cast<double>(i) / 40.0;
      const double ratio = 0.5 * (1.0 - std::cos(phase));
      gripper.setBothAngles(full_close * ratio, full_close * (1.0 - ratio));
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  std::cout << "=== 步骤 5: 复位张开 ===\n";
  gripper.openBoth();
  gripper.waitFor(gripper::Side::Left, 0.0, 0.08);
  gripper.waitFor(gripper::Side::Right, 0.0, 0.08);
  printFeedback(gripper);

  std::cout << "演示完成\n";
  gripper.stop();
  return 0;
}
