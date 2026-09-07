/**
 * Soft 力矩限位夹取演示（ros2_170D 同款 kp/kd 反算）
 *
 * 编译: ./build.sh
 * 运行: sudo ./build/gripper_grasp_soft_demo
 */
#include "gripper_interface.hpp"

#include <iostream>

int main() {
  gripper::Config cfg;
  cfg.soft_torque_limit_nm = 2.0;
  cfg.pos_filter = 0.2;
  cfg.soft_slew_rate = 10.0;

  gripper::Gripper g(cfg);
  if (!g.start()) {
    std::cerr << "夹爪启动失败\n";
    return 1;
  }

  for (const auto side : {gripper::Side::Left, gripper::Side::Right}) {
    if (!g.hasSide(side)) {
      continue;
    }
    const char *name = (side == gripper::Side::Left) ? "左" : "右";
    std::cout << "[" << name << "夹爪] Soft 张开\n";
    g.openSoftAndWait(side);

    std::cout << "[" << name << "夹爪] Soft 夹取\n";
    const auto grasp = g.graspSoftAndWait(side);
    std::cout << "  q=" << grasp.position_rad << " rad, torque=" << grasp.torque << " N·m\n";
  }

  g.stop();
  return 0;
}
