/**
 * DEX1 力矩限位夹取演示（与 main3 搬箱逻辑一致）
 *
 * 编译: ./build.sh
 * 运行: sudo ./build/gripper_grasp_demo
 */
#include "gripper_interface.hpp"

#include <iostream>

int main() {
  gripper::Config cfg;
  cfg.grasp_torque_limit_nm = 1.0;

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
    std::cout << "[" << name << "夹爪] 张开\n";
    g.openAndWait(side);

    std::cout << "[" << name << "夹爪] 夹取\n";
    const auto grasp = g.graspAndWait(side);
    std::cout << "  q=" << grasp.position_rad << " rad, torque=" << grasp.torque << " N·m\n";
  }

  g.stop();
  return 0;
}
