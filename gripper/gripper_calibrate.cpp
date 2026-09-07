/**
 * DEX1 夹爪标定程序（直接串口，无需 dex1_1_gripper_server）
 *
 * 用法：
 *   sudo ./gripper_calibrate
 *
 * 流程（与官方文档一致）：
 *   1. 手动将夹爪紧闭到极限
 *   2. 输入 s 并回车开始标定
 *   3. 左右夹爪依次标定
 *
 * 文档: https://support.unitree.com/home/zh/dex1-1_gripper/dex1_1
 */
#include "gripper_interface.hpp"

#include <iostream>

int main() {
  std::cout << "DEX1 夹爪标定工具（直接串口）\n";
  std::cout << "请使用 sudo 运行以确保串口权限。\n\n";

  gripper::Gripper gripper;
  if (!gripper.calibrateInteractive()) {
    std::cerr << "标定未完成或失败。\n";
    return 1;
  }
  return 0;
}
