#include <cstdlib>
#include <iostream>
#include <string>

#include "lanxincontrol.h"

namespace {

std::string ReadLine(const std::string& prompt) {
  std::cout << prompt << std::flush;
  std::string s;
  std::getline(std::cin, s);
  return s;
}

double ReadDouble(const std::string& prompt, double def) {
  std::string s = ReadLine(prompt);
  if (s.empty()) return def;
  return std::strtod(s.c_str(), nullptr);
}

int ReadInt(const std::string& prompt, int def) {
  std::string s = ReadLine(prompt);
  if (s.empty()) return def;
  return static_cast<int>(std::strtol(s.c_str(), nullptr, 10));
}

void PrintMenu() {
  std::cout << "\n====== Lanxin 底盘控制菜单 ======\n";
  std::cout << "1  导航到点 VMR_moveTasks(x,y,theta)\n";
  std::cout << "2  导航任务 VmrNavTask(前进/倒车/全向/无头)\n";
  std::cout << "3  相对前进/后退(输入米数)\n";
  std::cout << "4  原地旋转(输入角度)\n";
  std::cout << "5  去充电 charge_task(x,y,theta)\n";
  std::cout << "6  取消充电 cancel_charge_task(last_task_id)\n";
  std::cout << "7  充电继电器 ON charge_relay(true)\n";
  std::cout << "8  充电继电器 OFF charge_relay(false)\n";
  std::cout << "9  查询电量(电池回调缓存)\n";
  std::cout << "10 设置速度比例 VMR_setSpeedFactor(0.0~1.0)\n";
  std::cout << "11 开启 SDK 控速 VMR_enableSdkCtrlSpeed(true)\n";
  std::cout << "12 关闭 SDK 控速 VMR_enableSdkCtrlSpeed(false)\n";
  std::cout << "0  退出\n";
}

}  // namespace

int main() {
  std::string ip = ReadLine("请输入底盘IP(默认192.168.10.49): ");
  if (ip.empty()) ip = "192.168.10.49";
  int port = ReadInt("请输入端口(默认9000): ", 9000);

  LanxinControl ctrl;
  if (!ctrl.Connect(ip, port)) {
    return 1;
  }

  while (true) {
    PrintMenu();
    int choice = ReadInt("请选择: ", -1);
    if (choice == 0) break;

    switch (choice) {
      case 1: {
        double x = ReadDouble("x(米): ", 0);
        double y = ReadDouble("y(米): ", 0);
        double th = ReadDouble("theta(弧度): ", 0);
        ctrl.MoveToPose(x, y, th);
        break;
      }
      case 2: {
        double x = ReadDouble("x(米): ", 0);
        double y = ReadDouble("y(米): ", 0);
        double th = ReadDouble("theta(弧度): ", 0);
        std::cout << "move_mode: -1无头 0前进 1倒车 2全向\n";
        int mm = ReadInt("move_mode: ", 0);
        std::cout << "navi_mode: 0避障点到点 1直线\n";
        int nm = ReadInt("navi_mode: ", 0);
        bool forbid = ReadInt("forbid_rotation_on_start 0/1: ", 0) != 0;
        ctrl.MoveWithNavTask(x, y, th, mm, nm, forbid);
        break;
      }
      case 3: {
        std::cout << "1前进 2后退\n";
        int dir = ReadInt("方向: ", 1);
        double d = ReadDouble("距离(米): ", 0.5);
        ctrl.MoveForwardOrBackward(d, dir == 1);
        break;
      }
      case 4: {
        double deg = ReadDouble("旋转角度(左正右负, 度): ", 15);
        ctrl.RotateInPlace(deg);
        break;
      }
      case 5: {
        double x = ReadDouble("充电点x(默认-1.58): ", -1.58);
        double y = ReadDouble("充电点y(默认1.713): ", 1.713);
        double th = ReadDouble("充电点theta(默认3.1416): ", 3.1416);
        ctrl.StartChargeTask(x, y, th);
        break;
      }
      case 6:
        ctrl.CancelChargeTask();
        break;
      case 7:
        ctrl.ChargeRelay(true);
        break;
      case 8:
        ctrl.ChargeRelay(false);
        break;
      case 9: {
        ctrl.PrintBattery();
        break;
      }
      case 10: {
        double f = ReadDouble("速度比例 factor(0.0~1.0, 建议 <=0.8): ", 0.8);
        ctrl.SetSpeedFactor(f);
        break;
      }
      case 11: {
        ctrl.EnableSdkCtrlSpeed(true);
        break;
      }
      case 12: {
        ctrl.EnableSdkCtrlSpeed(false);
        break;
      }
      default:
        std::cout << "未知选项\n";
        break;
    }
  }

  ctrl.Disconnect();
  std::cout << "程序已退出\n";
  return 0;
}
