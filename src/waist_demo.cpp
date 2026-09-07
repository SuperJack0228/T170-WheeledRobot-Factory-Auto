/**
 * 腰部 TCP 直线 demo：把腰当末端，沿基座 X（及可选 Z）缓慢笛卡尔插补。
 * 三轴 CAN ID 4/3/2，最上一轴为 ID 2。
 *
 * Ctrl+C / SIGTERM：停止下发，保持最后一拍位置后退出。
 * 急停断电：CAN 读编码器失败 → 同样停发，避免上电后继续插补。
 *
 * 运行（需 CAN 权限）：
 *   cd build && sudo ./waist_demo
 */

#include "Ti5_socketcan.h"
#include "waist.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>

namespace
{

std::atomic<bool> g_stop{false};
std::atomic<bool> g_estop{false};

constexpr double kDefaultSpeedMps = 0.02;
constexpr double kMinSpeedMps = 0.005;
constexpr double kMaxSpeedMps = 0.08;
constexpr double kXMin = -0.32;
constexpr double kXMax = 0.20;
constexpr double kZMin = 0.50;
constexpr double kZMax = 0.75;
constexpr uint32_t kWaistTopMotorCanId = 2; // MotorsIDlist[2]，腰链最上一轴
constexpr int kCanFailLimit = 3;

// 与 config/move_box_params.yaml waist.layer3_home 一致（deg）
const Eigen::Matrix<double, 1, 6> kLayer3HomeDeg =
    (Eigen::Matrix<double, 1, 6>() << -0.132502, 0.0, 0.642464, -90.0, -90.0, 180.0).finished();

void on_stop_signal(int)
{
    g_stop.store(true, std::memory_order_relaxed);
}

void install_demo_signal_handlers()
{
    struct sigaction sa {};
    sa.sa_handler = on_stop_signal;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
}

bool ping_waist_top_motor()
{
    if (waist_id < 0)
        return false;

    const uint8_t cmd = 8;
    if (!send_can_frame(waist_id, kWaistTopMotorCanId, &cmd, 1))
        return false;

    uint8_t recv[8] = {};
    for (int i = 0; i < 12; ++i)
    {
        if (receive_can_frame_timeout(waist_id, recv))
            return true;
    }
    return false;
}

bool should_abort_motion()
{
    if (g_stop.load(std::memory_order_relaxed))
        return true;

    // 约每 100ms 探测一次最上轴，避免打满 5ms 插补周期
    static int cycle = 0;
    static int fail_streak = 0;
    if ((++cycle % 20) != 0)
        return false;

    if (ping_waist_top_motor())
    {
        fail_streak = 0;
        return false;
    }
    ++fail_streak;
    if (fail_streak >= kCanFailLimit)
    {
        g_estop.store(true, std::memory_order_relaxed);
        g_stop.store(true, std::memory_order_relaxed);
        std::cerr << "\n[estop] 腰最上轴 CAN 无应答（急停/断电/掉线），停止下发\n";
        return true;
    }
    return false;
}

void print_tcp(const char *tag, const Eigen::Matrix<double, 1, 6> &pos)
{
    std::cout << std::fixed << std::setprecision(4)
              << tag << "  x=" << pos(0) << "  y=" << pos(1) << "  z=" << pos(2)
              << "  rx=" << pos(3) * rad2deg << "  ry=" << pos(4) * rad2deg
              << "  rz=" << pos(5) * rad2deg << " deg\n";
}

void print_joints(WaistRobot &waist)
{
    const Eigen::Matrix<double, 1, 3> q = waist.getJointPos();
    std::cout << std::fixed << std::setprecision(2)
              << "[joint] q1=" << q(0) * rad2deg << "  q2=" << q(1) * rad2deg
              << "  q3=" << q(2) * rad2deg << " deg  (电机 CAN 4 / 3 / 2)\n";
}

void print_help()
{
    std::cout
        << "\n腰 TCP 直线 demo（速度默认 " << kDefaultSpeedMps << " m/s）\n"
        << "  status              打印当前 TCP / 关节\n"
        << "  speed [v]           查看或设置线速度 m/s  (" << kMinSpeedMps << " ~ " << kMaxSpeedMps << ")\n"
        << "  dx <m>              沿基座 X 直线（正=前进，例: dx 0.03）\n"
        << "  dz <m>              沿基座 Z 直线（例: dz -0.02）\n"
        << "  goto <x>            只改 X 到绝对值 (m)，姿态保持\n"
        << "  home                缓慢回到 layer3_home\n"
        << "  q / quit            退出\n"
        << "  Ctrl+C              立即停止运动并退出\n"
        << "X 限位 [" << kXMin << ", " << kXMax << "] m，Z 限位 [" << kZMin << ", " << kZMax << "] m\n\n";
}

Eigen::Matrix<double, 1, 6> layer3_home_rad()
{
    Eigen::Matrix<double, 1, 6> p = kLayer3HomeDeg;
    p(3) *= deg2rad;
    p(4) *= deg2rad;
    p(5) *= deg2rad;
    return p;
}

bool clamp_goal(Eigen::Matrix<double, 1, 6> &goal, std::string &err)
{
    if (goal(0) < kXMin || goal(0) > kXMax)
    {
        err = "目标 x=" + std::to_string(goal(0)) + " 超出 [" + std::to_string(kXMin) + ", " +
              std::to_string(kXMax) + "]";
        return false;
    }
    if (goal(2) < kZMin || goal(2) > kZMax)
    {
        err = "目标 z=" + std::to_string(goal(2)) + " 超出 [" + std::to_string(kZMin) + ", " +
              std::to_string(kZMax) + "]";
        return false;
    }
    return true;
}

int move_tcp_slow(WaistRobot &waist, Eigen::Matrix<double, 1, 6> goal, double speed_mps)
{
    std::string err;
    if (!clamp_goal(goal, err))
    {
        std::cerr << "[skip] " << err << "\n";
        return -2;
    }

    Eigen::Matrix<double, 1, 6> cur = waist.getTcpPos();
    print_tcp("[from]", cur);
    print_tcp("[to]  ", goal);
    std::cout << "[move] 线速度 " << speed_mps << " m/s，Ctrl+C 可随时停\n";

    const int ret = waist.moveLToPos(goal, speed_mps, should_abort_motion);
    Eigen::Matrix<double, 1, 6> after = waist.getTcpPos();
    print_tcp("[now] ", after);
    print_joints(waist);

    if (ret == -3)
        std::cout << "[move] 已中止\n";
    else if (ret != 0)
        std::cerr << "[move] 失败 ret=" << ret << "（限位或 IK）\n";
    else
        std::cout << "[move] 完成\n";
    return ret;
}

bool read_line_interruptible(std::string &line)
{
    line.clear();
    int can_fail = 0;
    while (!g_stop.load(std::memory_order_relaxed))
    {
        pollfd pfd{};
        pfd.fd = STDIN_FILENO;
        pfd.events = POLLIN;
        const int n = poll(&pfd, 1, 100);
        if (n < 0)
        {
            if (errno == EINTR)
                continue;
            return false;
        }
        if (n == 0)
        {
            if (ping_waist_top_motor())
            {
                can_fail = 0;
            }
            else if (++can_fail >= kCanFailLimit)
            {
                g_estop.store(true, std::memory_order_relaxed);
                g_stop.store(true, std::memory_order_relaxed);
                std::cerr << "\n[estop] 等待输入时 CAN 无应答，退出\n";
                return false;
            }
            continue;
        }
        if (pfd.revents & POLLIN)
        {
            if (!std::getline(std::cin, line))
                return false;
            return true;
        }
    }
    return false;
}

std::vector<std::string> split_ws(const std::string &s)
{
    std::istringstream iss(s);
    std::vector<std::string> out;
    std::string tok;
    while (iss >> tok)
        out.push_back(tok);
    return out;
}

} // namespace

std::string project_root_dir()
{
#ifdef MARKET_SIMPLE_SOURCE_DIR
    return MARKET_SIMPLE_SOURCE_DIR;
#else
    return ".";
#endif
}

int main(int argc, char **argv)
{
    install_demo_signal_handlers();

    double speed = kDefaultSpeedMps;
    bool auto_dx = false;
    double auto_dx_m = 0.0;
    for (int i = 1; i < argc; ++i)
    {
        const std::string a = argv[i];
        if ((a == "--speed" || a == "-s") && i + 1 < argc)
            speed = std::clamp(std::stod(argv[++i]), kMinSpeedMps, kMaxSpeedMps);
        else if ((a == "--dx") && i + 1 < argc)
        {
            auto_dx = true;
            auto_dx_m = std::stod(argv[++i]);
        }
        else if (a == "-h" || a == "--help")
        {
            print_help();
            return 0;
        }
    }

    std::cout << "[init] SocketCAN …\n";
    if (!init_socketcan())
    {
        std::cerr << "init_socketcan 失败（需要 sudo / CAN 接口）\n";
        return 1;
    }
    hand_socketid_bind();
    if (waist_id < 0)
    {
        std::cerr << "未绑定 waist_id\n";
        close_can_channels();
        return 1;
    }
    std::cout << "[init] waist_id=can" << waist_id << "  最上轴 CAN ID=" << kWaistTopMotorCanId << "\n";

    if (!ping_waist_top_motor())
    {
        std::cerr << "腰最上轴无应答，请检查急停、供电和 CAN\n";
        close_can_channels();
        return 1;
    }

    WaistRobot waist;
    print_help();
    print_tcp("[tcp]", waist.getTcpPos());
    print_joints(waist);

    auto run_dx = [&](double dx) {
        Eigen::Matrix<double, 1, 6> goal = waist.getTcpPos();
        goal(0) += dx;
        return move_tcp_slow(waist, goal, speed);
    };

    if (auto_dx)
    {
        run_dx(auto_dx_m);
        close_can_channels();
        return g_stop.load() ? 130 : 0;
    }

    while (!g_stop.load(std::memory_order_relaxed))
    {
        std::cout << "waist> " << std::flush;
        std::string line;
        if (!read_line_interruptible(line))
            break;
        const auto toks = split_ws(line);
        if (toks.empty())
            continue;

        const std::string &cmd = toks[0];
        try
        {
            if (cmd == "q" || cmd == "quit" || cmd == "exit")
                break;
            if (cmd == "help" || cmd == "h")
            {
                print_help();
                continue;
            }
            if (cmd == "status")
            {
                print_tcp("[tcp]", waist.getTcpPos());
                print_joints(waist);
                continue;
            }
            if (cmd == "speed")
            {
                if (toks.size() >= 2)
                    speed = std::clamp(std::stod(toks[1]), kMinSpeedMps, kMaxSpeedMps);
                std::cout << "[speed] " << speed << " m/s\n";
                continue;
            }
            if (cmd == "dx" && toks.size() >= 2)
            {
                run_dx(std::stod(toks[1]));
                continue;
            }
            if (cmd == "dz" && toks.size() >= 2)
            {
                Eigen::Matrix<double, 1, 6> goal = waist.getTcpPos();
                goal(2) += std::stod(toks[1]);
                move_tcp_slow(waist, goal, speed);
                continue;
            }
            if (cmd == "goto" && toks.size() >= 2)
            {
                Eigen::Matrix<double, 1, 6> goal = waist.getTcpPos();
                goal(0) = std::stod(toks[1]);
                move_tcp_slow(waist, goal, speed);
                continue;
            }
            if (cmd == "home")
            {
                move_tcp_slow(waist, layer3_home_rad(), speed);
                continue;
            }
            std::cout << "未知命令，输入 help\n";
        }
        catch (const std::exception &e)
        {
            std::cerr << "参数错误: " << e.what() << "\n";
        }
    }

    if (g_estop.load())
        std::cout << "[exit] 因急停/CAN 失联退出，已停止下发\n";
    else if (g_stop.load())
        std::cout << "[exit] Ctrl+C，已停止下发，电机保持最后位置\n";
    else
        std::cout << "[exit] 正常退出\n";

    close_can_channels();
    return g_stop.load() ? 130 : 0;
}
