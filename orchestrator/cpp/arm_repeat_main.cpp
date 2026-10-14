/** 双臂水平面重复到位测试。
 *
 *  编译：cmake --build /home/ti5robot/T170Clean/build_robot -j2 --target arm_plane_repeat
 *  运行：./build_robot/arm_plane_repeat [--cycles 5] [--span 0.04] [--vel 0.08]
 *
 *  先按现有 home 回正：头到标定角，双臂分段抬离再走到 yaml home_tcp，腰先回 x 再升到 home。
 *  再按现有分段路径走到放置准备 standby。然后腰按放置业务蹲到 ready：先前伸，再降到 ready_z。
 *  蹲稳后双臂保持该高度和腕角，在该水平面上走同一组偏移，重复若干次。
 *  不打开相机，不动底盘，不开爪。Ctrl+C 会中止动作，退出前释放 CAN。
 *  记录的是编码器正运动学相对指令点的偏差，不是外部测量的绝对精度。
 */

#include "robot_runtime.h"

#include "function.h"
#include "gripper_interface.hpp"
#include "move_box_config.h"
#include "move_box_runtime.h"
#include "seg_pose_bridge.h"
#include "Ti5_socketcan.h"
#include "waist.h"

#include <atomic>
#include <chrono>
#include <ctime>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

MoveBoxConfig g_move_cfg;

namespace
{

std::unique_ptr<gripper::Gripper> g_repeat_grip;

void release_devices_on_exit()
{
    static bool done = false;
    if (done)
        return;
    done = true;
    mark_app_cleanup_started();
    std::cerr << "[repeat] 退出，释放 CAN\n" << std::flush;
    if (g_repeat_grip)
        g_repeat_grip->stop();
    close_can_channels();
    std::cerr << "[repeat] CAN 已释放\n" << std::flush;
}

struct Options
{
    int cycles = 5;
    double span_m = 0.04;
    double vel_m_s = 0.08;
    int settle_ms = 400;
};

struct Sample
{
    int cycle = 0;
    std::string point;
    std::string hand;
    double cmd_x = 0, cmd_y = 0, cmd_z = 0;
    double act_x = 0, act_y = 0, act_z = 0;
};

bool parse_args(int argc, char **argv, Options &opt, std::string &err)
{
    for (int i = 1; i < argc; ++i)
    {
        const std::string key = argv[i];
        auto need = [&](double &dst) -> bool {
            if (i + 1 >= argc)
            {
                err = key + " 缺少数值";
                return false;
            }
            dst = std::atof(argv[++i]);
            return true;
        };
        if (key == "--cycles")
        {
            double v = 0;
            if (!need(v))
                return false;
            opt.cycles = static_cast<int>(v);
        }
        else if (key == "--span")
        {
            if (!need(opt.span_m))
                return false;
        }
        else if (key == "--vel")
        {
            if (!need(opt.vel_m_s))
                return false;
        }
        else if (key == "--settle-ms")
        {
            double v = 0;
            if (!need(v))
                return false;
            opt.settle_ms = static_cast<int>(v);
        }
        else if (key == "--help" || key == "-h")
        {
            err = "usage";
            return false;
        }
        else
        {
            err = "未知参数 " + key;
            return false;
        }
    }
    if (opt.cycles < 1 || opt.cycles > 20)
    {
        err = "--cycles 取 1 到 20";
        return false;
    }
    if (!(opt.span_m >= 0.01 && opt.span_m <= 0.06))
    {
        err = "--span 取 0.01 到 0.06 米，默认 0.04";
        return false;
    }
    if (!(opt.vel_m_s >= 0.02 && opt.vel_m_s <= 0.12))
    {
        err = "--vel 取 0.02 到 0.12 m/s，默认 0.08";
        return false;
    }
    if (opt.settle_ms < 0 || opt.settle_ms > 5000)
    {
        err = "--settle-ms 取 0 到 5000";
        return false;
    }
    return true;
}

bool init_like_debug(Robot_Arm *&arm_r, Robot_Arm *&arm_l, std::string &err)
{
    if (!load_move_box_config(default_move_box_config_path(), g_move_cfg, err))
        return false;
    print_move_box_config(g_move_cfg);

    if (!init_socketcan())
    {
        err = device_scan_error();
        return false;
    }
    hand_socketid_bind();

    static std::unique_ptr<Robot_Arm> right;
    static std::unique_ptr<Robot_Arm> left;
    static std::unique_ptr<WaistRobot> waist;
    static Eigen::Matrix<double, 1, 6> layer3;
    static orch::HwSession session;
    static std::atomic<bool> abort_flag{false};

    gripper::Config grip_cfg;
    grip_cfg.grasp_torque_limit_nm = 4.5;
    grip_cfg.soft_torque_limit_nm = 4.0;
    grip_cfg.pos_filter = 0.5;
    grip_cfg.soft_slew_rate = 100.0;
    grip_cfg.soft_coast_margin_rad = 0.12;
    grip_cfg.detect_retries = 2;
    try
    {
        g_repeat_grip = std::make_unique<gripper::Gripper>(grip_cfg);
        if (!g_repeat_grip->start())
        {
            err = "夹爪启动失败：一个夹爪都没有";
            return false;
        }
    }
    catch (const std::exception &ex)
    {
        err = std::string("夹爪初始化失败: ") + ex.what();
        return false;
    }

    const std::string arm_yaml = project_root_dir() + "/config/Robot_Arm_Model.yaml";
    right = std::make_unique<Robot_Arm>("T7", "T170", "right", arm_yaml);
    left = std::make_unique<Robot_Arm>("T7", "T170", "left", arm_yaml);
    waist = std::make_unique<WaistRobot>();
    rev_motor_error(0);
    rev_motor_error(1);
    motor_speed_change();

    layer3 = g_move_cfg.waist.layer3_home;
    if (!move_box::sync_waist_layer3_from_encoders(*waist, layer3, "启动"))
    {
        layer3 = g_move_cfg.waist.layer3_home;
        std::cerr << "[waist] 警告：腰部位置未知，仍继续 home，请确认现场允许腰运动\n";
    }

    session = orch::HwSession{};
    session.g = g_repeat_grip.get();
    session.arm_r = right.get();
    session.arm_l = left.get();
    session.waist = waist.get();
    session.layer3 = &layer3;
    session.abort = &abort_flag;
    orch::g_hw = &session;
    can_io_fault_clear();

    arm_r = right.get();
    arm_l = left.get();
    return true;
}

struct Offset
{
    const char *name;
    double dx;
    double dy;
};

std::vector<Offset> make_grid(double span)
{
    const double s = span;
    return {
        {"x- y-", -s, -s},
        {"x0 y-", 0.0, -s},
        {"x+ y-", s, -s},
        {"x+ y0", s, 0.0},
        {"x0 y0", 0.0, 0.0},
        {"x- y0", -s, 0.0},
        {"x- y+", -s, s},
        {"x0 y+", 0.0, s},
        {"x+ y+", s, s},
        {"home", 0.0, 0.0},
    };
}

bool plane_is_safe(const Eigen::Matrix<double, 1, 6> &home_r,
                   const Eigen::Matrix<double, 1, 6> &home_l,
                   double span,
                   std::string &err)
{
    const double y_gap = home_l(1) - home_r(1);
    if (y_gap < 0.40)
    {
        err = "放置准备左右间距小于 0.40 m，拒绝在这个高度上对向平移";
        return false;
    }
    const double right_in = home_r(1) + span;
    const double left_in = home_l(1) - span;
    if (right_in > -0.15 || left_in < 0.15)
    {
        err = "水平偏移会让双手靠近中缝，拒绝执行";
        return false;
    }
    const double x_lo = std::min(home_r(0), home_l(0)) - span;
    const double x_hi = std::max(home_r(0), home_l(0)) + span;
    if (x_lo < 0.30 || x_hi > 0.60)
    {
        err = "水平偏移超出 x=[0.30,0.60] m，拒绝执行";
        return false;
    }
    return true;
}

void log_sample(std::ostream &csv, const Sample &s)
{
    const double dx = (s.act_x - s.cmd_x) * 1000.0;
    const double dy = (s.act_y - s.cmd_y) * 1000.0;
    const double dz = (s.act_z - s.cmd_z) * 1000.0;
    const double err = std::sqrt(dx * dx + dy * dy + dz * dz);
    csv << s.cycle << ',' << s.point << ',' << s.hand << ','
        << std::fixed << std::setprecision(6)
        << s.cmd_x << ',' << s.cmd_y << ',' << s.cmd_z << ','
        << s.act_x << ',' << s.act_y << ',' << s.act_z << ','
        << std::setprecision(3) << err << ',' << dx << ',' << dy << ',' << dz << '\n';
    std::cout << std::fixed << std::setprecision(2)
              << "[repeat] 第" << s.cycle << "轮 " << s.point << ' ' << s.hand
              << " 偏差 " << err << " mm  (dx " << dx << " dy " << dy << " dz " << dz << ")\n";
}

bool stopped_or_abort()
{
    return hardware_abort_requested() || app_stop_requested();
}

int visit_point(Robot_Arm &arm_r,
                Robot_Arm &arm_l,
                const Eigen::Matrix<double, 1, 6> &home_r,
                const Eigen::Matrix<double, 1, 6> &home_l,
                const Offset &off,
                int cycle,
                const Options &opt,
                std::ostream &csv,
                std::vector<Sample> &samples)
{
    if (stopped_or_abort())
        return -4;
    Eigen::Matrix<double, 1, 6> goal_r = home_r;
    Eigen::Matrix<double, 1, 6> goal_l = home_l;
    goal_r(0) += off.dx;
    goal_r(1) += off.dy;
    goal_l(0) += off.dx;
    goal_l(1) += off.dy;

    std::cout << std::fixed << std::setprecision(4)
              << "[repeat] 第" << cycle << "轮 → " << off.name
              << "  右xy=(" << goal_r(0) << "," << goal_r(1) << ")"
              << " 左xy=(" << goal_l(0) << "," << goal_l(1) << ")"
              << " z=" << goal_r(2) << " 速度 " << opt.vel_m_s << " m/s\n";

    const ArmLineMoveResult moved = arm_dual_line_move_selective(
        arm_r, goal_r, true, arm_l, goal_l, true, opt.vel_m_s);
    if (moved.ret_r != 0 || moved.ret_l != 0)
    {
        std::cerr << "[repeat] 直线规划/执行失败 右=" << moved.ret_r
                  << " 左=" << moved.ret_l << "\n";
        return moved.ret_r != 0 ? moved.ret_r : moved.ret_l;
    }
    if (!wait_arms_motors_stopped(true, true, 8.0, 6, "水平到位"))
    {
        std::cerr << "[repeat] 到位等待超时\n";
        return -8;
    }
    if (opt.settle_ms > 0)
        hardware_abort_sleep_ms(opt.settle_ms);
    if (stopped_or_abort())
        return -4;

    const Eigen::Matrix<double, 1, 6> act_r = arm_get_tcp_pos(arm_r);
    const Eigen::Matrix<double, 1, 6> act_l = arm_get_tcp_pos(arm_l);
    Sample sr;
    sr.cycle = cycle;
    sr.point = off.name;
    sr.hand = "右";
    sr.cmd_x = goal_r(0);
    sr.cmd_y = goal_r(1);
    sr.cmd_z = goal_r(2);
    sr.act_x = act_r(0);
    sr.act_y = act_r(1);
    sr.act_z = act_r(2);
    Sample sl = sr;
    sl.hand = "左";
    sl.cmd_x = goal_l(0);
    sl.cmd_y = goal_l(1);
    sl.cmd_z = goal_l(2);
    sl.act_x = act_l(0);
    sl.act_y = act_l(1);
    sl.act_z = act_l(2);
    samples.push_back(sr);
    samples.push_back(sl);
    log_sample(csv, sr);
    log_sample(csv, sl);
    return 0;
}

void print_summary(const std::vector<Sample> &samples)
{
    std::cout << "\n[repeat] 汇总（毫米）。到位偏差是相对指令点；重复偏差是各次落点相对自身平均位置。\n";
    std::vector<std::string> keys;
    for (const Sample &s : samples)
    {
        const std::string key = s.hand + " " + s.point;
        bool seen = false;
        for (const std::string &k : keys)
        {
            if (k == key)
            {
                seen = true;
                break;
            }
        }
        if (!seen)
            keys.push_back(key);
    }
    std::cout << std::fixed << std::setprecision(2);
    for (const std::string &key : keys)
    {
        double sx = 0, sy = 0, sz = 0;
        double ex = 0, ey = 0, ez = 0;
        int n = 0;
        for (const Sample &s : samples)
        {
            if (s.hand + " " + s.point != key)
                continue;
            sx += s.act_x;
            sy += s.act_y;
            sz += s.act_z;
            ex += s.act_x - s.cmd_x;
            ey += s.act_y - s.cmd_y;
            ez += s.act_z - s.cmd_z;
            ++n;
        }
        if (n <= 0)
            continue;
        const double mx = sx / n, my = sy / n, mz = sz / n;
        double max_repeat = 0;
        double sum_sq = 0;
        for (const Sample &s : samples)
        {
            if (s.hand + " " + s.point != key)
                continue;
            const double d = std::sqrt((s.act_x - mx) * (s.act_x - mx) +
                                       (s.act_y - my) * (s.act_y - my) +
                                       (s.act_z - mz) * (s.act_z - mz));
            sum_sq += d * d;
            if (d > max_repeat)
                max_repeat = d;
        }
        const double bias = std::sqrt((ex / n) * (ex / n) + (ey / n) * (ey / n) + (ez / n) * (ez / n));
        std::cout << "  " << key
                  << "  n=" << n
                  << "  到位偏差均值 " << (bias * 1000.0) << " mm"
                  << "  重复RMS " << (std::sqrt(sum_sq / n) * 1000.0) << " mm"
                  << "  重复最大 " << (max_repeat * 1000.0) << " mm\n";
    }
}

} // namespace

int main(int argc, char **argv)
{
    struct ExitRelease
    {
        ~ExitRelease() { release_devices_on_exit(); }
    } exit_release;

    Options opt;
    std::string err;
    if (!parse_args(argc, argv, opt, err))
    {
        std::cerr << "双臂水平面重复到位\n"
                  << "  --cycles N     重复次数，默认 5，最大 20\n"
                  << "  --span M       相对放置准备点的水平半宽，默认 0.04，最大 0.06\n"
                  << "  --vel M/S      直线速度，默认 0.08\n"
                  << "  --settle-ms N  停稳后再等，默认 400\n";
        if (!err.empty() && err != "usage")
            std::cerr << err << "\n";
        return err == "usage" ? 0 : 2;
    }

    install_app_sigint_handler();
    Robot_Arm *arm_r = nullptr;
    Robot_Arm *arm_l = nullptr;
    if (!init_like_debug(arm_r, arm_l, err))
    {
        std::cerr << "[repeat] 初始化失败: " << err << "\n";
        return 1;
    }
    if (right_arm_motors_locked())
    {
        std::cerr << "[repeat] 右臂电机锁定，双臂测试不启动\n";
        return 1;
    }

    const Eigen::Matrix<double, 1, 6> place_r = g_move_cfg.standby.right;
    const Eigen::Matrix<double, 1, 6> place_l = g_move_cfg.standby.left;
    if (!plane_is_safe(place_r, place_l, opt.span_m, err))
    {
        std::cerr << "[repeat] " << err << "\n";
        return 2;
    }

    std::cout << "[repeat] 先按现有 home 缓慢回正：头标定 → 双臂分段到 home_tcp → 腰回 x 再升高。不开爪，不动底盘。\n";
    const int home_rc = orch::go_home(*arm_r, *arm_l);
    if (home_rc != 0)
    {
        std::cerr << "[repeat] home 失败 code=" << home_rc << "，不开始水平测试\n";
        return home_rc == -4 ? 4 : 1;
    }
    std::cout << "[repeat] 双臂按现有分段路径到放置准备 standby，腰仍在 home。\n";
    if (!move_box::move_arms_to_standby(*arm_r, *arm_l))
    {
        std::cerr << "[repeat] 到放置准备失败，不开始水平测试\n";
        orch::go_home(*arm_r, *arm_l);
        return stopped_or_abort() ? 4 : 1;
    }
    if (orch::g_hw == nullptr || orch::g_hw->waist == nullptr || orch::g_hw->layer3 == nullptr)
    {
        std::cerr << "[repeat] 腰未初始化，不开始水平测试\n";
        orch::go_home(*arm_r, *arm_l);
        return 1;
    }
    std::cout << "[repeat] 手臂已在放置准备。腰按放置业务蹲下：先前伸 "
              << g_move_cfg.waist.ready_forward_m << " m，再降到 z="
              << g_move_cfg.waist.ready_z_m << " m，然后才开始水平移动。\n";
    if (!move_box::ensure_waist_ready_start(*orch::g_hw->waist, *orch::g_hw->layer3))
    {
        std::cerr << "[repeat] 蹲到放置腰高失败，收回 home\n";
        orch::go_home(*arm_r, *arm_l);
        return stopped_or_abort() ? 4 : 1;
    }
    move_box::waist_layer3_set_synced(true);

    const auto stamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{};
    localtime_r(&stamp, &tm);
    std::ostringstream name;
    name << "arm_plane_repeat_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";
    const std::filesystem::path csv_path = std::filesystem::path(project_root_dir()) / "logs" / name.str();
    std::error_code ec;
    std::filesystem::create_directories(csv_path.parent_path(), ec);
    std::ofstream csv(csv_path);
    if (!csv)
    {
        std::cerr << "[repeat] 无法写 " << csv_path << "\n";
        return 1;
    }
    csv << "cycle,point,hand,cmd_x,cmd_y,cmd_z,act_x,act_y,act_z,err_mm,dx_mm,dy_mm,dz_mm\n";

    std::cout << std::fixed << std::setprecision(3)
              << "[repeat] 水平面以放置准备为中心 z=" << place_r(2)
              << " m，腕角保持 standby，半宽 " << opt.span_m
              << " m，共 " << opt.cycles << " 轮。双手同步平移，间距保持不变。腰保持蹲姿。\n"
              << "[repeat] 记录 " << csv_path << "\n";

    const std::vector<Offset> grid = make_grid(opt.span_m);
    std::vector<Sample> samples;
    int rc = 0;
    for (int cycle = 1; cycle <= opt.cycles && rc == 0; ++cycle)
    {
        std::cout << "\n======== [repeat] 第 " << cycle << " / " << opt.cycles << " 轮 ========\n";
        for (const Offset &off : grid)
        {
            rc = visit_point(*arm_r, *arm_l, place_r, place_l, off, cycle, opt, csv, samples);
            if (rc != 0)
                break;
        }
    }
    csv.flush();
    if (!samples.empty())
        print_summary(samples);

    std::cout << "[repeat] 测试结束，再走一次现有 home\n";
    const int back_rc = orch::go_home(*arm_r, *arm_l);
    if (rc == 0)
        rc = back_rc;
    if (rc != 0)
        std::cerr << "[repeat] 结束 code=" << rc << "\n";
    else
        std::cout << "[repeat] 完成，双臂已在 home\n";
    return rc == 0 ? 0 : 1;
}
