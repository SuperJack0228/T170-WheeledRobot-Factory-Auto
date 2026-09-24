#include "Ti5_socketcan.h"

#include "Ti5_Device_SDK.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <exception>
#include <iostream>
#include <mutex>
#include <net/if.h>
#include <span>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace
{
thread_local bool t_can_io_watch = false;
std::atomic<bool> g_can_io_fault{false};
std::atomic<bool> g_hw_abort{false};
std::atomic<int> g_can_send_fail_streak{0};
constexpr int kCanSendFailStreakAbort = 3;
std::mutex g_can_io_msg_mu;
std::string g_can_io_msg;
bool g_device_ready = false;

constexpr double kCntPerRad = 4.0 * 65536.0 / (2.0 * M_PI);
/** 旧协议速度限位 2500 所用换算（与腰 vel2hz 一致）。 */
constexpr double kOldSpeedToRad = (2.0 * M_PI) / (4.0 * 1000.0);

void can_io_set_msg(const std::string &msg)
{
    std::lock_guard<std::mutex> lock(g_can_io_msg_mu);
    g_can_io_msg = msg;
}

void note_can_send_ok()
{
    if (!t_can_io_watch)
        return;
    g_can_send_fail_streak.store(0);
}

void note_can_send_fail(uint32_t can_id)
{
    if (!t_can_io_watch)
        return;
    const int n = g_can_send_fail_streak.fetch_add(1) + 1;
    if (n < kCanSendFailStreakAbort)
        return;
    std::ostringstream oss;
    oss << "电机发送失败 id=" << can_id << " 连续" << n << "次";
    can_io_set_msg(oss.str());
    g_can_io_fault.store(true);
}

void note_can_recv_fail(uint32_t can_id)
{
    if (!t_can_io_watch)
        return;
    std::ostringstream oss;
    oss << "电机读取失败 id=" << can_id;
    can_io_set_msg(oss.str());
    g_can_io_fault.store(true);
}

bool is_right_arm_can_id(uint32_t can_id)
{
    return can_id >= 16 && can_id <= 22;
}

bool skip_id(uint32_t can_id)
{
    return right_arm_motors_locked() && is_right_arm_can_id(can_id);
}

void zero_motor_data7(int *data)
{
    if (data == nullptr)
        return;
    for (int i = 0; i < 7; ++i)
        data[i] = 0;
}

bool skip_locked_right_arm(int a, int *data, bool zero_data)
{
    if (a != 1 || !right_arm_motors_locked())
        return false;
    if (zero_data)
        zero_motor_data7(data);
    return true;
}

double cnt_to_rad(int cnt)
{
    return static_cast<double>(cnt) / kCntPerRad;
}

int rad_to_cnt(double rad)
{
    return static_cast<int>(std::lround(rad * kCntPerRad));
}

double old_speed_to_rad_s(int raw)
{
    return static_cast<double>(raw) * kOldSpeedToRad;
}

int rad_s_to_old_speed(double rad_s)
{
    return static_cast<int>(std::lround(rad_s / kOldSpeedToRad));
}

const uint32_t *arm_ids(int a)
{
    return a == 1 ? kRightArmCanIds : kLeftArmCanIds;
}

template <typename Fn>
bool sdk_try(uint32_t can_id, const char *what, Fn &&fn)
{
    if (skip_id(can_id))
        return true;
    try
    {
        fn();
        note_can_send_ok();
        return true;
    }
    catch (const std::exception &ex)
    {
        std::cerr << "[motor] " << what << " id=" << can_id << " 失败: " << ex.what() << std::endl;
        note_can_send_fail(can_id);
        return false;
    }
}

bool motor_get_pos_rad(uint32_t id, double &rad)
{
    if (skip_id(id))
    {
        rad = 0.0;
        return true;
    }
    try
    {
        rad = get_Position(id);
        note_can_send_ok();
        return true;
    }
    catch (const std::exception &ex)
    {
        std::cerr << "[motor] get_Position id=" << id << " 失败: " << ex.what() << std::endl;
        note_can_recv_fail(id);
        rad = 0.0;
        return false;
    }
}

bool motor_hold_current(uint32_t id)
{
    double q = 0.0;
    if (!motor_get_pos_rad(id, q))
        return false;
    return motor_set_pos_rad(id, q);
}

bool can_iface_exists(const std::string &ifname)
{
    return if_nametoindex(ifname.c_str()) != 0;
}

bool can_iface_is_admin_up(const std::string &ifname)
{
    const int fd = ::socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd < 0)
        return false;
    struct ifreq ifr {};
    std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifname.c_str());
    const int rc = ioctl(fd, SIOCGIFFLAGS, &ifr);
    ::close(fd);
    return rc == 0 && (ifr.ifr_flags & IFF_UP);
}

int run_ip_link(const std::string &args)
{
    std::string cmd = "/usr/bin/timeout -s KILL 8 ";
    if (geteuid() != 0)
        cmd += "sudo -n ";
    cmd += "ip ";
    cmd += args;
    std::cout << "[can] " << cmd << std::endl;
    std::cout.flush();
    const int ret = system(cmd.c_str());
    if (ret != 0)
        std::cerr << "[can] 失败 code=" << ret << "  cmd=" << cmd << std::endl;
    return ret;
}

int count_up_can_ifaces()
{
    int n = 0;
    for (int i = 0; i < 32; ++i)
    {
        const std::string name = "can" + std::to_string(i);
        if (can_iface_exists(name) && can_iface_is_admin_up(name))
            ++n;
    }
    return n;
}

int log_paired_motors()
{
    std::cout << "[device-sdk] 已配对电机:";
    int n = 0;
    for (uint32_t id = 1; id <= 32; ++id)
    {
        if (motor_paired(id))
        {
            std::cout << ' ' << id;
            ++n;
        }
    }
    if (n == 0)
        std::cout << " (无)";
    std::cout << std::endl;
    return n;
}

int log_paired_grippers()
{
    std::cout << "[device-sdk] 已配对夹爪:";
    int n = 0;
    for (std::uint8_t id = 0; id < 4; ++id)
    {
        if (gripper_paired(id))
        {
            std::cout << ' ' << static_cast<int>(id);
            ++n;
        }
    }
    if (n == 0)
        std::cout << " (无)";
    std::cout << std::endl;
    return n;
}

std::vector<uint32_t> required_motor_ids()
{
    std::vector<uint32_t> ids;
    ids.insert(ids.end(), std::begin(kWaistAllCanIds), std::end(kWaistAllCanIds));
    ids.insert(ids.end(), std::begin(kLeftArmCanIds), std::end(kLeftArmCanIds));
    if (!right_arm_motors_locked())
        ids.insert(ids.end(), std::begin(kRightArmCanIds), std::end(kRightArmCanIds));
    ids.insert(ids.end(), std::begin(kHeadCanIds), std::end(kHeadCanIds));
    return ids;
}

std::vector<std::uint8_t> required_gripper_ids()
{
    std::vector<std::uint8_t> ids;
    if (!right_arm_motors_locked())
        ids.push_back(0);
    ids.push_back(1);
    return ids;
}

void format_missing(const std::vector<uint32_t> &motors,
                    const std::vector<std::uint8_t> &grippers,
                    std::string &out)
{
    std::ostringstream oss;
    if (!motors.empty())
    {
        oss << "电机";
        for (size_t i = 0; i < motors.size(); ++i)
        {
            if (i)
                oss << ',';
            oss << motors[i];
        }
    }
    if (!grippers.empty())
    {
        if (!motors.empty())
            oss << "；";
        oss << "夹爪";
        for (size_t i = 0; i < grippers.size(); ++i)
        {
            if (i)
                oss << ',';
            oss << static_cast<int>(grippers[i]);
            oss << (grippers[i] == 0 ? "(右)" : grippers[i] == 1 ? "(左)" : "");
        }
    }
    out = oss.str();
}

int count_paired_required_grippers(std::vector<std::uint8_t> &miss_g)
{
    miss_g.clear();
    int n = 0;
    for (std::uint8_t id : required_gripper_ids())
    {
        if (gripper_paired(id))
            ++n;
        else
            miss_g.push_back(id);
    }
    return n;
}

bool collect_missing(std::vector<uint32_t> &miss_m, std::vector<std::uint8_t> &miss_g)
{
    miss_m.clear();
    for (uint32_t id : required_motor_ids())
    {
        if (!motor_paired(id))
            miss_m.push_back(id);
    }
    const int n_g = count_paired_required_grippers(miss_g);
    // 电机必须齐全；夹爪允许少一个，零个才算缺。
    return miss_m.empty() && n_g >= 1;
}

std::string g_device_scan_err;

constexpr int kDeviceScanAttempts = 2;
constexpr int kDeviceScanGapMs = 400;

bool scan_until_all_devices()
{
    g_device_scan_err.clear();
    std::vector<uint32_t> miss_m;
    std::vector<std::uint8_t> miss_g;
    for (int attempt = 1; attempt <= kDeviceScanAttempts; ++attempt)
    {
        log_paired_motors();
        log_paired_grippers();
        if (collect_missing(miss_m, miss_g))
        {
            std::cout << "[device-sdk] 第 " << attempt << "/" << kDeviceScanAttempts
                      << " 次扫描：电机齐全";
            if (!miss_g.empty())
            {
                std::string miss;
                format_missing({}, miss_g, miss);
                std::cout << "，" << miss << "未扫到（允许少一个夹爪）";
            }
            std::cout << "，允许启动\n";
            return true;
        }
        std::string miss;
        format_missing(miss_m, miss_g, miss);
        std::cerr << "[device-sdk] 第 " << attempt << "/" << kDeviceScanAttempts
                  << " 次扫描未齐：" << miss << "\n";
        if (attempt == kDeviceScanAttempts)
            break;
        std::cerr << "[device-sdk] close 后重新 pair()，不改 CAN 链路\n";
        try
        {
            ::close();
        }
        catch (const std::exception &ex)
        {
            std::cerr << "[device-sdk] close 失败: " << ex.what() << "\n";
        }
        usleep(static_cast<useconds_t>(kDeviceScanGapMs) * 1000);
        try
        {
            ::pair();
        }
        catch (const std::exception &ex)
        {
            std::cerr << "[device-sdk] pair 失败: " << ex.what() << "\n";
        }
    }
    std::string miss;
    format_missing(miss_m, miss_g, miss);
    g_device_scan_err = "设备未扫齐，拒绝启动（" + miss +
                        "）。电机必须齐全；夹爪允许少一个。";
    return false;
}

} // namespace

std::string device_scan_error()
{
    return g_device_scan_err.empty() ? "设备扫描失败" : g_device_scan_err;
}

bool motor_enable_id(uint32_t id)
{
    return sdk_try(id, "Enable", [id]() {
        Clear_Error(id);
        Motor_Enable(id);
    });
}

bool motor_set_pos_rad(uint32_t id, double rad)
{
    return sdk_try(id, "set_Position", [id, rad]() { set_Position(id, rad); });
}

bool can_io_faulted()
{
    return g_can_io_fault.load();
}

std::string can_io_fault_message()
{
    std::lock_guard<std::mutex> lock(g_can_io_msg_mu);
    return g_can_io_msg.empty() ? "电机收发失败" : g_can_io_msg;
}

void can_io_fault_clear()
{
    g_can_io_fault.store(false);
    g_can_send_fail_streak.store(0);
    std::lock_guard<std::mutex> lock(g_can_io_msg_mu);
    g_can_io_msg.clear();
}

void can_io_watch_begin()
{
    t_can_io_watch = true;
}

void can_io_watch_end()
{
    t_can_io_watch = false;
}

bool hardware_abort_requested()
{
    return g_hw_abort.load();
}

void hardware_abort_request()
{
    g_hw_abort.store(true);
}

void hardware_abort_clear()
{
    g_hw_abort.store(false);
}

void hardware_abort_sleep_ms(int ms)
{
    if (ms <= 0)
        return;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (g_hw_abort.load())
            return;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

std::array<int, CAN_CHANNEL_COUNT> can_sockets = {-1, -1, -1, -1, -1, -1};

constexpr bool kLockRightArmMotors = false;

bool right_arm_motors_locked()
{
    return kLockRightArmMotors;
}

int left_h_id = -1, left_m_id = -1, left_l_id = -1, right_h_id = -1, right_m_id = -1, right_l_id = -1;
int head_id = -1;
int waist_id = -1;

bool set_can_bitrate(const std::string &ifname, uint32_t bitrate)
{
    if (ifname.empty() ||
        ifname.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789") !=
            std::string::npos)
    {
        std::cerr << "[can] 非法接口名\n";
        return false;
    }
    if (!can_iface_exists(ifname))
        return false;
    if (can_iface_is_admin_up(ifname))
    {
        std::cout << "[can] " << ifname << " 已 UP，跳过（禁止 down 再 up type can）\n";
        return true;
    }
    // KCAN-USB + PREEMPT_RT：`ip link set canN down && up type can bitrate`
    // 会在驱动 ndo_stop/ndo_open 里把内核卡死，SSH/ping/Ctrl+C 全部无响应。
    // 口已 down 时只分两步：先设波特率，再单独 up。
    if (run_ip_link("link set " + ifname + " type can bitrate " + std::to_string(bitrate)) != 0)
        return false;
    if (run_ip_link("link set " + ifname + " up") != 0)
        return false;
    return true;
}

bool init_can_channel(int, const CAN_CONFIG &)
{
    return g_device_ready;
}

bool init_socketcan()
{
    // 首次沿用加载时自动 pair。缺设备才 close/pair 重扫，禁止动 ip link。
    std::cout << "[device-sdk] 启动扫描最多 2 次：腰1-5、左臂23-29、"
              << (right_arm_motors_locked() ? "" : "右臂16-22、")
              << "头30-32 必须齐全；夹爪允许少一个\n";
    const int can_up = count_up_can_ifaces();
    std::cout << "[can] 当前 UP 的 can 口: " << can_up << "\n";
    if (!scan_until_all_devices())
    {
        std::cerr << "[device-sdk] " << g_device_scan_err << "\n";
        g_device_ready = false;
        return false;
    }
    g_device_ready = true;
    return true;
}

bool init_socketcan_can2()
{
    return init_socketcan();
}

void close_can_channels()
{
    try
    {
        ::close();
    }
    catch (const std::exception &ex)
    {
        std::cerr << "[device-sdk] close 失败: " << ex.what() << std::endl;
    }
    g_device_ready = false;
    waist_id = head_id = -1;
    left_h_id = left_m_id = left_l_id = -1;
    right_h_id = right_m_id = right_l_id = -1;
}

void close_can_channels2()
{
    close_can_channels();
}

bool send_can_frame(int, uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    if (skip_id(can_id) || data == nullptr || dlc < 1)
        return true;
    const uint8_t cmd = data[0];
    if (cmd == 1)
        return sdk_try(can_id, "Enable", [can_id]() { Motor_Enable(can_id); });
    if (cmd == 2)
        return sdk_try(can_id, "Disable", [can_id]() { Motor_Disable(can_id); });
    if (cmd == 11)
        return sdk_try(can_id, "Clear_Error", [can_id]() { Clear_Error(can_id); });
    std::cerr << "[motor] 未映射的单字节命令 " << static_cast<int>(cmd)
              << " id=" << can_id << std::endl;
    return false;
}

bool receive_can_frame_timeout(int, uint8_t *)
{
    return false;
}

bool can_channel_ready(int)
{
    return g_device_ready;
}

bool can_motion_ready()
{
    return g_device_ready && motor_paired(23);
}

void toIntArray(int number, uint8_t *res, int size)
{
    unsigned int unsignedNumber = static_cast<unsigned int>(number);
    for (int i = 0; i < size; ++i)
    {
        res[i] = unsignedNumber & 0xFF;
        unsignedNumber >>= 8;
    }
}

int socketcan_sendcommand(int, int cannum, uint32_t *can_idlist, uint8_t command, int *data)
{
    if (can_idlist == nullptr || data == nullptr || cannum <= 0)
        return -1;
    int flag = 1;
    for (int i = 0; i < cannum; ++i)
    {
        const uint32_t id = can_idlist[i];
        if (skip_id(id))
            continue;
        bool ok = false;
        if (command == 30)
            ok = motor_set_pos_rad(id, cnt_to_rad(data[i]));
        else if (command == 36)
            ok = sdk_try(id, "set_Max_Speed", [id, v = old_speed_to_rad_s(data[i])]() {
                set_Max_Speed(id, v);
            });
        else if (command == 37)
            ok = sdk_try(id, "set_Min_Speed", [id, v = old_speed_to_rad_s(data[i])]() {
                set_Min_Speed(id, v);
            });
        else if (command == 34)
            ok = sdk_try(id, "set_Max_ACC", [id, v = old_speed_to_rad_s(data[i])]() {
                set_Max_ACC(id, v);
            });
        else if (command == 35)
            ok = sdk_try(id, "set_Min_ACC", [id, v = old_speed_to_rad_s(data[i])]() {
                set_Min_ACC(id, v);
            });
        else if (command == 28)
            ok = sdk_try(id, "set_Current", [id, a = static_cast<double>(data[i])]() {
                set_Current(id, a);
            });
        else
        {
            std::cerr << "[motor] 未映射写命令 " << static_cast<int>(command)
                      << " id=" << id << std::endl;
            ok = false;
        }
        if (!ok)
            flag = -1;
    }
    return flag;
}

int socketcan_sendsimplecommand(int, int cannum, uint32_t *can_idlist, uint8_t command, int *data)
{
    if (can_idlist == nullptr || cannum <= 0)
        return -1;
    int flag = 1;
    for (int i = 0; i < cannum; ++i)
    {
        const uint32_t id = can_idlist[i];
        if (skip_id(id))
        {
            if (data)
                data[i] = 0;
            continue;
        }
        bool ok = true;
        if (command == 1)
            ok = sdk_try(id, "Enable", [id]() { Motor_Enable(id); });
        else if (command == 2)
            ok = sdk_try(id, "Disable", [id]() { Motor_Disable(id); });
        else if (command == 11)
            ok = sdk_try(id, "Clear_Error", [id]() { Clear_Error(id); });
        else if (command == 8)
        {
            double q = 0.0;
            ok = motor_get_pos_rad(id, q);
            if (data)
                data[i] = rad_to_cnt(q);
        }
        else if (command == 6)
        {
            try
            {
                const double v = get_Speed(id);
                if (data)
                    data[i] = rad_to_cnt(v);
                note_can_send_ok();
            }
            catch (const std::exception &ex)
            {
                std::cerr << "[motor] get_Speed id=" << id << " 失败: " << ex.what() << std::endl;
                note_can_recv_fail(id);
                if (data)
                    data[i] = 0;
                ok = false;
            }
        }
        else if (command == 24)
        {
            try
            {
                if (data)
                    data[i] = rad_s_to_old_speed(get_Max_Speed(id));
                note_can_send_ok();
            }
            catch (...)
            {
                ok = false;
            }
        }
        else if (command == 25)
        {
            try
            {
                if (data)
                    data[i] = rad_s_to_old_speed(get_Min_Speed(id));
                note_can_send_ok();
            }
            catch (...)
            {
                ok = false;
            }
        }
        else
        {
            std::cerr << "[motor] 未映射读/简命令 " << static_cast<int>(command)
                      << " id=" << id << std::endl;
            ok = false;
        }
        if (!ok)
            flag = -1;
    }
    return flag;
}

bool select_bind(int &hand_id, int id, std::string name)
{
    if (skip_id(static_cast<uint32_t>(id)))
    {
        hand_id = -1;
        std::cout << "[device-sdk] " << name << " id=" << id << " 已锁定，跳过\n";
        return false;
    }
    if (!motor_paired(static_cast<uint32_t>(id)))
    {
        hand_id = -1;
        std::cerr << "[device-sdk] " << name << " 电机 " << id << " 未配对\n";
        return false;
    }
    hand_id = 0;
    std::cout << "[device-sdk] " << name << " 电机 " << id << " 已配对\n";
    return true;
}

void hand_socketid_bind()
{
    select_bind(left_h_id, 23, "left_h_id");
    select_bind(left_m_id, 26, "left_m_id");
    select_bind(left_l_id, 28, "left_l_id");
    if (right_arm_motors_locked())
    {
        right_h_id = -1;
        right_m_id = -1;
        right_l_id = -1;
        std::cout << "[can] 右臂已锁定：不使能/不通讯 CAN 16-22\n";
    }
    else
    {
        select_bind(right_h_id, 16, "right_h_id");
        select_bind(right_m_id, 19, "right_m_id");
        select_bind(right_l_id, 21, "right_l_id");
        std::cout << "[can] 右臂已解锁，CAN 16-22 正常使能/通讯\n";
    }
    if (select_bind(waist_id, static_cast<int>(kWaistPitchCanIds[2]), "waist_id"))
        waist_id = 0;
    if (select_bind(head_id, static_cast<int>(kHeadCanIds[1]), "head_id"))
        head_id = 0;
}

void get_motor_position(int *data, int a)
{
    if (skip_locked_right_arm(a, data, true))
        return;
    uint32_t ids[7];
    std::memcpy(ids, arm_ids(a), sizeof(ids));
    socketcan_sendsimplecommand(0, 7, ids, 8, data);
}

void get_motor_running_speed(int *data, int a)
{
    if (skip_locked_right_arm(a, data, true))
        return;
    uint32_t ids[7];
    std::memcpy(ids, arm_ids(a), sizeof(ids));
    socketcan_sendsimplecommand(0, 7, ids, 6, data);
}

void set_motor_position(int *data, int a)
{
    if (skip_locked_right_arm(a, data, false))
        return;
    uint32_t ids[7];
    std::memcpy(ids, arm_ids(a), sizeof(ids));
    socketcan_sendcommand(0, 7, ids, 30, data);
}

void get_motor_speed(int *data, int a)
{
    if (skip_locked_right_arm(a, data, true))
        return;
    const uint8_t cmd = (data && data[0] > 0) ? 24 : 25;
    uint32_t ids[7];
    std::memcpy(ids, arm_ids(a), sizeof(ids));
    socketcan_sendsimplecommand(0, 7, ids, cmd, data);
}

void set_motor_speed(int *data, int a)
{
    if (skip_locked_right_arm(a, data, false))
        return;
    const uint8_t cmd = (data && data[0] > 0) ? 36 : 37;
    uint32_t ids[7];
    std::memcpy(ids, arm_ids(a), sizeof(ids));
    socketcan_sendcommand(0, 7, ids, cmd, data);
}

void set_motor_addspeed(int *data, int a)
{
    if (skip_locked_right_arm(a, data, false))
        return;
    const uint8_t cmd = (data && data[0] > 0) ? 34 : 35;
    uint32_t ids[7];
    std::memcpy(ids, arm_ids(a), sizeof(ids));
    socketcan_sendcommand(0, 7, ids, cmd, data);
}

void get_motor_addspeed(int *data, int a)
{
    if (skip_locked_right_arm(a, data, true))
        return;
    if (data)
        zero_motor_data7(data);
}

void set_motor(int *data, uint8_t command, int a)
{
    if (skip_locked_right_arm(a, data, false))
        return;
    uint32_t ids[7];
    std::memcpy(ids, arm_ids(a), sizeof(ids));
    socketcan_sendcommand(0, 7, ids, command, data);
}

void set_motor_singn_mode(int *data, uint8_t command, int a)
{
    uint32_t ids[7];
    std::memcpy(ids, arm_ids(a), sizeof(ids));
    socketcan_sendsimplecommand(0, 7, ids, command, data);
}

void set_motor_current_zero()
{
    int data[7] = {0, 0, 0, 0, 0, 0, 0};
    set_motor(data, 28, 0);
    set_motor(data, 28, 1);
}

void set_waist_motor_position(int *data)
{
    uint32_t ids[5];
    std::memcpy(ids, kWaistKinCanIds, sizeof(ids));
    socketcan_sendcommand(0, 5, ids, 30, data);
}

void get_motor_waist_position(int *data)
{
    uint32_t ids[5];
    std::memcpy(ids, kWaistKinCanIds, sizeof(ids));
    socketcan_sendsimplecommand(0, 5, ids, 8, data);
}

void enable_waist_motors()
{
    std::cout << "[waist] 使能 4踝/3膝/2髋/5侧倾/1回转（LowerBody 五轴位置模式）\n";
    for (uint32_t id : kWaistAllCanIds)
        motor_enable_id(id);
    int spd[5] = {2500, 2500, 2500, 2500, 2500};
    int nspd[5] = {-2500, -2500, -2500, -2500, -2500};
    uint32_t ids[5];
    std::memcpy(ids, kWaistAllCanIds, sizeof(ids));
    socketcan_sendcommand(0, 5, ids, 36, spd);
    socketcan_sendcommand(0, 5, ids, 37, nspd);
    can_io_fault_clear();
}

namespace
{

double quintic_scalar(double t, double total_time, double start, double target)
{
    if (total_time <= 1e-6)
        return target;
    const double ratio = std::clamp(t / total_time, 0.0, 1.0);
    const double r2 = ratio * ratio;
    const double r3 = r2 * ratio;
    const double r4 = r3 * ratio;
    const double r5 = r4 * ratio;
    const double pos_term = 10.0 * r3 - 15.0 * r4 + 6.0 * r5;
    return start + (target - start) * pos_term;
}

} // namespace

int smooth_motor_move_deg(
    int,
    int motor_count,
    uint32_t *can_idlist,
    const double *target_angles_deg,
    double speed_deg_per_s,
    int dt_ms)
{
    if (motor_count <= 0 || can_idlist == nullptr || target_angles_deg == nullptr)
        return -1;
    if (speed_deg_per_s <= 1e-6)
        speed_deg_per_s = 10.0;
    if (dt_ms <= 0)
        dt_ms = 20;

    std::vector<int> current_cnt(static_cast<size_t>(motor_count), 0);
    if (socketcan_sendsimplecommand(0, motor_count, can_idlist, 8, current_cnt.data()) != 1)
        return -1;

    std::vector<double> start_deg(static_cast<size_t>(motor_count));
    std::vector<double> target_deg(static_cast<size_t>(motor_count));
    std::vector<int> pos_data(static_cast<size_t>(motor_count));

    double max_delta_deg = 0.0;
    for (int i = 0; i < motor_count; ++i)
    {
        start_deg[static_cast<size_t>(i)] = motor_cnt_to_angle_deg(current_cnt[static_cast<size_t>(i)]);
        target_deg[static_cast<size_t>(i)] = target_angles_deg[i];
        max_delta_deg = std::max(
            max_delta_deg,
            std::abs(target_deg[static_cast<size_t>(i)] - start_deg[static_cast<size_t>(i)]));
    }

    auto send_pos = [&]() {
        socketcan_sendcommand(0, motor_count, can_idlist, 30, pos_data.data());
    };

    if (max_delta_deg < 0.01)
    {
        for (int i = 0; i < motor_count; ++i)
            pos_data[static_cast<size_t>(i)] = motor_angle_deg_to_cnt(target_deg[static_cast<size_t>(i)]);
        send_pos();
        return 0;
    }

    const double total_time_s = max_delta_deg / speed_deg_per_s;
    const double dt_s = static_cast<double>(dt_ms) / 1000.0;
    double t = 0.0;
    while (t < total_time_s)
    {
        for (int i = 0; i < motor_count; ++i)
        {
            const double angle_deg = quintic_scalar(
                t,
                total_time_s,
                start_deg[static_cast<size_t>(i)],
                target_deg[static_cast<size_t>(i)]);
            pos_data[static_cast<size_t>(i)] = motor_angle_deg_to_cnt(angle_deg);
        }
        send_pos();
        if (hardware_abort_requested() || can_io_faulted())
            return -1;
        usleep(static_cast<useconds_t>(dt_ms * 1000));
        t += dt_s;
    }

    for (int i = 0; i < motor_count; ++i)
        pos_data[static_cast<size_t>(i)] = motor_angle_deg_to_cnt(target_deg[static_cast<size_t>(i)]);
    send_pos();
    return 0;
}

void motor_speed_change()
{
    int data[7] = {};
    int speed_data[7] = {1, 1, 1, 1, 1, 1, 1};

    get_motor_speed(speed_data, 1);
    std::cout << "[motor] right max_speed";
    for (int i = 0; i < 7; ++i)
        std::cout << ' ' << speed_data[i];
    std::cout << std::endl;

    get_motor_speed(speed_data, 0);
    std::cout << "[motor] left max_speed";
    for (int i = 0; i < 7; ++i)
        std::cout << ' ' << speed_data[i];
    std::cout << std::endl;

    int speed[7] = {2500, 2500, 2500, 2500, 2500, 2500, 2500};
    int speed2[7] = {-2500, -2500, -2500, -2500, -2500, -2500, -2500};
    set_motor_speed(speed, 1);
    set_motor_speed(speed2, 1);
    set_motor_speed(speed, 0);
    set_motor_speed(speed2, 0);
}

void rev_motor_error(int a)
{
    if (skip_locked_right_arm(a, nullptr, false))
        return;
    int data[7] = {};
    uint32_t ids[7];
    std::memcpy(ids, arm_ids(a), sizeof(ids));
    socketcan_sendsimplecommand(0, 7, ids, 11, data);
    socketcan_sendsimplecommand(0, 7, ids, 1, data);
}
