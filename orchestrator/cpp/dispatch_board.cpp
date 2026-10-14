#include "dispatch_board.h"

#include "seg_pose_bridge.h"

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>

namespace orch
{
namespace
{

constexpr size_t kMaxEvents = 60;
constexpr size_t kKeepLogs = 30;

std::mutex g_mu;
DispatchBoard g_board;

std::string now_hms()
{
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[16];
    std::strftime(buf, sizeof(buf), "%H:%M:%S", &tm);
    return buf;
}

std::string now_stamp()
{
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
    return buf;
}

struct Console
{
    std::mutex mu;
    bool on = false;
    std::atomic<bool> stopping{false};
    int saved_out = -1;
    int saved_err = -1;
    int rd_out = -1;
    int rd_err = -1;
    std::thread th_out;
    std::thread th_err;
    std::ofstream file;
};

Console g_con;

void write_fd(int fd, const std::string &s)
{
    const char *p = s.data();
    size_t left = s.size();
    while (left > 0)
    {
        const ssize_t n = ::write(fd, p, left);
        if (n < 0)
        {
            if (errno == EINTR)
                continue;
            return;
        }
        p += n;
        left -= static_cast<size_t>(n);
    }
}

void handle_raw_line(const std::string &line, bool is_err)
{
    std::lock_guard<std::mutex> lk(g_con.mu);
    const std::string t = now_hms();
    if (g_con.file)
    {
        g_con.file << t << (is_err ? " E " : "   ") << line << '\n';
        g_con.file.flush();
    }
    if (is_err && g_con.saved_err >= 0 && !line.empty())
        write_fd(g_con.saved_err, t + (line.find("警告") == std::string::npos ? "  警告  " : "  ") +
                                      line + "\n");
}

void pump(int rd, bool is_err)
{
    std::string buf;
    char tmp[4096];
    while (true)
    {
        pollfd p{rd, POLLIN, 0};
        const int pr = ::poll(&p, 1, 200);
        if (pr < 0 && errno == EINTR)
            continue;
        if (pr <= 0)
        {
            if (g_con.stopping.load())
                break;
            continue;
        }
        const ssize_t n = ::read(rd, tmp, sizeof(tmp));
        if (n < 0 && errno == EINTR)
            continue;
        if (n <= 0)
            break;
        buf.append(tmp, static_cast<size_t>(n));
        size_t pos = 0;
        while (true)
        {
            const size_t nl = buf.find('\n', pos);
            if (nl == std::string::npos)
                break;
            std::string line = buf.substr(pos, nl - pos);
            if (!line.empty() && line.back() == '\r')
                line.pop_back();
            handle_raw_line(line, is_err);
            pos = nl + 1;
        }
        buf.erase(0, pos);
    }
    if (!buf.empty())
        handle_raw_line(buf, is_err);
}

void prune_logs(const std::filesystem::path &dir)
{
    std::error_code ec;
    std::vector<std::filesystem::path> logs;
    for (const auto &e : std::filesystem::directory_iterator(dir, ec))
    {
        const std::string name = e.path().filename().string();
        if (name.rfind("dispatch_", 0) == 0 && e.path().extension() == ".log")
            logs.push_back(e.path());
    }
    if (logs.size() <= kKeepLogs)
        return;
    std::sort(logs.begin(), logs.end());
    for (size_t i = 0; i + kKeepLogs < logs.size(); ++i)
        std::filesystem::remove(logs[i], ec);
}

std::string console_start()
{
    if (const char *v = std::getenv("T170_DISPATCH_VERBOSE"))
    {
        if (v[0] != '\0' && std::strcmp(v, "0") != 0)
            return {};
    }
    const std::filesystem::path dir = std::filesystem::path(project_root_dir()) / "logs";
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    prune_logs(dir);
    const std::string path = (dir / ("dispatch_" + now_stamp() + ".log")).string();

    std::lock_guard<std::mutex> lk(g_con.mu);
    if (g_con.on)
        return {};
    g_con.file.open(path, std::ios::out | std::ios::app);
    if (!g_con.file)
        return {};

    int po[2] = {-1, -1};
    int pe[2] = {-1, -1};
    if (::pipe2(po, O_CLOEXEC) != 0)
        return {};
    if (::pipe2(pe, O_CLOEXEC) != 0)
    {
        ::close(po[0]);
        ::close(po[1]);
        return {};
    }
    std::cout.flush();
    std::cerr.flush();
    std::fflush(stdout);
    std::fflush(stderr);
    g_con.saved_out = ::fcntl(1, F_DUPFD_CLOEXEC, 3);
    g_con.saved_err = ::fcntl(2, F_DUPFD_CLOEXEC, 3);
    ::dup2(po[1], 1);
    ::dup2(pe[1], 2);
    ::close(po[1]);
    ::close(pe[1]);
    g_con.rd_out = po[0];
    g_con.rd_err = pe[0];
    g_con.stopping = false;
    g_con.th_out = std::thread(pump, g_con.rd_out, false);
    g_con.th_err = std::thread(pump, g_con.rd_err, true);
    g_con.on = true;
    return path;
}

void console_stop()
{
    {
        std::lock_guard<std::mutex> lk(g_con.mu);
        if (!g_con.on)
            return;
    }
    std::cout.flush();
    std::cerr.flush();
    std::fflush(stdout);
    std::fflush(stderr);
    ::dup2(g_con.saved_out, 1);
    ::dup2(g_con.saved_err, 2);
    g_con.stopping = true;
    if (g_con.th_out.joinable())
        g_con.th_out.join();
    if (g_con.th_err.joinable())
        g_con.th_err.join();
    std::lock_guard<std::mutex> lk(g_con.mu);
    ::close(g_con.rd_out);
    ::close(g_con.rd_err);
    ::close(g_con.saved_out);
    ::close(g_con.saved_err);
    g_con.rd_out = g_con.rd_err = g_con.saved_out = g_con.saved_err = -1;
    g_con.file.close();
    g_con.on = false;
}

/** 面板事件写终端和日志。调用时不能持有 g_mu。 */
void print_event(const std::string &line)
{
    {
        std::lock_guard<std::mutex> lk(g_con.mu);
        if (g_con.on)
        {
            if (g_con.file)
            {
                g_con.file << "== " << line << '\n';
                g_con.file.flush();
            }
            write_fd(g_con.saved_out, line + "\n");
            return;
        }
    }
    std::cout << "[调度面板] " << line << std::endl;
}

std::string step_tag_locked()
{
    if (g_board.mission.empty())
        return {};
    std::ostringstream os;
    os << '[' << dispatch_mission_zh(g_board.mission);
    if (g_board.step_index >= 0 && !g_board.steps.empty())
        os << ' ' << (g_board.step_index + 1) << '/' << g_board.steps.size();
    os << "] ";
    return os.str();
}

/** 持锁记录事件，返回要打印的整行。面板未启用时返回空。 */
std::string push_event_locked(const std::string &level, const std::string &text)
{
    if (!g_board.active)
        return {};
    DispatchEvent e{now_hms(), level, text};
    g_board.events.push_back(e);
    if (g_board.events.size() > kMaxEvents)
        g_board.events.erase(g_board.events.begin(),
                             g_board.events.begin() + (g_board.events.size() - kMaxEvents));
    g_board.updated = e.time;
    return e.time + "  " + step_tag_locked() + text;
}

void emit(const std::string &line)
{
    if (!line.empty())
        print_event(line);
}

std::vector<std::string> steps_for(const std::string &mission)
{
    if (mission == "TASK_LOAD")
        return {"去料盘一", "料盘一抓取", "去传送带一", "申请上料放置", "传送带一放置", "汇报完成"};
    if (mission == "TASK_TRANSFER")
        return {"去传送带一", "左手申请抓取", "右手申请抓取", "去传送带二并申请放置", "传送带二放置", "汇报完成"};
    if (mission == "TASK_UNLOAD")
        return {"去传送带二", "左手申请抓取", "右手申请抓取", "去料盘二", "料盘二放置", "汇报完成"};
    if (mission == "CHARGE")
        return {"准备去充电", "整机 home", "去等待点", "挥手下垂", "腰降抓取高",
                "失能电机", "去充电路上", "去充电点", "充电中", "汇报完成"};
    if (mission == "CHARGE_TEST")
        return {"整机 home", "去等待点", "挥手下垂", "腰降抓取高", "失能电机", "去充电路上",
                "去充电点", "等待吸合", "检测到充电"};
    return {};
}

} // namespace

std::string dispatch_mission_zh(const std::string &mission)
{
    if (mission == "TASK_LOAD")
        return "上料";
    if (mission == "TASK_TRANSFER")
        return "转运";
    if (mission == "TASK_UNLOAD")
        return "下料";
    if (mission == "CHARGE")
        return "充电";
    if (mission == "CHARGE_TEST")
        return "充电测试";
    return mission;
}

std::string dispatch_state_zh(const std::string &state)
{
    if (state == "STANDBY")
        return "待命";
    if (state == "WORKING")
        return "工作中";
    if (state == "GO_CHARGE")
        return "去充电";
    if (state == "commuting")
        return "去充电路上";
    if (state == "CHARGING")
        return "充电中";
    if (state == "ERR_MOTOR_LOST")
        return "电机掉线";
    if (state == "ERR_GRIPPER_LOST")
        return "夹爪掉线";
    if (state == "ERR_CAMERA")
        return "相机/二维码异常";
    if (state == "ERR_MOTOR_DISABLE")
        return "电机失能";
    if (state == "ERR_AGV_OBSTACLE")
        return "底盘遇障";
    return state;
}

DispatchBoard dispatch_board_snapshot()
{
    std::lock_guard<std::mutex> lk(g_mu);
    return g_board;
}

bool dispatch_board_active()
{
    std::lock_guard<std::mutex> lk(g_mu);
    return g_board.active;
}

void board_begin(const std::string &server, const std::string &robot)
{
    std::string line;
    {
        std::lock_guard<std::mutex> lk(g_mu);
        // 进入调度不继承调试指令留下的料盘计数，面板从空开始。
        g_board = DispatchBoard{};
        g_board.active = true;
        g_board.started_at = now_hms();
        g_board.server = server;
        g_board.robot = robot;
        g_board.link = "连接中";
        line = push_event_locked("info", "进入调度模式  调度 " + server + "  本机 " + robot);
    }
    emit(line);
}

void board_end(int rc)
{
    std::string line;
    {
        std::lock_guard<std::mutex> lk(g_mu);
        g_board.mission.clear();
        g_board.steps.clear();
        g_board.step_index = -1;
        g_board.link = "已退出";
        g_board.activity = rc == -4 ? "已停止" : "调度模式已结束";
        line = push_event_locked(rc == 0 || rc == -4 ? "info" : "error",
                                 "退出调度模式 code=" + std::to_string(rc));
        g_board.active = false;
    }
    emit(line);
}

void board_link(const std::string &text)
{
    std::string line;
    {
        std::lock_guard<std::mutex> lk(g_mu);
        if (g_board.link == text)
            return;
        g_board.link = text;
        line = push_event_locked(text.find("断") != std::string::npos ? "warn" : "info",
                                 "调度连接：" + text);
    }
    emit(line);
}

void board_state(const std::string &state, bool sent_ok)
{
    std::string line;
    {
        std::lock_guard<std::mutex> lk(g_mu);
        const bool same = g_board.robot_state == state && g_board.robot_state_ok == sent_ok;
        g_board.robot_state = state;
        g_board.robot_state_ok = sent_ok;
        g_board.robot_state_time = now_hms();
        if (same)
            return;
        const bool err = state.rfind("ERR_", 0) == 0;
        line = push_event_locked(
            !sent_ok || err ? "error" : "info",
            "本机状态 " + state + "（" + dispatch_state_zh(state) + "）" +
                (sent_ok ? "" : " 未记下"));
    }
    emit(line);
}

void board_command(const std::string &cmd)
{
    std::string line;
    {
        std::lock_guard<std::mutex> lk(g_mu);
        g_board.last_command = cmd;
        g_board.last_command_time = now_hms();
        line = push_event_locked("info", "收到指令 " + cmd + "（" + dispatch_mission_zh(cmd) + "）");
    }
    emit(line);
}

void board_clear_command()
{
    std::lock_guard<std::mutex> lk(g_mu);
    g_board.last_command.clear();
    g_board.last_command_time.clear();
    g_board.updated = now_hms();
}

void board_mission_begin(const std::string &mission)
{
    std::string line;
    {
        std::lock_guard<std::mutex> lk(g_mu);
        g_board.mission = mission;
        g_board.mission_since = now_hms();
        g_board.steps = steps_for(mission);
        g_board.step_index = -1;
        g_board.link = "执行任务";
        g_board.apply_state = "idle";
        g_board.apply_name.clear();
        g_board.apply_grant.clear();
        g_board.apply_what.clear();
        g_board.apply_time.clear();
        g_board.report_state = "idle";
        g_board.report_name.clear();
        g_board.report_what.clear();
        g_board.report_time.clear();
        g_board.tray1_task_taken = 0;
        g_board.tray2_task_placed = 0;
        g_board.last_error.clear();
        line = push_event_locked("info", "====== 开始任务 " + dispatch_mission_zh(mission) + " ======");
    }
    emit(line);
}

void board_mission_end(bool ok, const std::string &result)
{
    std::string line;
    {
        std::lock_guard<std::mutex> lk(g_mu);
        if (ok)
            ++g_board.tasks_done;
        else
            ++g_board.tasks_failed;
        const std::string name = dispatch_mission_zh(g_board.mission);
        g_board.last_result = name + "：" + result;
        g_board.last_result_ok = ok;
        line = push_event_locked(ok ? "ok" : "error", "====== " + name + " " + result + " ======");
        g_board.mission.clear();
        g_board.steps.clear();
        g_board.step_index = -1;
    }
    emit(line);
}

void board_step(const std::string &step)
{
    std::string line;
    {
        std::lock_guard<std::mutex> lk(g_mu);
        if (!g_board.active)
            return;
        const auto it = std::find(g_board.steps.begin(), g_board.steps.end(), step);
        if (it == g_board.steps.end())
            return;
        const int idx = static_cast<int>(it - g_board.steps.begin());
        if (idx == g_board.step_index)
            return;
        g_board.step_index = idx;
        line = push_event_locked("info", "步骤：" + step);
    }
    emit(line);
}

void board_activity(const std::string &text)
{
    std::string line;
    {
        std::lock_guard<std::mutex> lk(g_mu);
        if (!g_board.active || g_board.activity == text)
            return;
        g_board.activity = text;
        line = push_event_locked("info", text);
    }
    emit(line);
}

void board_apply_begin(const std::string &request, const std::string &grant, const std::string &what)
{
    std::string line;
    {
        std::lock_guard<std::mutex> lk(g_mu);
        g_board.apply_name = request;
        g_board.apply_grant = grant;
        g_board.apply_what = what;
        g_board.apply_state = "waiting";
        g_board.apply_time = now_hms();
        line = push_event_locked("wait", "申请 " + request + "（" + what + "）已发出，等待 " + grant);
    }
    emit(line);
}

void board_apply_end(bool granted)
{
    std::string line;
    {
        std::lock_guard<std::mutex> lk(g_mu);
        g_board.apply_state = granted ? "granted" : "denied";
        g_board.apply_time = now_hms();
        line = push_event_locked(
            granted ? "ok" : "warn",
            granted ? ("已同意 " + g_board.apply_grant + "（" + g_board.apply_what + "）")
                    : ("申请 " + g_board.apply_name + " 20s 未同意，原地待命"));
    }
    emit(line);
}

void board_report_pending(const std::string &name, const std::string &what)
{
    std::lock_guard<std::mutex> lk(g_mu);
    g_board.report_name = name;
    g_board.report_what = what;
    g_board.report_state = "pending";
    g_board.report_time = now_hms();
}

void board_report(const std::string &name, const std::string &what, bool ok)
{
    std::string line;
    {
        std::lock_guard<std::mutex> lk(g_mu);
        g_board.report_name = name;
        g_board.report_what = what;
        g_board.report_state = ok ? "sent" : "failed";
        g_board.report_time = now_hms();
        line = push_event_locked(ok ? "ok" : "error",
                                 "汇报 " + name + "（" + what + "）" + (ok ? "已发送" : "发送失败"));
    }
    emit(line);
}

void board_station(const std::string &at, const std::string &target)
{
    std::string line;
    {
        std::lock_guard<std::mutex> lk(g_mu);
        if (g_board.station == at && g_board.target_station == target)
            return;
        g_board.station = at;
        g_board.target_station = target;
        if (!target.empty())
            line = push_event_locked("info", "底盘前往 " + target);
        else if (!at.empty())
            line = push_event_locked("info", "底盘已到 " + at);
    }
    emit(line);
}

void board_tray1(int right, int left)
{
    std::string line;
    {
        std::lock_guard<std::mutex> lk(g_mu);
        const bool same = g_board.tray1_known && g_board.tray1_right == right && g_board.tray1_left == left;
        g_board.tray1_known = true;
        g_board.tray1_right = std::max(0, right);
        g_board.tray1_left = std::max(0, left);
        g_board.tray1_time = now_hms();
        if (right + left > 0)
            g_board.tray1_empty_reported = false;
        if (same)
            return;
        line = push_event_locked("info", "料盘一 剩余毛坯 右=" + std::to_string(g_board.tray1_right) +
                                             " 左=" + std::to_string(g_board.tray1_left));
    }
    emit(line);
}

void board_tray1_taken(int n)
{
    std::lock_guard<std::mutex> lk(g_mu);
    g_board.tray1_task_taken += n;
    g_board.tray1_session_taken += n;
}

void board_tray1_empty_reported()
{
    std::lock_guard<std::mutex> lk(g_mu);
    g_board.tray1_empty_reported = true;
}

void board_tray2(int right, int left)
{
    std::string line;
    {
        std::lock_guard<std::mutex> lk(g_mu);
        const bool same = g_board.tray2_known && g_board.tray2_right == right && g_board.tray2_left == left;
        g_board.tray2_known = true;
        g_board.tray2_right = std::max(0, right);
        g_board.tray2_left = std::max(0, left);
        g_board.tray2_time = now_hms();
        if (right + left > 0)
            g_board.tray2_full_reported = false;
        if (same)
            return;
        line = push_event_locked("info", "料盘二 剩余空孔 右=" + std::to_string(g_board.tray2_right) +
                                             " 左=" + std::to_string(g_board.tray2_left));
    }
    emit(line);
}

void board_tray2_placed(int n)
{
    std::lock_guard<std::mutex> lk(g_mu);
    g_board.tray2_task_placed += n;
    g_board.tray2_session_placed += n;
}

void board_tray2_full_reported()
{
    std::lock_guard<std::mutex> lk(g_mu);
    g_board.tray2_full_reported = true;
}

void board_error(const std::string &text)
{
    std::string line;
    {
        std::lock_guard<std::mutex> lk(g_mu);
        g_board.last_error = now_hms() + " " + text;
        line = push_event_locked("error", text);
    }
    emit(line);
}

void board_note(const std::string &level, const std::string &text)
{
    std::string line;
    {
        std::lock_guard<std::mutex> lk(g_mu);
        line = push_event_locked(level, text);
    }
    emit(line);
}

DispatchConsole::DispatchConsole()
{
    const std::string path = console_start();
    std::string line;
    {
        std::lock_guard<std::mutex> lk(g_mu);
        g_board.log_path = path;
        line = push_event_locked(
            "info", path.empty() ? std::string("终端不过滤（T170_DISPATCH_VERBOSE）")
                                 : ("完整日志 " + path + "，终端只显示调度事件和警告"));
    }
    emit(line);
}

DispatchConsole::~DispatchConsole()
{
    console_stop();
}

} // namespace orch
