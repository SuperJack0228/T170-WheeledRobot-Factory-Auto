#include "t170/common/paths.h"
#include "t170/common/logging.h"
#include "t170/telemetry/factory_status.h"
#include "t170/telemetry/dispatch_metrics.h"

#include "t170/common/abort_query.h"

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

#include "internal/console.h"
namespace orch {
namespace {
constexpr size_t kMaxEvents = 60;
}

namespace {
std::mutex g_mu;
}

namespace {
DispatchBoard g_board;
}

std::string now_hms()
{
    return t170::logging::local_time("%H:%M:%S");
}

namespace {
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
}

namespace {
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
}

namespace {
void emit(const std::string &line)
{
    if (!line.empty())
        print_event(line);
}
}

namespace {
std::vector<std::string> steps_for(const std::string &mission)
{
    if (mission == "TASK_LOAD")
        return {"去料盘一", "料盘一抓取", "去传送带一", "申请上料放置", "传送带一放置", "汇报完成"};
    if (mission == "TASK_TRANSFER")
        return {"去传送带一", "检测框抓取", "去传送带二并申请放置", "传送带二放置", "汇报完成"};
    if (mission == "TASK_UNLOAD")
        return {"去传送带二", "申请抓取", "去料盘二", "料盘二放置", "汇报完成"};
    if (mission == "CHARGE")
        return {"准备去充电", "整机 home", "去等待点", "挥手下垂", "腰降抓取高",
                "失能电机", "去充电路上", "去充电点", "充电中", "汇报完成"};
    if (mission == "CHARGE_TEST")
        return {"整机 home", "去等待点", "挥手下垂", "腰降抓取高", "失能电机", "去充电路上",
                "去充电点", "等待吸合", "检测到充电"};
    return {};
}
}

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

void board_begin(const std::string &server, const std::string &robot)
{
    timing_run_begin(robot,server);
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
    timing_run_end(rc);
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
    timing_mission_begin(mission);
}
void board_mission_end(bool ok, const std::string &result)
{
    timing_mission_end(ok, result);
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
    bool advanced = false;
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
        advanced = true;
        line = push_event_locked("info", "步骤：" + step);
    }
    if (advanced)
        timing_step(step);
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
                    : ("申请 " + g_board.apply_name + " 20s 没有回复"));
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
    timing_event("dispatch_report",name,ok?"sent":"failed",what);
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
    timing_event("part_count","tray1_taken","reported","",{{"count",double(n)}});
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
    timing_event("part_count","tray2_placed","reported","",{{"count",double(n)}});
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
    timing_event("fault","board_error","reported",text);
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
}
