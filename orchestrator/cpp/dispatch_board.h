#pragma once

/** 调度模式的状态面板。网页 dispatch_status 读它，终端只打印它产生的事件。 */

#include <string>
#include <vector>

namespace orch
{

struct DispatchEvent
{
    std::string time;
    /** info / ok / wait / warn / error */
    std::string level;
    std::string text;
};

struct DispatchBoard
{
    bool active = false;
    std::string started_at;
    std::string server;
    std::string robot;
    std::string log_path;

    /** 和调度系统的连接：等待任务 / 执行任务 / 断线重试 */
    std::string link;
    /** 本机流程状态，STANDBY / WORKING / ERR_*。1.8 调度库不接收这些名字。 */
    std::string robot_state;
    bool robot_state_ok = true;
    std::string robot_state_time;

    std::string last_command;
    std::string last_command_time;

    std::string mission;
    std::string mission_since;
    std::vector<std::string> steps;
    int step_index = -1;
    std::string activity;

    std::string apply_name;
    std::string apply_grant;
    std::string apply_what;
    /** idle / waiting / granted / denied */
    std::string apply_state = "idle";
    std::string apply_time;

    std::string report_name;
    std::string report_what;
    /** idle / pending / sent / failed */
    std::string report_state = "idle";
    std::string report_time;

    std::string station;
    std::string target_station;

    bool tray1_known = false;
    int tray1_right = 0;
    int tray1_left = 0;
    int tray1_task_taken = 0;
    int tray1_session_taken = 0;
    bool tray1_empty_reported = false;
    std::string tray1_time;

    bool tray2_known = false;
    int tray2_right = 0;
    int tray2_left = 0;
    int tray2_task_placed = 0;
    int tray2_session_placed = 0;
    bool tray2_full_reported = false;
    std::string tray2_time;

    int tasks_done = 0;
    int tasks_failed = 0;
    std::string last_result;
    bool last_result_ok = true;
    std::string last_error;

    std::vector<DispatchEvent> events;
    std::string updated;
};

DispatchBoard dispatch_board_snapshot();
bool dispatch_board_active();

void board_begin(const std::string &server, const std::string &robot);
void board_end(int rc);
void board_link(const std::string &text);
void board_state(const std::string &state, bool sent_ok);
void board_command(const std::string &cmd);
/** 进入待命且尚无新任务时清掉「收到的指令」，避免仍显示上一次上料等。 */
void board_clear_command();
void board_mission_begin(const std::string &mission);
void board_mission_end(bool ok, const std::string &result);
void board_step(const std::string &step);
void board_activity(const std::string &text);
void board_apply_begin(const std::string &request, const std::string &grant, const std::string &what);
void board_apply_end(bool granted);
void board_report_pending(const std::string &name, const std::string &what);
void board_report(const std::string &name, const std::string &what, bool ok);
void board_station(const std::string &at, const std::string &target);
void board_tray1(int right, int left);
void board_tray1_taken(int n);
void board_tray1_empty_reported();
void board_tray2(int right, int left);
void board_tray2_placed(int n);
void board_tray2_full_reported();
void board_error(const std::string &text);
void board_note(const std::string &level, const std::string &text);

/** 中文名，未知的原样返回。 */
std::string dispatch_mission_zh(const std::string &mission);
std::string dispatch_state_zh(const std::string &state);

/** 调度期间：原始输出全部进 logs/dispatch_*.log，终端只显示面板事件和 stderr。
 *  设环境变量 T170_DISPATCH_VERBOSE=1 则不过滤。 */
struct DispatchConsole
{
    DispatchConsole();
    ~DispatchConsole();
    DispatchConsole(const DispatchConsole &) = delete;
    DispatchConsole &operator=(const DispatchConsole &) = delete;
};

} // namespace orch
