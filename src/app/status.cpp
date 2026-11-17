#include "internal/state.h"
#include "t170/hardware/device_bus.h"
#include "t170/vision/debug.h"
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <thread>

namespace t170::app::detail {
Json::Value dispatch_status_json()
{
    const orch::DispatchBoard b = orch::dispatch_board_snapshot();
    Json::Value out = reply(true);
    out["active"] = b.active;
    out["started_at"] = b.started_at;
    out["server"] = b.server;
    out["robot"] = b.robot;
    out["log_path"] = b.log_path;
    out["link"] = b.link;
    out["robot_state"] = b.robot_state;
    out["robot_state_zh"] = orch::dispatch_state_zh(b.robot_state);
    out["robot_state_ok"] = b.robot_state_ok;
    out["robot_state_time"] = b.robot_state_time;
    out["last_command"] = b.last_command;
    out["last_command_zh"] = orch::dispatch_mission_zh(b.last_command);
    out["last_command_time"] = b.last_command_time;
    out["mission"] = b.mission;
    out["mission_zh"] = orch::dispatch_mission_zh(b.mission);
    out["mission_since"] = b.mission_since;
    Json::Value steps(Json::arrayValue);
    for (const auto &s : b.steps)
        steps.append(s);
    out["steps"] = steps;
    out["step_index"] = b.step_index;
    out["activity"] = b.activity;

    Json::Value apply;
    apply["name"] = b.apply_name;
    apply["grant"] = b.apply_grant;
    apply["what"] = b.apply_what;
    apply["state"] = b.apply_state;
    apply["time"] = b.apply_time;
    out["apply"] = apply;

    Json::Value report;
    report["name"] = b.report_name;
    report["what"] = b.report_what;
    report["state"] = b.report_state;
    report["time"] = b.report_time;
    out["report"] = report;

    out["station"] = b.station;
    out["target_station"] = b.target_station;
    // 界面持件：实时夹爪 q；闩锁仅作流程用，附带上报便于对照。
    out["holding_right"] = orch::live_gripper_holding(true);
    out["holding_left"] = orch::live_gripper_holding(false);
    out["latch_holding_right"] = g_session.last_grasped_r;
    out["latch_holding_left"] = g_session.last_grasped_l;
    if (g_grip)
    {
        const auto fr = g_grip->feedback(gripper::Side::Right);
        const auto fl = g_grip->feedback(gripper::Side::Left);
        out["gripper_q_right"] = fr.position_rad;
        out["gripper_q_left"] = fl.position_rad;
        out["gripper_hold_thresh"] = g_move_cfg.gripper_check.empty_q_rad;
    }

    Json::Value t1;
    t1["known"] = b.tray1_known;
    t1["right"] = b.tray1_right;
    t1["left"] = b.tray1_left;
    t1["task"] = b.tray1_task_taken;
    t1["session"] = b.tray1_session_taken;
    t1["reported"] = b.tray1_empty_reported;
    t1["time"] = b.tray1_time;
    out["tray1"] = t1;

    Json::Value t2;
    t2["known"] = b.tray2_known;
    t2["right"] = b.tray2_right;
    t2["left"] = b.tray2_left;
    t2["task"] = b.tray2_task_placed;
    t2["session"] = b.tray2_session_placed;
    t2["reported"] = b.tray2_full_reported;
    t2["time"] = b.tray2_time;
    out["tray2"] = t2;

    out["tasks_done"] = b.tasks_done;
    out["tasks_failed"] = b.tasks_failed;
    out["last_result"] = b.last_result;
    out["last_result_ok"] = b.last_result_ok;
    out["last_error"] = b.last_error;

    Json::Value events(Json::arrayValue);
    for (const auto &e : b.events)
    {
        Json::Value ev;
        ev["time"] = e.time;
        ev["level"] = e.level;
        ev["text"] = e.text;
        events.append(ev);
    }
    out["events"] = events;
    out["updated"] = b.updated;
    return out;
}
}
