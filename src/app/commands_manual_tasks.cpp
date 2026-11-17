#include "internal/state.h"
#include "t170/hardware/device_bus.h"
#include "t170/vision/debug.h"
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <thread>

namespace t170::app::detail {
std::optional<Json::Value> handle_manual_tasks_commands(const Json::Value &request) {
    const std::string cmd=request.get("cmd","").asString();
if (cmd == "belt")
    {
        const int rc = orch::go_belt();
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("皮带工位失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        out["station"] = g_move_cfg.chassis.belt_station;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        out["head_yaw_deg"] = g_move_cfg.conveyor.head_yaw_deg;
        out["head_pitch_deg"] = g_move_cfg.conveyor.head_pitch_deg;
        out["head_roll_deg"] = g_move_cfg.conveyor.head_roll_deg;
        out["offset_right_x"] = g_move_cfg.conveyor.offset_right.x;
        out["offset_right_y"] = g_move_cfg.conveyor.offset_right.y;
        out["offset_right_z"] = g_move_cfg.conveyor.offset_right.z;
        out["offset_left_x"] = g_move_cfg.conveyor.offset_left.x;
        out["offset_left_y"] = g_move_cfg.conveyor.offset_left.y;
        out["offset_left_z"] = g_move_cfg.conveyor.offset_left.z;
        out["belt_ok"] = g_session.last_belt_ok;
        out["belt_name"] = g_session.last_belt_name;
        out["belt_message"] = g_session.last_belt_message;
        if (std::isfinite(g_session.last_belt_x))
        {
            out["belt_x"] = g_session.last_belt_x;
            out["belt_y"] = g_session.last_belt_y;
            out["belt_z"] = g_session.last_belt_z;
        }
        if (!g_session.last_debug_photo.empty())
            out["photo"] = g_session.last_debug_photo;
        return out;
    }
if (cmd == "belt2" || cmd == "belt2_place")
    {
        const int rc = orch::go_belt2();
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("第二传送带放置失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        out["station"] = g_move_cfg.chassis.out_station;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        out["offset_right_x"] = g_move_cfg.conveyor2.offset_right.x;
        out["offset_right_y"] = g_move_cfg.conveyor2.offset_right.y;
        out["offset_right_z"] = g_move_cfg.conveyor2.offset_right.z;
        out["offset_left_x"] = g_move_cfg.conveyor2.offset_left.x;
        out["offset_left_y"] = g_move_cfg.conveyor2.offset_left.y;
        out["offset_left_z"] = g_move_cfg.conveyor2.offset_left.z;
        out["belt_ok"] = g_session.last_belt_ok;
        out["belt_name"] = g_session.last_belt_name;
        out["belt_message"] = g_session.last_belt_message;
        if (std::isfinite(g_session.last_belt_x))
        {
            out["belt_x"] = g_session.last_belt_x;
            out["belt_y"] = g_session.last_belt_y;
            out["belt_z"] = g_session.last_belt_z;
        }
        if (!g_session.last_debug_photo.empty())
            out["photo"] = g_session.last_debug_photo;
        return out;
    }
if (cmd == "belt2_grasp")
    {
        const int rc = orch::go_belt2_grasp();
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("第二传送带抓取失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        out["station"] = g_move_cfg.chassis.out_station;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        out["grasp_offset_left_x"] = g_move_cfg.conveyor2.grasp_offset_left.x;
        out["grasp_offset_left_y"] = g_move_cfg.conveyor2.grasp_offset_left.y;
        out["grasp_offset_left_z"] = g_move_cfg.conveyor2.grasp_offset_left.z;
        out["grasp_offset_right_x"] = g_move_cfg.conveyor2.grasp_offset_right.x;
        out["grasp_offset_right_y"] = g_move_cfg.conveyor2.grasp_offset_right.y;
        out["grasp_offset_right_z"] = g_move_cfg.conveyor2.grasp_offset_right.z;
        out["grasp_hover_above_m"] = g_move_cfg.conveyor2.grasp_hover_above_m;
        if (std::isfinite(g_session.last_belt_x))
        {
            out["grasp_x"] = g_session.last_belt_x;
            out["grasp_y"] = g_session.last_belt_y;
            out["grasp_z"] = g_session.last_belt_z;
        }
        out["belt_ok"] = g_session.last_belt_ok;
        out["belt_name"] = g_session.last_belt_name;
        out["belt_message"] = g_session.last_belt_message;
        out["grasped_right"] = g_session.last_grasped_r;
        out["grasped_left"] = g_session.last_grasped_l;
        if (!g_session.last_debug_photo.empty())
            out["photo"] = g_session.last_debug_photo;
        return out;
    }
if (cmd == "belt_grasp")
    {
        const int rc = orch::go_belt_grasp();
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("皮带抓取失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        out["grasp_offset_left_x"] = g_move_cfg.conveyor.grasp_offset_left.x;
        out["grasp_offset_left_y"] = g_move_cfg.conveyor.grasp_offset_left.y;
        out["grasp_offset_left_z"] = g_move_cfg.conveyor.grasp_offset_left.z;
        out["grasp_offset_right_x"] = g_move_cfg.conveyor.grasp_offset_right.x;
        out["grasp_offset_right_y"] = g_move_cfg.conveyor.grasp_offset_right.y;
        out["grasp_offset_right_z"] = g_move_cfg.conveyor.grasp_offset_right.z;
        out["grasp_hover_above_m"] = g_move_cfg.conveyor.grasp_hover_above_m;
        if (std::isfinite(g_session.last_belt_x))
        {
            out["grasp_x"] = g_session.last_belt_x;
            out["grasp_y"] = g_session.last_belt_y;
            out["grasp_z"] = g_session.last_belt_z;
        }
        out["belt_ok"] = g_session.last_belt_ok;
        out["belt_name"] = g_session.last_belt_name;
        out["belt_message"] = g_session.last_belt_message;
        out["grasped_right"] = g_session.last_grasped_r;
        out["grasped_left"] = g_session.last_grasped_l;
        if (std::isfinite(g_session.last_belt_x))
        {
            out["belt_x"] = g_session.last_belt_x;
            out["belt_y"] = g_session.last_belt_y;
            out["belt_z"] = g_session.last_belt_z;
        }
        if (!g_session.last_debug_photo.empty())
            out["photo"] = g_session.last_debug_photo;
        return out;
    }

if (cmd == "tray2_auto")
    {
        const bool right=orch::live_gripper_holding(true),left=orch::live_gripper_holding(false);
        if(!right && !left) return reply(false,"没有已识别的持件侧，先完成抓取后再执行 tray2_auto");
        const int rc=orch::vision_place_tray2(right,left,false);
        Json::Value out=reply(rc==0,rc==0 ? std::string() : "料盘2放置未全部完成，code="+std::to_string(rc));
        out["code"]=rc;out["station"]=g_move_cfg.chassis.tray2_station;
        out["placed_right"]=g_session.last_placed_r;out["placed_left"]=g_session.last_placed_l;
        out["placed_count"]=g_session.last_grasp_count;
        return out;
    }
if (cmd == "tray2" || cmd == "tray2_place")
    {
        const int rc = orch::vision_place_tray2();
        Json::Value out = reply(rc >= 0, rc < 0 ? "料盘2精度测试失败/中止" : "");
        out["code"] = rc;
        out["station"] = g_move_cfg.chassis.tray2_station;
        out["placed_right"] = g_session.last_grasped_r;
        out["placed_left"] = g_session.last_grasped_l;
        out["placed_count"] = g_session.last_grasp_count;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        return out;
    }


if (cmd == "vision_grasp_once" || cmd == "vision_servo_dual_once")
    {
        const int rc = cmd == "vision_servo_dual_once" ? orch::vision_servo_dual_once() : orch::vision_grasp_once();
        Json::Value out = reply(rc >= 0, rc < 0 ? "抓一次料盘失败/中止" : "");
        out["code"] = rc;
        out["grasped_right"] = g_session.last_grasped_r;
        out["grasped_left"] = g_session.last_grasped_l;
        out["grasped_count"] = g_session.last_grasp_count;
        return out;
    }


if (cmd == "vision_grasp")
    {
        const int rc = orch::vision_grasp_existing_pipeline(false);
        Json::Value out = reply(rc >= 0, rc < 0 ? "视觉抓取失败/中止" : "");
        out["code"] = rc;
        out["grasped_right"] = g_session.last_grasped_r;
        out["grasped_left"] = g_session.last_grasped_l;
        out["grasped_count"] = g_session.last_grasp_count;
        return out;
    }



if (cmd == "charge")
    {
        const int rc = orch::run_charge_test();
        Json::Value out = reply(
            rc == 0,
            rc == 0 ? std::string()
                    : (rc == -4 ? "充电测试已中止"
                                : ("充电测试失败/中止，code=" + std::to_string(rc))));
        out["code"] = rc;
        out["chassis_ok"] = g_session.last_chassis_ok;
        out["chassis_station"] = g_session.last_chassis_station;
        out["chassis_message"] = g_session.last_chassis_message;
        return out;
    }
    return std::nullopt;
}
}
