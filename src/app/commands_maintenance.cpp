#include "internal/state.h"
#include "t170/app/diagnostic_gestures.h"
#include "t170/hardware/device_bus.h"
#include "t170/vision/debug.h"
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <thread>

namespace t170::app::detail {
std::optional<Json::Value> handle_maintenance_commands(const Json::Value &request) {
    const std::string cmd=request.get("cmd","").asString();
if (cmd == "home")
    {
        const int rc = orch::go_home(*g_arm_r, *g_arm_l);
        return rc == 0 ? reply(true) : reply(false, "归位失败/中止，code=" + std::to_string(rc));
    }
if (cmd == "bow")
    {
        const int waist_rc = orch::home_waist_upright();
        if (waist_rc != 0)
        {
            Json::Value out = reply(false, "鞠躬：腰升到 home 失败/中止，code=" + std::to_string(waist_rc));
            out["code"] = waist_rc;
            return out;
        }
        const int rc = arm_bow(*g_arm_r, *g_arm_l);
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("鞠躬失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        return out;
    }
if (cmd == "wave")
    {
        const int waist_rc = orch::home_waist_upright();
        if (waist_rc != 0)
        {
            Json::Value out = reply(false, "挥手：腰升到 home 失败/中止，code=" + std::to_string(waist_rc));
            out["code"] = waist_rc;
            return out;
        }
        const int rc = arm_wave_right(*g_arm_r, *g_arm_l);
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("挥手失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        return out;
    }
if (cmd == "welcome")
    {
        const int waist_rc = orch::home_waist_upright();
        if (waist_rc != 0)
        {
            Json::Value out = reply(false, "迎宾：腰升到 home 失败/中止，code=" + std::to_string(waist_rc));
            out["code"] = waist_rc;
            return out;
        }
        const int rc = arm_welcome(*g_arm_r, *g_arm_l);
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("迎宾失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        return out;
    }
if (cmd == "wave_ready")
    {
        const int waist_rc = orch::home_waist_upright();
        if (waist_rc != 0)
        {
            Json::Value out = reply(false, "挥手准备：腰回 home 失败/中止，code=" + std::to_string(waist_rc));
            out["code"] = waist_rc;
            return out;
        }
        const int rc = arm_wave_hang(*g_arm_r, *g_arm_l);
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("挥手准备失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        return out;
    }
if (cmd == "tray2ready" || cmd == "tray2ready1" || cmd == "tray2ready2" ||
        cmd == "tray2ready3" || cmd == "tray2ready6")
    {
        int group = 1;
        if (cmd == "tray2ready6")
            group = 6;
        else if (cmd.size() >= 11 && cmd.back() >= '1' && cmd.back() <= '3')
            group = cmd.back() - '0';
        else if (request.isMember("row_group") && request["row_group"].isInt())
            group = request["row_group"].asInt();
        const int rc = orch::go_tray2_ready(*g_arm_r, *g_arm_l, group);
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("料盘2准备姿态失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        out["row_group"] = group;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        return out;
    }
if (cmd == "grasp_ready" || cmd == "grasp_ready1" || cmd == "grasp_ready2" ||
        cmd == "grasp_ready3" || cmd == "grasp_ready6")
    {
        int group = 1;
        if (cmd == "grasp_ready6")
            group = 6;
        else if (cmd.size() >= 12 && cmd.back() >= '1' && cmd.back() <= '3')
            group = cmd.back() - '0';
        else if (request.isMember("row_group") && request["row_group"].isInt())
            group = request["row_group"].asInt();
        const int rc = orch::go_grasp_ready(*g_arm_r, *g_arm_l, group);
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("抓取准备姿态失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        out["row_group"] = group;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        return out;
    }
if (cmd == "waist_cycle")
    {
        const int rc = orch::go_waist_cycle();
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("腰循环测试失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        return out;
    }
if (cmd == "waist1" || cmd == "waist2")
    {
        const int pose = (cmd == "waist2") ? 2 : 1;
        const int rc = orch::go_waist_debug(pose);
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("腰平移失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        out["pose"] = pose;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        if (pose == 2 && !g_session.last_debug_photo.empty())
            out["photo"] = g_session.last_debug_photo;
        return out;
    }
if (cmd == "belt_ready")
    {
        const int rc = orch::go_belt_ready(*g_arm_r, *g_arm_l);
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("皮带准备姿态失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        return out;
    }
if (cmd == "belt_grasp_rpy")
    {
        const int rc = orch::go_belt_grasp_rpy(*g_arm_r, *g_arm_l);
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("皮带抓取姿态失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        return out;
    }
if (cmd == "belt_place")
    {
        const int rc = orch::go_belt_place(*g_arm_r, *g_arm_l);
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("皮带放置姿态失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        return out;
    }
if (cmd == "belt2_grasp_rpy")
    {
        const int rc = orch::go_belt2_grasp_rpy(*g_arm_r, *g_arm_l);
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("第二传送带抓取姿态失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        return out;
    }
if (cmd == "belt2_ready")
    {
        const int rc = orch::go_belt2_ready(*g_arm_r, *g_arm_l);
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("第二传送带准备失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        out["station"] = g_move_cfg.chassis.out_station;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        return out;
    }
if (cmd == "waist_jog")
    {
        const int joint = request.get("joint", 2).asInt();
        const double dq = request.get("dq_rad", 0.05).asDouble();
        const int rc = orch::go_waist_jog(joint, dq);
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("腰关节点动失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        out["joint"] = joint;
        out["dq_rad"] = dq;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        return out;
    }
    return std::nullopt;
}
}
