#include "internal/state.h"
#include "t170/hardware/device_bus.h"
#include "t170/vision/debug.h"
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <thread>

namespace t170::app::detail {
std::optional<Json::Value> handle_inspection_commands(const Json::Value &request) {
    const std::string cmd=request.get("cmd","").asString();


if (cmd == "aruco_detect")
    {
        const int rc = orch::vision_aruco_detect_pipeline();
        Json::Value out = reply(rc >= 0, rc < 0 ? "头相机 ArUco 检测失败/中止" : "");
        out["code"] = rc;
        out["count"] = g_session.last_aruco_head;
        out["markers"] = markers_json();
        return out;
    }
if (cmd == "detect_tray_holes")
    {
        const int rc = orch::vision_tray_holes_pipeline();
        Json::Value out = reply(
            rc == 0,
            rc == 0 ? std::string()
                    : (rc < 0 ? std::string("料盘孔位检测失败/中止") : g_session.last_tray_message));
        out["code"] = rc;
        out["ok_tray"] = g_session.last_tray_ok;
        out["message"] = g_session.last_tray_message;
        out["save_path"] = g_session.last_tray_save_path;
        out["reproj_px"] = g_session.last_tray_reproj_px;
        out["tilt_deg"] = json_finite_or_null(g_session.last_tray_tilt_deg);
        Json::Value ids(Json::arrayValue);
        for (int id : g_session.last_tray_used_ids)
            ids.append(id);
        out["used_ids"] = ids;
        out["holes"] = tray_holes_json();
        if (rc > 0 && g_session.last_tray_message.size())
            out["error"] = g_session.last_tray_message;
        return out;
    }
    return std::nullopt;
}
}
