#include "internal/state.h"
#include "t170/hardware/device_bus.h"
#include "t170/vision/debug.h"
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <thread>

namespace t170::app::detail {
Json::Value reply(bool ok, const std::string &error)
{
    Json::Value out(Json::objectValue);
    out["ok"] = ok;
    if (!error.empty())
        out["error"] = error;
    return out;
}
Json::Value xyz_json(double x, double y, double z)
{
    Json::Value out(Json::arrayValue);
    out.append(x);
    out.append(y);
    out.append(z);
    return out;
}
Json::Value with_arm_arrivals(Json::Value out)
{
    const std::vector<ArmArrivalRecord> recs = arm_arrival_log_copy();
    if (recs.empty())
        return out;
    Json::Value arr(Json::arrayValue);
    for (const ArmArrivalRecord &rec : recs)
    {
        Json::Value item(Json::objectValue);
        item["stage"] = rec.stage;
        item["hand"] = rec.hand;
        item["goal_xyz"] = xyz_json(rec.goal_x, rec.goal_y, rec.goal_z);
        item["actual_xyz"] = xyz_json(rec.actual_x, rec.actual_y, rec.actual_z);
        item["err_xyz"] = xyz_json(rec.err_x, rec.err_y, rec.err_z);
        item["err_m"] = rec.err_m;
        item["settled"] = rec.settled;
        arr.append(item);
    }
    out["arm_arrivals"] = arr;
    return out;
}
Json::Value pose_json(const Eigen::Matrix<double, 1, 6> &p)
{
    constexpr double kRadToDeg = 180.0 / M_PI;
    Json::Value out(Json::arrayValue);
    out.append(p(0));
    out.append(p(1));
    out.append(p(2));
    out.append(p(3) * kRadToDeg);
    out.append(p(4) * kRadToDeg);
    out.append(p(5) * kRadToDeg);
    return out;
}
Json::Value markers_json()
{
    Json::Value out(Json::arrayValue);
    for (const auto &hit : g_session.last_aruco_hits)
    {
        Json::Value item(Json::objectValue);
        item["id"] = hit.id;
        item["side_mm"] = hit.side_m * 1000.0;
        item["reproj_px"] = hit.reproj_px;
        item["camera_m"]["x"] = hit.cam_x;
        item["camera_m"]["y"] = hit.cam_y;
        item["camera_m"]["z"] = hit.cam_z;
        if (hit.robot_ok)
        {
            item["robot_m_deg"]["x"] = hit.robot_x;
            item["robot_m_deg"]["y"] = hit.robot_y;
            item["robot_m_deg"]["z"] = hit.robot_z;
            item["robot_m_deg"]["rx"] = hit.robot_rx_deg;
            item["robot_m_deg"]["ry"] = hit.robot_ry_deg;
            item["robot_m_deg"]["rz"] = hit.robot_rz_deg;
        }
        out.append(item);
    }
    return out;
}
Json::Value json_finite_or_null(double v)
{
    if (!std::isfinite(v))
        return Json::Value(Json::nullValue);
    return Json::Value(v);
}
Json::Value tray_holes_json()
{
    Json::Value out(Json::arrayValue);
    for (const auto &h : g_session.last_tray_holes)
    {
        Json::Value item(Json::objectValue);
        item["id"] = h.id;
        item["row"] = h.row;
        item["col"] = h.col;
        item["x"] = h.x;
        item["y"] = h.y;
        item["tray_z"] = json_finite_or_null(h.tray_z);
        item["tray_z_level"] = json_finite_or_null(h.tray_z_level);
        item["top_z"] = json_finite_or_null(h.top_z);
        item["top_z_level"] = json_finite_or_null(h.top_z_level);
        item["height_on_tray"] = json_finite_or_null(h.height_on_tray);
        item["class_id"] = h.class_id;
        item["class_name"] = h.class_name;
        item["conf"] = h.conf;
        item["dxy"] = (h.dxy >= 0.0) ? Json::Value(h.dxy) : Json::Value(Json::nullValue);
        item["depth_pts"] = h.depth_pts;
        out.append(item);
    }
    return out;
}
}
