#include "internal/state.h"
#include "t170/hardware/device_bus.h"
#include "t170/vision/debug.h"
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <thread>

namespace t170::app::detail {
std::optional<Json::Value> handle_status_commands(const Json::Value &request) {
    const std::string cmd=request.get("cmd","").asString();
if (cmd == "dispatch_status")
        return dispatch_status_json();
if (cmd == "snapshot")
    {
        Json::Value out = reply(true);
        out["right_tcp_m_deg"] = pose_json(arm_get_tcp_pos(*g_arm_r));
        out["left_tcp_m_deg"] = pose_json(arm_get_tcp_pos(*g_arm_l));
        out["grasped_right"] = orch::live_gripper_holding(true);
        out["grasped_left"] = orch::live_gripper_holding(false);
        out["latch_holding_right"] = g_session.last_grasped_r;
        out["latch_holding_left"] = g_session.last_grasped_l;
        out["grasped_count"] = g_session.last_grasp_count;
        if (g_grip)
        {
            out["gripper_q_right"] = g_grip->feedback(gripper::Side::Right).position_rad;
            out["gripper_q_left"] = g_grip->feedback(gripper::Side::Left).position_rad;
            out["gripper_hold_thresh"] = g_move_cfg.gripper_check.empty_q_rad;
        }
        out["chassis_ok"] = g_session.last_chassis_ok;
        out["chassis_station"] = g_session.last_chassis_station;
        out["chassis_message"] = g_session.last_chassis_message;
        out["belt_ok"] = g_session.last_belt_ok;
        out["belt_message"] = g_session.last_belt_message;
        out["tray_message"] = g_session.last_tray_message;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_y"] = (*g_session.layer3)(1);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        if (g_session.waist != nullptr)
        {
            try
            {
                const Eigen::MatrixXd q = g_session.waist->getJointPos();
                Json::Value joints(Json::arrayValue);
                const int n = static_cast<int>(q.cols() > 1 ? q.cols() : q.rows());
                const int count = std::min(n, 5);
                for (int i = 0; i < count; ++i)
                {
                    const double rad = q.cols() > 1 ? q(0, i) : q(i, 0);
                    joints.append(rad * 180.0 / M_PI);
                }
                out["waist_joints_deg"] = joints;
            }
            catch (const std::exception &)
            {
            }
        }
        return out;
    }
    return std::nullopt;
}
}
