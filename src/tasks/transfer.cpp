#include "internal/runtime.h"
#include "t170/telemetry/dispatch_metrics.h"
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace orch {
int dispatch_transfer(DispatchClient &client)
{
    const TaskPoints points = read_task_points();

    if (!task_points_ready(dispatch_msg::TASK_TRANSFER, points))
        return kDispatchPointsMissing;
    StationOverride stations(g_move_cfg.chassis);
    g_move_cfg.chassis.belt_station = points.load;
    g_move_cfg.chassis.out_station = points.unload;
    std::cout << "[dispatch] 转运：" << points.load
              << " 不申请，按码系抓取窗口抓；窗口内没料报 1102，倒件报 1301；抓完发 "
              << dispatch_msg::PICK_UP_DONE
              << "。有件才去 " << points.unload << " 放\n";
    board_step("去传送带一");
    if (!belt_ready_to_pick(g_move_cfg.chassis.belt_station, g_move_cfg.conveyor, "转运"))
    {
        const int ready_rc = drive_belt_ready(
            g_move_cfg.chassis.belt_station, g_move_cfg.conveyor, "dispatch_transfer");
        if (ready_rc != 0)
            return ready_rc;
    }
    g_hw->last_grasped_r = false;
    g_hw->last_grasped_l = false;
    ConveyorStationConfig transfer_grasp = g_move_cfg.conveyor;
    const int pick_rc = dispatch_pick_pair(
        client, apply_pick_half, dispatch_msg::APPLY_PICK_HALF, dispatch_msg::PICK_HALF,
        transfer_grasp, g_move_cfg.chassis.belt_station, "belt_grasp",
        true, true, "转运左手抓取", "转运右手抓取",
        g_move_cfg.chassis.out_station, true);
    if (pick_rc < 0)
        return pick_rc;
    if (pick_rc == kDispatchNoPartStay || pick_rc == kDispatchGripperStay)
        return pick_rc;
    const bool hold_r = g_hw->last_grasped_r;
    const bool hold_l = g_hw->last_grasped_l;
    ConveyorStationConfig transfer_place = g_move_cfg.conveyor2;
    const int place_rc = place_while_driving(
        client, [](DispatchClient &c, int timeout_ms) { return c.apply_place_half_done(timeout_ms); },
        dispatch_msg::APPLY_PLACE_HALF, dispatch_msg::PLACE,
        g_move_cfg.chassis.out_station, transfer_place, "belt2",
        hold_r, hold_l, "转运放置");
    if (place_rc != 0)
        return place_rc;
    const int done_rc = dispatch_report(client, DispatchNote::Mission, "转运");
    if (done_rc != 0)
        return done_rc;
    std::cout << "[dispatch] 转运完成，停在当前位置\n";
    return 0;
}
}
