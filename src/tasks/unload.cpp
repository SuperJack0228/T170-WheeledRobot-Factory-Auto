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
int dispatch_unload(DispatchClient &client)
{
    const TaskPoints points = read_task_points();

    if (!task_points_ready(dispatch_msg::TASK_UNLOAD, points))
        return kDispatchPointsMissing;
    StationOverride stations(g_move_cfg.chassis);
    g_move_cfg.chassis.out_station = points.unload;
    g_move_cfg.chassis.tray2_station = points.well;
    std::cout << "[dispatch] 下料：" << points.unload
              << " 不申请，到站拍一次按抓取窗口抓，然后去 " << points.well
              << " 放置。放完再发 " << dispatch_msg::DONE
              << "。空抓后再拍：倒件上报 1301 并等待；立件再抓一次；没有空孔再报满盘\n";
    board_step("去传送带二");
    if (!belt_ready_to_pick(g_move_cfg.chassis.out_station, g_move_cfg.conveyor2, "下料"))
    {
        const int ready_rc = drive_belt_ready(
            g_move_cfg.chassis.out_station, g_move_cfg.conveyor2, "dispatch_unload");
        if (ready_rc != 0)
            return ready_rc;
    }
    g_hw->last_grasped_r = false;
    g_hw->last_grasped_l = false;
    const int pick_rc = dispatch_pick_pair(
        client, apply_pick_well, dispatch_msg::APPLY_PICK_WELL, dispatch_msg::PICK_WELL,
        g_move_cfg.conveyor2, g_move_cfg.chassis.out_station, "belt2_grasp",
        true, true, "下料左手抓取", "下料右手抓取",
        g_move_cfg.chassis.tray2_station, true);
    if (pick_rc < 0)
        return pick_rc;
    if (pick_rc == kDispatchNoPartStay)
        return pick_rc;
    const bool hold_r = g_hw->last_grasped_r;
    const bool hold_l = g_hw->last_grasped_l;
    if (hold_r || hold_l)
    {
        int place_rc = 0;
        for (int attempt = 0;; ++attempt)
        {
            place_rc = vision_place_tray2(hold_r, hold_l, false);
            if (place_rc != kVisionQrFail)
                break;
            std::cerr << "[dispatch] 下料放置 料盘二维码解算失败，上报 1201\n";
            if (!dispatch_report_ex(client, dispatch_gateway().exception_code(DispatchFault::QrUnrecognized), "二维码识别错误"))
                return kDispatchHoldStay;
            board_activity("调度已解决，再拍一次料盘");
        }
        if (place_rc < 0)
            return place_rc;
        if (g_unload_empty_known && g_unload_empty_right + g_unload_empty_left <= 0)
        {
            std::cout << "[dispatch] 下料盘已经没有空孔\n";
            board_tray2(0, 0);
            dispatch_send(
                client, dispatch_msg::WELL_DONE_MATERIAL_FULL, client.well_done_material_full());
            g_unload_empty_known = false;
        }
    }
    else
        std::cout << "[dispatch] 这次没有抓到成品，不去 AP9 放\n";
    const int done_rc = dispatch_report(client, DispatchNote::Mission, "下料");
    if (done_rc != 0)
        return done_rc;
    std::cout << "[dispatch] 下料完成，停在当前位置\n";
    return 0;
}
}
