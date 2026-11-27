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
#include "t170/motion/arm.h"
#include "t170/common/units.h"
namespace orch {
namespace {

}

int dispatch_pick_pair(
    DispatchClient &client, bool (*apply_pick)(DispatchClient &, int),
    const std::string &request_state, const std::string &grant_state,
    const ConveyorStationConfig &c, const std::string &station, const char *tag,
    bool do_left, bool do_right, const char *left_what, const char *right_what,
    const std::string &depart_after_right, bool skip_apply)
{
    if (do_left || do_right)
    {
        auto raw_apply = [&]() { return apply_pick(client, kDispatchApplyTimeoutMs); };
        if (skip_apply)
        {
            std::cout << "[dispatch] 本次抓取不申请，按码系抓取窗口直接抓\n";
            board_step("检测框抓取");
        }
        else
            std::cout << "[dispatch] 抓取只申请一次，按码系抓取窗口决定伸哪只手\n";
        if (!skip_apply)
        {
            board_step("申请抓取");
            if (!dispatch_apply(request_state, grant_state, right_what, raw_apply))
            {
                std::cout << "[dispatch] " << right_what
                          << " 申请超时，不抓，发送 " << dispatch_msg::PICK_UP_DONE
                          << " 后继续任务\n";
                return dispatch_report(client, DispatchNote::PickHand, right_what);
            }
        }
        bool reported = false;
        int rc = 0;
        int vision_retry = 1;
        int next_hand = -1;
        for (;;)
        {
            reported = false;
            rc = go_belt_grasp(
                true, c, station, tag, next_hand,
                [&](bool) {
                    reported = true;
                    return dispatch_report_now(client, DispatchNote::PickHand, right_what);
                },
                true, depart_after_right, {}, true);
            if (rc == kBeltNoPartVision || rc == kBeltFallen)
            {
                const bool fallen = rc == kBeltFallen;
                std::cout << "[dispatch] "
                          << (fallen ? "空抓后再拍是倒件\n"
                                     : "抓取窗口内没有料\n");
                const int choice = fallen
                                       ? dispatch_report_ex(client, "工件倒了")
                                       : dispatch_report_ex(client, "下料点没有料可以抓");
                if (choice != 0)
                    return choice;
                const bool right_held = g_hw != nullptr && g_hw->last_grasped_r;
                const bool left_held = g_hw != nullptr && g_hw->last_grasped_l;
                if (right_held && !left_held)
                {
                    next_hand = 0;
                    std::cout << "[dispatch] 选择重试，右手保持夹持不动，只让左手再抓\n";
                }
                else if (left_held && !right_held)
                {
                    next_hand = 1;
                    std::cout << "[dispatch] 选择重试，左手保持夹持不动，只让右手再抓\n";
                }
                else
                {
                    next_hand = -1;
                    if (rc == kBeltFallen)
                        std::cout << "[dispatch] 选择重试，再拍照，按窗口决定伸哪只手\n";
                }
                continue;
            }
            if (rc == kBeltGripperNotReady)
            {
                std::cout << "[dispatch] 夹爪未到准备开度\n";
                const int choice = dispatch_report_ex(client, "夹爪未到准备开度");
                if (choice != 0)
                    return choice;
                std::cout << "[dispatch] 选择重试，重新下发准备开度并再抓\n";
                continue;
            }
            if (rc == kVisionQrFail)
            {
                const int again = dispatch_belt_vision_again(client, vision_retry, right_what);
                if (again != 0)
                    return again;
                continue;
            }
            break;
        }
        if (rc == kBeltGraspEmpty)
        {
            std::cout << "[dispatch] 抓空，不重试，继续任务\n";
            if (!reported)
            {
                const int note_rc = dispatch_report(client, DispatchNote::PickHand, right_what);
                if (note_rc != 0)
                    return note_rc;
            }
            return 0;
        }
        if (rc != 0)
            return rc;
        if (!reported)
            return dispatch_report(client, DispatchNote::PickHand, right_what);
        return 0;
    }
    return 0;
}
}
