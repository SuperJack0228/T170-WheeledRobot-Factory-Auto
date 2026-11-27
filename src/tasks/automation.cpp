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
static int run_dispatch_mode_body()
{
    if (g_hw == nullptr || g_hw->arm_r == nullptr || g_hw->arm_l == nullptr)
        return -1;
    const auto &ch = g_move_cfg.chassis;
    if (ch.robot_type != "T170C2.0" && ch.robot_type != "T230C")
    {
        std::cerr << "[dispatch] robot_type 只能是 T170C2.0 或 T230C，当前 "
                  << ch.robot_type << "\n";
        return -1;
    }
    if (ch.sequence_number.empty())
    {
        std::cerr << "[dispatch] sequence_number 为空\n";
        return -1;
    }
    std::cout << "[dispatch] 系统对接模式，调度 "
              << dispatch_msg::kDefaultHost << ":" << dispatch_msg::kDefaultPort
              << " 型号 " << ch.robot_type << " 编号 " << ch.sequence_number
              << "。底盘点位由系统下发。无任务时去调度 standby_point，还没有则去 "
              << dispatch_idle_station() << "\n";
    // 先记当前地图下的坐标，再上报地图名。库连上后才知道要不要换图。
    double map_x = 0;
    double map_y = 0;
    double map_angle = 0;
    if (!read_pose_before_dispatch(map_x, map_y, map_angle))
        return -1;
    if (!stage_dispatch_map_name())
        return -1;
    std::cout << "[dispatch] 正在连接调度并核对地图\n";
    board_note("info", "正在连接调度并核对地图");
    // client 必须活过 inbox 线程；退出时先拆连接，apply_mission 才会返回，否则 join 卡死、
    // board_end 迟迟不跑，网页一直显示「调度中」。
    auto client_holder = dispatch_gateway().connect(ch.robot_type, ch.sequence_number);
    DispatchClient &client = *client_holder;
    const int map_rc = finish_dispatch_map(client, map_x, map_y, map_angle);
    if (map_rc != 0)
    {
        // No task may run against an unprepared map. SKIP here cancels entry.
        if (map_rc == kDispatchTaskSkip || map_rc == kDispatchOffline)
        {
            board_activity("取消本次进入调度，地图尚未就绪");
            return 0;
        }
        if (map_rc == kDispatchReportFailed)
        {
            board_activity("地图异常上报失败，原地停下");
            while (!hardware_abort_requested())
                hardware_abort_sleep_ms(500);
            return -4;
        }
        return map_rc;
    }
    struct BodyMotorRestore
    {
        bool enabled = true;
        ~BodyMotorRestore()
        {
            if (enabled && !hardware_abort_requested() && g_body_disabled_for_charge)
                set_body_motors(true);
        }
    } body_restore;
    DispatchInbox box;
    struct WakeInboxBeforeJoin
    {
        std::unique_ptr<DispatchClient> &holder;
        DispatchInbox &box;
        ~WakeInboxBeforeJoin()
        {
            // 先关闭 SDK 连接解除 apply_mission 阻塞，再 join 收信线程，最后释放适配器。
            if (holder)
            {
                std::cout << "[dispatch] 退出调度：断开调度连接，解除等待任务\n";
            }
            close_dispatch_inbox(holder,box);
        }
    } wake_inbox{client_holder, box};
    std::string mission;
    std::string wait_station = dispatch_idle_station();
    auto take_ready = [&]() -> bool {
        if (!inbox_poll(box, mission))
            return false;
        inbox_join(box);
        return true;
    };
    // 到待命点后：途中缓存的任务立刻取出执行；没有则清掉界面上的旧「收到指令」。
    auto take_cached_or_standby = [&]() {
        g_dispatch_stay_at_tray2 = false;
        if (g_hw != nullptr && !g_hw->last_chassis_station.empty())
            wait_station = g_hw->last_chassis_station;
        dispatch_note_state(dispatch_msg::STANDBY);
        std::string cached;
        if (inbox_poll(box, cached))
        {
            inbox_join(box);
            if (!cached.empty())
            {
                mission = std::move(cached);
                std::cout << "[dispatch] 到 " << wait_station << " 发现缓存任务 " << mission
                          << "，立刻执行\n";
                board_activity("到 " + wait_station + "，执行缓存任务");
                return;
            }
            // apply_mission 空返回（断线）：下面重新听
        }
        mission.clear();
        board_clear_command();
        board_activity("在 " + wait_station + " 等待任务");
        if (!box.running)
            inbox_start(box, client);
    };
    // 无任务 / 失败 / 断线：先 home，再去待命点。调度 standby_point 优先于 yaml。途中缓存任务。
    auto recover_home_ap14 = [&](const std::string &reason) -> int {
        move_box::tray1_far_band_clear();
        wait_station = dispatch_idle_station();
        std::cout << "[dispatch] " << reason << " → home → " << wait_station << " 待命\n";
        board_activity(reason + "，回 home");
        while (g_body_disabled_for_charge && set_body_motors(true) != 0)
        {
            const int motor = dispatch_wait_system(client, dispatch_msg::ERR_MOTOR_DISABLE);
            if (motor != 0)
                return motor;
        }
        if (!box.running)
            inbox_start(box, client);
        dispatch_note_state(dispatch_msg::WORKING);
        const int home_rc = go_home(*g_hw->arm_r, *g_hw->arm_l);
        if (home_rc != 0)
            return home_rc;
        if (hardware_abort_requested())
            return -4;
        board_activity("去等待点 " + wait_station);
        {
            ChassisTravel travel(wait_station);
            const int nav_rc = travel.join();
            if (nav_rc != 0)
                return nav_rc;
        }
        if (hardware_abort_requested())
            return -4;
        take_cached_or_standby();
        return 0;
    };
    // 充完电：手臂回 home_tcp、腰高不动，底盘去 charge_wait_station（默认 AP12）等指令。
    // 在 AP12 待命期间断线重连也留在这里，不要自动改去 AP14。
    bool post_charge_standby = false;
    auto charge_wait_name = [&]() -> std::string {
        return g_move_cfg.chassis.charge_wait_station.empty()
                   ? "AP12"
                   : g_move_cfg.chassis.charge_wait_station;
    };
    auto recover_after_charge = [&]() -> int {
        post_charge_standby = true;
        wait_station = charge_wait_name();
        std::cout << "[dispatch] 充电完成 → 手臂 home_tcp（腰高不变）→ " << wait_station
                  << " 等指令（不回 AP14）\n";
        board_activity("充电完成，手臂回 home_tcp（腰高不变）");
        while (g_body_disabled_for_charge && set_body_motors(true) != 0)
        {
            const int motor = dispatch_wait_system(client, dispatch_msg::ERR_MOTOR_DISABLE);
            if (motor != 0)
                return motor;
        }
        if (!box.running)
            inbox_start(box, client);
        dispatch_note_state(dispatch_msg::WORKING);
        const int home_rc = go_home_arms_keep_waist_z(*g_hw->arm_r, *g_hw->arm_l);
        if (home_rc != 0)
            return home_rc;
        if (hardware_abort_requested())
            return -4;
        board_activity("去 " + wait_station + " 等指令");
        {
            ChassisTravel travel(wait_station);
            const int nav_rc = travel.join();
            if (nav_rc != 0)
                return nav_rc;
        }
        if (hardware_abort_requested())
            return -4;
        take_cached_or_standby();
        // 没有缓存任务时：待命点固定为 charge_wait（AP12），不要被其它站名带去 AP14。
        if (mission.empty())
        {
            wait_station = charge_wait_name();
            if (g_hw != nullptr)
                g_hw->last_chassis_station = wait_station;
            board_activity("在 " + wait_station + " 等待任务");
        }
        return 0;
    };
    // 进入调度：只在原地 home，底盘不导航。任务到来后由各任务自己去目标点。
    // 任务结束、断线、超时无任务仍走 recover_home_ap14。
    auto boot_home_here = [&]() -> int {
        move_box::tray1_far_band_clear();
        std::string here = chassis_location_station();
        if (here.empty() && g_hw != nullptr)
            here = g_hw->last_chassis_station;
        if (here.empty())
            here = "当前位置";
        wait_station = here;
        std::cout << "[dispatch] 调度启动 → 原地 home，底盘留在 " << wait_station
                  << "，不去待命点\n";
        board_activity("调度启动，原地回 home");
        while (g_body_disabled_for_charge && set_body_motors(true) != 0)
        {
            const int motor = dispatch_wait_system(client, dispatch_msg::ERR_MOTOR_DISABLE);
            if (motor != 0)
            {
                boot_announce("failure");
                return motor;
            }
        }
        if (!box.running)
            inbox_start(box, client);
        dispatch_note_state(dispatch_msg::WORKING);
        // dispatch.mp3 一句里包含「调度连接成功」和「进入初始姿态」。
        boot_announce("dispatch");
        const int home_rc = go_home(*g_hw->arm_r, *g_hw->arm_l);
        if (home_rc != 0)
        {
            if (home_rc != -4 && !hardware_abort_requested())
                boot_announce("failure");
            return home_rc;
        }
        if (hardware_abort_requested())
            return -4;
        if (g_hw != nullptr && !g_hw->last_chassis_station.empty())
            wait_station = g_hw->last_chassis_station;
        else
        {
            const std::string now = chassis_location_station();
            if (!now.empty())
                wait_station = now;
        }
        std::cout << "[dispatch] 原地 home 完成，在 " << wait_station << " 待命\n";
        boot_announce("standby");
        take_cached_or_standby();
        return 0;
    };
    int operator_exit = 0;
    // 1：跳过，继续等下一条。0：下线或上报失败，调用方返回 operator_exit。-1：不是这三个结果。
    auto consume_operator = [&](int code) -> int {
        if (code == kDispatchTaskSkip)
        {
            g_dispatch_stay_at_tray2 = false;
            if (g_hw != nullptr && !g_hw->last_chassis_station.empty())
                wait_station = g_hw->last_chassis_station;
            mission.clear();
            board_clear_command();
            board_mission_end(false, "跳过此次任务");
            dispatch_note_state(dispatch_msg::STANDBY);
            std::cout << "[dispatch] 跳过此次任务，不汇报完成，在 " << wait_station
                      << " 等下一条\n";
            board_activity("跳过此次任务，等待下一条");
            if (!box.running)
                inbox_start(box, client);
            return 1;
        }
        if (code == kDispatchOffline)
        {
            body_restore.enabled = false;
            board_mission_end(false, "下线");
            std::cout << "[dispatch] 下线，退出调度，不再申请任务\n";
            board_activity("下线，退出调度");
            operator_exit = 0;
            return 0;
        }
        if (code == kDispatchReportFailed)
        {
            body_restore.enabled = false;
            board_mission_end(false, "异常上报失败，原地停下");
            std::cout << "[dispatch] 异常没有发出去，停在原地，不再申请任务\n";
            board_activity("异常上报失败，原地停下");
            while (!hardware_abort_requested())
                hardware_abort_sleep_ms(500);
            operator_exit = -4;
            return 0;
        }
        return -1;
    };
    dispatch_note_state(dispatch_msg::STANDBY);
    {
        const int boot_rc = boot_home_here();
        if (boot_rc != 0)
        {
            const int op = consume_operator(boot_rc);
            if (op == 0)
                return operator_exit;
            if (op != 1)
                return boot_rc;
        }
    }
    while (!hardware_abort_requested())
    {
        if (mission.empty())
        {
            std::cout << "[dispatch] 在 " << wait_station << " 保持当前姿态等待任务\n";
            board_link("等待任务");
            board_clear_command();
            board_activity("在 " + wait_station + " 等待任务");
            if (!box.running)
                inbox_start(box, client);
            bool wait_reported_charging = false;
            auto poll_wait_ms = 0;
            while (!take_ready())
            {
                if (hardware_abort_requested())
                    return -4;
                // 待命时也看一眼有没有在充：吸合就报 CHARGING，断开再回到 STANDBY。
                poll_wait_ms += 100;
                if (poll_wait_ms >= 1000)
                {
                    poll_wait_ms = 0;
                    if (ensure_chassis())
                    {
                        double level = 0, voltage = 0, current = 0, temp = 0;
                        bool charging = false;
                        int cycle = 0;
                        if (chassis_seer().GetBattery(level, charging, voltage, current, temp,
                                                      cycle) == 0)
                        {
                            publish_dispatch_battery(level);
                            if (charging && !wait_reported_charging)
                            {
                                wait_reported_charging = true;
                                std::cout << "[dispatch] 待命时检测到充电，本机记 "
                                          << dispatch_msg::CHARGING << "\n";
                                dispatch_note_state(dispatch_msg::CHARGING);
                                board_activity("充电中（待命）");
                            }
                            else if (!charging && wait_reported_charging)
                            {
                                wait_reported_charging = false;
                                dispatch_note_state(dispatch_msg::STANDBY);
                                board_activity("在 " + wait_station + " 等待任务");
                            }
                        }
                    }
                }
                hardware_abort_sleep_ms(100);
            }
        }
        if (hardware_abort_requested())
            return -4;
        if (mission.empty())
        {
            if (post_charge_standby)
            {
                // 充完电在 AP12 等指令：断线只重连，不改去 AP14。
                wait_station = charge_wait_name();
                std::cerr << "[dispatch] 充电待命断线，仍在 " << wait_station << " 重连等指令\n";
                board_link("断线，仍在 " + wait_station + " 等指令");
                board_activity("在 " + wait_station + " 等待任务");
                if (!box.running)
                    inbox_start(box, client);
                dispatch_note_state(dispatch_msg::STANDBY);
                continue;
            }
            std::cerr << "[dispatch] 调度断线或没有任务，回 home → " << wait_station << "\n";
            board_link("断线，回 AP14 待命");
            const int r = recover_home_ap14("调度断线或无任务");
            if (r != 0)
            {
                const int op = consume_operator(r);
                if (op == 1)
                    continue;
                if (op == 0)
                    return operator_exit;
                return r;
            }
            continue;
        }
        std::cout << "[dispatch] 收到任务 " << mission << "\n";
        const std::string current = mission;
        mission.clear();
        post_charge_standby = false;
        board_command(current);
        if (current != dispatch_msg::TASK_LOAD && current != dispatch_msg::TASK_TRANSFER &&
            current != dispatch_msg::TASK_UNLOAD && current != dispatch_msg::TASK_CHARGE)
        {
            std::cerr << "[dispatch] 未知任务 " << current << "，回 home → AP14\n";
            board_note("warn", "未知指令 " + current + "，回 AP14 待命");
            const int r = recover_home_ap14("未知指令 " + current);
            if (r != 0)
            {
                const int op = consume_operator(r);
                if (op == 1)
                    continue;
                if (op == 0)
                    return operator_exit;
                return r;
            }
            continue;
        }
        board_mission_begin(current);
        const auto mission_t0 = std::chrono::steady_clock::now();
        auto spent_text = [&]() {
            const double s = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - mission_t0)
                                 .count();
            std::ostringstream os;
            os << std::fixed << std::setprecision(1) << s << " s";
            return os.str();
        };
        int motor = 0;
        while (current != dispatch_msg::TASK_CHARGE && g_body_disabled_for_charge &&
               set_body_motors(true) != 0)
        {
            motor = dispatch_wait_system(client, dispatch_msg::ERR_MOTOR_DISABLE);
            if (motor == 0)
                continue;
            break;
        }
        if (motor != 0)
        {
            const int op = consume_operator(motor);
            if (op == 1)
                continue;
            if (op == 0)
                return operator_exit;
            return motor;
        }
        g_hw->last_belt_message.clear();
        g_hw->last_tray_message.clear();
        g_dispatch_nav_fault.clear();
        g_dispatch_stay_at_tray2 = false;
        dispatch_note_state(dispatch_msg::WORKING);
        int rc = -1;
        if (current == dispatch_msg::TASK_CHARGE)
            rc = dispatch_charge(client);
        else if (current == dispatch_msg::TASK_LOAD)
            rc = dispatch_load(client);
        else if (current == dispatch_msg::TASK_TRANSFER)
            rc = dispatch_transfer(client);
        else
            rc = dispatch_unload(client);
        timing_event("mission_execution",current,rc==0?"success":"failed","algorithm return code",{{"code",double(rc)}});
        if ((rc == -4 && !can_io_faulted()) || hardware_abort_requested())
        {
            board_mission_end(false, "已中止");
            return -4;
        }
        // Motion also returns -4 when its stop predicate sees a latched CAN
        // fault. That is a hardware exception, not an operator cancellation.
        if (rc == -4 && can_io_faulted())
        {
            const int choice = dispatch_wait_system(client, dispatch_exception_state());
            if (choice == 0)
            {
                mission = current;
                continue;
            }
            const int op = consume_operator(choice);
            if (op == 1)
                continue;
            if (op == 0)
                return operator_exit;
            return choice;
        }
        // Operator choices take precedence over task-specific recovery,
        // including charging. OFFLINE must never trigger home/navigation.
        if (rc == kDispatchTaskSkip || rc == kDispatchOffline || rc == kDispatchReportFailed)
        {
            const int op = consume_operator(rc);
            if (op == 1)
                continue;
            return operator_exit;
        }
        if (current == dispatch_msg::TASK_CHARGE)
        {
            if (rc == kDispatchPointsMissing)
            {
                board_mission_end(false, "充电点未下发，未执行");
                const int r = recover_home_ap14("充电点未下发");
                if (r != 0)
                {
                    const int op = consume_operator(r);
                    if (op == 1)
                        continue;
                    if (op == 0)
                        return operator_exit;
                    return r;
                }
                continue;
            }
            // 已达阈值并 mission_done 后，停充失败（-6）或汇报失败（-7）仍去 AP12 等指令。
            if (rc == 0 || rc == -6 || rc == -7)
            {
                board_mission_end(rc == 0, rc == 0
                                               ? ("充电完成，用时 " + spent_text())
                                               : ("充电收尾异常 code=" + std::to_string(rc) +
                                                  "，仍去 " + charge_wait_name() + " 等指令"));
                const int r = recover_after_charge();
                if (r != 0)
                {
                    const int op = consume_operator(r);
                    if (op == 1)
                        continue;
                    if (op == 0)
                        return operator_exit;
                    return r;
                }
                continue;
            }
            board_mission_end(false, "充电未完成 code=" + std::to_string(rc));
            const int r = recover_home_ap14("充电未完成 code=" + std::to_string(rc));
            if (r != 0)
            {
                const int op = consume_operator(r);
                if (op == 1)
                    continue;
                if (op == 0)
                    return operator_exit;
                return r;
            }
            continue;
        }
        if (rc == kDispatchPointsMissing)
        {
            board_mission_end(false, "点位未下发齐，未执行");
            const int r = recover_home_ap14("点位未下发齐");
            if (r != 0)
            {
                const int op = consume_operator(r);
                if (op == 1)
                    continue;
                if (op == 0)
                    return operator_exit;
                return r;
            }
            continue;
        }
        if (rc == kVisionQrFail)
        {
            const int choice = dispatch_report_ex(client, "二维码识别错误");
            if (choice == 0)
            {
                std::cout << "[dispatch] 选择重试，重新执行 " << current << "\n";
                mission = current;
                continue;
            }
            const int op = consume_operator(choice);
            if (op == 1)
                continue;
            if (op == 0)
                return operator_exit;
            rc = choice;
        }
        if (rc == kDispatchNoPartStay || rc == kDispatchHoldStay || rc == kVisionQrFail ||
            rc == kDispatchGripperStay)
        {
            g_dispatch_stay_at_tray2 = false;
            const std::string why = rc == kDispatchNoPartStay ? "抓取点没有料"
                                   : rc == kDispatchGripperStay ? "夹爪未到准备开度"
                                   : rc == kVisionQrFail      ? "二维码识别错误"
                                                              : "异常";
            board_mission_end(false, why + "，原地待命");
            if (g_hw != nullptr && !g_hw->last_chassis_station.empty())
                wait_station = g_hw->last_chassis_station;
            mission.clear();
            board_clear_command();
            dispatch_note_state(dispatch_msg::STANDBY);
            std::cout << "[dispatch] " << why << "，停在 " << wait_station
                      << " 待命，不回 AP14\n";
            board_activity(why + "，原地待命");
            for (;;)
            {
                if (!box.running)
                    inbox_start(box, client);
                std::string soon;
                if (!inbox_wait(box, kDispatchNextMissionWaitMs, soon))
                {
                    if (hardware_abort_requested())
                        return -4;
                    std::cout << "[dispatch] 仍在 " << wait_station << " 待命\n";
                    continue;
                }
                inbox_join(box);
                if (hardware_abort_requested())
                    return -4;
                if (!soon.empty())
                {
                    std::cout << "[dispatch] 待命中收到 " << soon << "，原地执行\n";
                    mission = soon;
                    break;
                }
            }
            continue;
        }
        if (rc != 0)
        {
            g_dispatch_stay_at_tray2 = false;
            const std::string fault = dispatch_exception_state();
            if (!fault.empty() &&
                (fault == dispatch_msg::ERR_MOTOR_LOST || fault == dispatch_msg::ERR_MOTOR_DISABLE ||
                 fault == dispatch_msg::ERR_CAMERA || fault == dispatch_msg::ERR_GRIPPER_LOST))
            {
                board_mission_end(false, "异常 " + fault + "，等待处理");
                const int choice = dispatch_wait_system(client, fault);
                if (choice == 0)
                {
                    std::cout << "[dispatch] 选择重试，重新执行 " << current << "\n";
                    mission = current;
                    continue;
                }
                const int op = consume_operator(choice);
                if (op == 1)
                    continue;
                if (op == 0)
                    return operator_exit;
                return choice;
            }
            const bool holding = g_hw != nullptr && (g_hw->last_grasped_r || g_hw->last_grasped_l);
            const std::string why =
                fault.empty()
                    ? ("任务失败 code=" + std::to_string(rc) +
                       (holding ? "（手里有件）" : "") +
                       (rc == kDispatchHoldStay ? "（申请超时/待命）" : "") +
                       (rc == kVisionQrFail ? "（二维码失败）" : ""))
                    : ("异常 " + fault);
            board_mission_end(false, why + "，回 AP14");
            const int r = recover_home_ap14(why);
            if (r != 0)
            {
                const int op = consume_operator(r);
                if (op == 1)
                    continue;
                if (op == 0)
                    return operator_exit;
                return r;
            }
            continue;
        }

        board_mission_end(true, "完成，用时 " + spent_text());
        dispatch_note_state(dispatch_msg::STANDBY);
        g_dispatch_stay_at_tray2 = false;
        if (g_hw != nullptr && !g_hw->last_chassis_station.empty())
            wait_station = g_hw->last_chassis_station;
        mission.clear();
        board_clear_command();
        std::cout << "[dispatch] 子任务完成，原地在 " << wait_station << " 听下一条 "
                  << (kDispatchNextMissionWaitMs / 1000) << "s；收到则连做，超时/断线再 home→AP14\n";
        board_activity("任务完成，原地等 " + std::to_string(kDispatchNextMissionWaitMs / 1000) +
                       "s 看有没有下一条");
        if (!box.running)
            inbox_start(box, client);
        std::string soon;
        if (inbox_wait(box, kDispatchNextMissionWaitMs, soon))
        {
            inbox_join(box);
            if (hardware_abort_requested())
                return -4;
            if (!soon.empty())
            {
                std::cout << "[dispatch] " << (kDispatchNextMissionWaitMs / 1000) << "s 内收到 "
                          << soon << "，原地执行\n";
                mission = std::move(soon);
                continue;
            }
            std::cerr << "[dispatch] 等待期间调度断线，home → AP14\n";
            const int r = recover_home_ap14("等待下一条时断线");
            if (r != 0)
            {
                const int op = consume_operator(r);
                if (op == 1)
                    continue;
                if (op == 0)
                    return operator_exit;
                return r;
            }
            continue;
        }
        if (hardware_abort_requested())
            return -4;
        // 仍连着但超时没任务：回 AP14 待命（途中若又来任务，到站后立刻执行）。
        const int r = recover_home_ap14(std::to_string(kDispatchNextMissionWaitMs / 1000) +
                                        "s 内无新任务");
        if (r != 0)
        {
            const int op = consume_operator(r);
            if (op == 1)
                continue;
            if (op == 0)
                return operator_exit;
            return r;
        }
    }
    return hardware_abort_requested() ? -4 : 0;
}

int run_dispatch_mode()
{
    const auto &ch = g_move_cfg.chassis;
    board_begin(std::string(dispatch_msg::kDefaultHost) + ":" +
                    std::to_string(dispatch_msg::kDefaultPort),
                ch.robot_type + " / " + ch.sequence_number);
    int rc = -4;
    {
        DispatchConsole console;
        rc = run_dispatch_mode_body();
    }
    board_end(rc);
    return rc;
}
}
