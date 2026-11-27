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
void dispatch_note_state(const std::string &state)
{
    std::cout << "[dispatch] 本机状态 " << state
              << "（站点随 POSE 的 current_point 上报）\n";
    board_state(state, true);
}

bool dispatch_apply(
    const std::string &request, const std::string &grant, const std::string &what,
    const std::function<bool()> &call)
{
    TimingSpan span("apply_wait");
    board_apply_begin(request, grant, what);
    const bool ok = call();
    board_apply_end(ok);
    span.set_ok(ok);
    return ok;
}

namespace {

}

namespace {

}

int dispatch_report(DispatchClient &client, DispatchNote note, const char *what)
{
    const char *state = dispatch_msg::DONE;
    if (note == DispatchNote::PlaceHand)
        state = dispatch_msg::PLACE_DONE;
    else if (note == DispatchNote::PickHand)
        state = dispatch_msg::PICK_UP_DONE;
    std::cout << "[dispatch] " << what << " 完成，停 2s 后发送 " << state << "\n";
    if (note == DispatchNote::Mission)
        board_step("汇报完成");
    board_report_pending(state, what);
    hardware_abort_sleep_ms(2000);
    if (hardware_abort_requested())
        return -4;
    const bool ok = note == DispatchNote::PlaceHand ? client.place_done()
                    : note == DispatchNote::PickHand ? client.pick_up_done()
                                                     : client.mission_done();
    board_report(state, what, ok);
    if (!ok)
    {
        std::cerr << "[dispatch] 发送 " << state << " 失败\n";
        return -7;
    }
    std::cout << "[dispatch] 已发送 " << state << "\n";
    return 0;
}

int dispatch_report_now(DispatchClient &client, DispatchNote note, const char *what)
{
    const char *state = dispatch_msg::DONE;
    if (note == DispatchNote::PlaceHand)
        state = dispatch_msg::PLACE_DONE;
    else if (note == DispatchNote::PickHand)
        state = dispatch_msg::PICK_UP_DONE;
    std::cout << "[dispatch] " << what << " 手已收回，发送 " << state << std::endl;
    if (note == DispatchNote::Mission)
        board_step("汇报完成");
    board_report_pending(state, what);
    if (hardware_abort_requested())
        return -4;
    const bool ok = note == DispatchNote::PlaceHand ? client.place_done()
                    : note == DispatchNote::PickHand ? client.pick_up_done()
                                                     : client.mission_done();
    board_report(state, what, ok);
    if (!ok)
    {
        std::cerr << "[dispatch] 发送 " << state << " 失败\n";
        return -7;
    }
    std::cout << "[dispatch] 已发送 " << state << std::endl;
    return 0;
}

namespace {

}

void dispatch_send(DispatchClient &client, const char *name, bool sent)
{
    (void)client;
    std::cout << "[dispatch] 发送 " << name << "\n";
    const std::string n = name;
    board_report(n, n == dispatch_msg::RAW_MATERIAL_EMPTY        ? "料盘一已空"
                    : n == dispatch_msg::WELL_DONE_MATERIAL_FULL ? "料盘二已满"
                                                                 : n,
                 sent);
    if (sent && n == dispatch_msg::RAW_MATERIAL_EMPTY)
        board_tray1_empty_reported();
    if (sent && n == dispatch_msg::WELL_DONE_MATERIAL_FULL)
        board_tray2_full_reported();
    if (!sent)
        std::cerr << "[dispatch] 发送 " << name << " 失败，继续当前流程\n";
    else
        std::cout << "[dispatch] 已发送 " << name << "\n";
}

int dispatch_report_ex(DispatchClient &client, const char *zh)
{
    using t170::integration::dispatch::kReportOffline;
    using t170::integration::dispatch::kReportRetry;
    using t170::integration::dispatch::kReportSkip;
    if (hardware_abort_requested())
        return -4;
    std::string message = zh ? zh : "";
    const auto first = message.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
        message.clear();
    else
        message = message.substr(first, message.find_last_not_of(" \t\r\n") - first + 1);
    // Publish before entering the SDK's blocking operator prompt.
    board_error(message.empty() ? "异常说明为空，未发送" : message);
    board_activity("异常等待处理：" + message);
    board_report_pending("异常处理", message);
    TimingSpan wait("exception_wait");
    std::string decision;
    try
    {
        if (!message.empty())
            decision = client.report_exception(message);
    }
    catch (const std::exception &ex)
    {
        std::cerr << "[dispatch] 异常上报调用失败: " << ex.what() << '\n';
    }
    if (hardware_abort_requested())
    {
        wait.fail();
        timing_event("recovery", "dispatch_resolution", "cancelled", message);
        board_report("已中止", message, false);
        return -4;
    }
    const char *label = "发送失败";
    int code = kDispatchReportFailed;
    if (decision == kReportRetry)
    {
        label = "重试";
        code = 0;
    }
    else if (decision == kReportSkip)
    {
        label = "跳过此次任务";
        code = kDispatchTaskSkip;
    }
    else if (decision == kReportOffline)
    {
        label = "下线";
        code = kDispatchOffline;
    }
    const bool recognized = code != kDispatchReportFailed;
    wait.set_ok(recognized);
    timing_event("recovery", "dispatch_resolution", decision.empty() ? "unsent" :
                 (recognized ? decision : "invalid_response"), message);
    std::cout << "[dispatch] 上报「" << message << "」，结果="
              << (decision.empty() ? "空" : decision) << "（" << label << "）\n";
    board_report(recognized ? decision : "上报失败", message, recognized);
    return code;
}

int dispatch_belt_vision_again(DispatchClient &client, int &vision_retry, const char *what)
{
    const char *name = (what && what[0]) ? what : "传送带";
    if (vision_retry > 0)
    {
        --vision_retry;
        timing_event("retry","belt_perception","retrying",what?what:"");
        std::cerr << "[dispatch] " << name << " 视觉失败，再拍一次\n";
        board_note("warn", std::string(name) + " 视觉失败，再拍一次");
        board_activity(std::string(name) + "：再拍一次");
        return 0;
    }
    std::cerr << "[dispatch] " << name << " 再拍仍失败，上报传送带视觉识别错误\n";
    const int choice = dispatch_report_ex(client, "传送带视觉识别错误");
    if (choice == 0)
        vision_retry = 1;
    return choice;
}

bool apply_pick_half(DispatchClient &client, int timeout_ms)
{
    return client.apply_pick_up_half_done(timeout_ms);
}

bool apply_pick_well(DispatchClient &client, int timeout_ms)
{
    return client.apply_pick_up_well_done(timeout_ms);
}
}
