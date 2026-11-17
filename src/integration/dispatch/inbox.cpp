#include "t170/integration/dispatch/inbox.h"
#include "t170/common/abort_query.h"
#include <chrono>
namespace orch {
void inbox_start(DispatchInbox &box, DispatchClient &client)
{
    if (box.running)
        return;
    {
        std::lock_guard<std::mutex> lk(box.mu);
        box.mission.clear();
        box.finished = false;
    }
    box.running = true;
    box.worker = std::thread([&box, &client]() {
        std::string mission = client.apply_mission();
        std::lock_guard<std::mutex> lk(box.mu);
        box.mission = std::move(mission);
        box.finished = true;
    });
}
bool inbox_poll(DispatchInbox &box, std::string &out)
{
    std::lock_guard<std::mutex> lk(box.mu);
    if (!box.finished)
        return false;
    out = box.mission;
    return true;
}
void inbox_join(DispatchInbox &box)
{
    box.join_worker();
}
bool inbox_wait(DispatchInbox &box, int timeout_ms, std::string &out)
{
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (inbox_poll(box, out) || hardware_abort_requested())
            return inbox_poll(box, out);
        hardware_abort_sleep_ms(100);
    }
    return inbox_poll(box, out);
}
void close_dispatch_inbox(std::unique_ptr<DispatchClient> &holder,DispatchInbox &box) noexcept {
    if(holder) holder->close();
    box.join_worker();
    holder.reset();
}
}
