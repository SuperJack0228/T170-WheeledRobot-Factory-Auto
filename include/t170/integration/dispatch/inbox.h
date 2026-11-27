#pragma once
#include "t170/integration/dispatch/client.h"
#include <thread>
#include <mutex>
namespace orch {
using DispatchClient=t170::integration::dispatch::Client;
struct DispatchInbox
{
    std::thread worker;
    std::mutex mu;
    std::string mission;
    bool finished = false;
    bool running = false;

    void join_worker()
    {
        if (worker.joinable())
            worker.join();
        running = false;
    }

    ~DispatchInbox()
    {
        join_worker();
    }
};
void inbox_start(DispatchInbox &box, DispatchClient &client);
bool inbox_poll(DispatchInbox &box, std::string &out);
// Reap a completed receive only; timeout leaves the same worker listening.
// Shutdown must use close_dispatch_inbox to unblock and join pending receives.
void inbox_join(DispatchInbox &box);
bool inbox_wait(DispatchInbox &box, int timeout_ms, std::string &out);
void close_dispatch_inbox(std::unique_ptr<DispatchClient> &,DispatchInbox &) noexcept;
}
