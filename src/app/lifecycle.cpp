#include "t170/app/lifecycle.h"
#include "t170/common/abort_query.h"
#include <atomic>
#include <csignal>
#include <chrono>
#include <thread>
#include <unistd.h>
void hardware_abort_request();
namespace
{

std::atomic<bool> g_app_stop{false};
std::atomic<bool> g_app_cleanup{false};

void on_sigint(int)
{
    // 信号处理函数里只能做原子写。::close() 会 down CAN，不能在这里调用。
    hardware_abort_request();
    g_app_stop.store(true, std::memory_order_release);
    if (g_app_cleanup.load(std::memory_order_acquire))
        _Exit(130);
}

} // namespace

void request_app_stop()
{
    g_app_stop.store(true, std::memory_order_relaxed);
}

bool app_stop_requested()
{
    return g_app_stop.load(std::memory_order_relaxed);
}

void install_app_sigint_handler()
{
    struct sigaction sa{};
    sa.sa_handler = on_sigint;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0; // 不要 SA_RESTART，让 accept/poll 被信号打断
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
}

void mark_app_cleanup_started()
{
    g_app_cleanup.store(true, std::memory_order_release);
}



