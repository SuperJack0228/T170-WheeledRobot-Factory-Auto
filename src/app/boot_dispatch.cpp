#include "internal/state.h"
#include "t170/hardware/device_bus.h"
#include "t170/vision/debug.h"
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <thread>

namespace t170::app::detail {
void start_boot_dispatch()
{
    std::thread([]() {
        std::lock_guard<std::mutex> lock(g_hw_mu);
        if (app_stop_requested() || g_placement_calibration.active())
            return;
        g_abort.store(false);
        hardware_abort_clear();
        if (app_stop_requested())
        {
            g_abort.store(true);
            hardware_abort_request();
            return;
        }
        can_io_fault_clear();
        can_io_watch_begin();
        struct WatchEnd
        {
            ~WatchEnd() { can_io_watch_end(); }
        } watch_end;
        std::cerr << "[t170_factory] 开机进入调度：原地 home 后待命\n" << std::flush;
        int rc = 0;
        try
        {
            rc = orch::run_dispatch_mode();
        }
        catch (const std::exception &ex)
        {
            std::cerr << "[t170_factory] 开机调度异常: " << ex.what() << "\n";
            orch::boot_announce("failure");
            return;
        }
        catch (...)
        {
            std::cerr << "[t170_factory] 开机调度异常\n";
            orch::boot_announce("failure");
            return;
        }
        std::cerr << "[t170_factory] 开机调度结束 code=" << rc << "\n" << std::flush;
        if (rc != 0 && rc != -4)
            orch::boot_announce("failure");
    }).detach();
}
}
