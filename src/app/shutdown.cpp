#include "internal/state.h"
#include "t170/hardware/device_bus.h"
#include "t170/vision/debug.h"
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <thread>
#include "t170/vision/python_runtime.h"

namespace t170::app::detail {
[[noreturn]] void shutdown_process(int code)
{
    static std::atomic<int> once{0};
    if (once.exchange(1) != 0)
        _Exit(code);

    // 先打标：清理若卡在 close()，再按一次 Ctrl+C 才会硬退出。
    mark_app_cleanup_started();
    g_abort.store(true);
    hardware_abort_request();
    request_app_stop();
    std::cerr << "[t170_factory] 正在退出，释放 CAN\n" << std::flush;

    // 持锁直到进程结束，避免已排队的客户端在 close() 之后再写电机。
    orch::stop_chassis_status_reporter();
    g_hw_mu.lock();
    try
    {
        if (g_grip)
            g_grip->stop();
        if(g_pipeline) shutdown_pose_pipeline(*g_pipeline,false);
        close_can_channels();
    }
    catch (const std::exception &ex)
    {
        std::cerr << "[t170_factory] 退出清理异常: " << ex.what() << "\n";
        try
        {
            close_can_channels();
        }
        catch (...)
        {
        }
    }
    catch (...)
    {
        try
        {
            close_can_channels();
        }
        catch (...)
        {
        }
    }
    std::cerr << "[t170_factory] CAN 已释放\n" << std::flush;
    _Exit(code);
}
}
