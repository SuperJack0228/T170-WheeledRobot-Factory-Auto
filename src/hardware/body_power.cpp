#include "t170/hardware/body_power.h"
#include "t170/hardware/device_bus.h"
#include <iostream>
namespace t170::hardware {
int set_body_motors(bool enable)
{
    uint32_t ids[22];
    int n = 0;
    auto push = [&](uint32_t id) { ids[n++] = id; };
    for (uint32_t id : kWaistAllCanIds)
        push(id);
    if (!right_arm_motors_locked())
    {
        for (uint32_t id : kRightArmCanIds)
            push(id);
    }
    for (uint32_t id : kLeftArmCanIds)
        push(id);
    for (uint32_t id : kHeadCanIds)
        push(id);
    std::cout << "[dispatch] " << (enable ? "使能" : "失能") << "全身电机，共 " << n << " 轴\n";
    if (enable)
    {
        bool ok = true;
        for (int i = 0; i < n; ++i)
            ok = motor_enable_id(ids[i]) && ok;
        if (!ok)
        {
            std::cerr << "[dispatch] 使能全身电机失败\n";
            return -5;
        }
        return 0;
    }
    if (socketcan_sendsimplecommand(0, n, ids, 2, nullptr) != 1)
    {
        std::cerr << "[dispatch] 失能全身电机失败\n";
        return -5;
    }
    return 0;
}
}
