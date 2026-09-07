#pragma once

#include <cstdint>
#include <vector>

namespace ti5_can_bridge
{
    // 封装 ti5_body_controller_leg_0408 的核心接口，供 main3 使用。
    bool Init();
    void Cleanup();

    void MotorEnable(uint8_t can_id);
    void MotorDisable(uint8_t can_id);
    void ClearError(uint8_t can_id);

    bool SetCurrent(uint8_t can_id, int current);
    bool SetSpeed(uint8_t can_id, int speed);
    bool SetPositionCount(uint8_t can_id, int position_count);
    bool SetPositionRad(uint8_t can_id, float rad);

    int GetPosition(uint8_t can_id);
    int GetSpeed(uint8_t can_id);
    int GetCurrent(uint8_t can_id);
    std::vector<int> GetCSP(uint8_t can_id);
}
