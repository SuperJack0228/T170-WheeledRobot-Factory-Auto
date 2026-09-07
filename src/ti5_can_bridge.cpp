#include "ti5_can_bridge.h"
#include <cstdint>
#include <vector>

// 只声明桥接层需要的符号，避免包含 Can_Ti5.h 后触发其全局对象静态初始化。
bool init_can();
void cleanup_can_channels();
void Motor_Enable(uint8_t Can_ID);
void Motor_Disable(uint8_t Can_ID);
void Clear_Error(uint8_t Can_ID);
bool set_Current(uint8_t Can_ID, int Current);
bool set_Speed(uint8_t Can_ID, int Speed);
bool set_Position(uint8_t Can_ID, int Position);
int get_Position(uint8_t Can_ID);
int get_Speed(uint8_t Can_ID);
int get_Current(uint8_t Can_ID);
std::vector<int> get_CSP(uint8_t Can_ID);
int Rad2Count(float Rad, int Deduction_Rate = 4);

namespace ti5_can_bridge
{
    bool Init()
    {
        return init_can();
    }

    void Cleanup()
    {
        cleanup_can_channels();
    }

    void MotorEnable(uint8_t can_id)
    {
        Motor_Enable(can_id);
    }

    void MotorDisable(uint8_t can_id)
    {
        Motor_Disable(can_id);
    }

    void ClearError(uint8_t can_id)
    {
        Clear_Error(can_id);
    }

    bool SetCurrent(uint8_t can_id, int current)
    {
        return set_Current(can_id, current);
    }

    bool SetSpeed(uint8_t can_id, int speed)
    {
        return set_Speed(can_id, speed);
    }

    bool SetPositionCount(uint8_t can_id, int position_count)
    {
        return set_Position(can_id, position_count);
    }

    bool SetPositionRad(uint8_t can_id, float rad)
    {
        return set_Position(can_id, Rad2Count(rad));
    }

    int GetPosition(uint8_t can_id)
    {
        return get_Position(can_id);
    }

    int GetSpeed(uint8_t can_id)
    {
        return get_Speed(can_id);
    }

    int GetCurrent(uint8_t can_id)
    {
        return get_Current(can_id);
    }

    std::vector<int> GetCSP(uint8_t can_id)
    {
        return get_CSP(can_id);
    }
}
