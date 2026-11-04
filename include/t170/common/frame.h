#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>

enum class CameraSlot
{
    Head,
    RightHand,
    LeftHand,
};

struct CameraFrameData
{
    int width = 0;
    int height = 0;
    std::vector<uint8_t> color_bgr; /**< BGR 行连续, size = H*W*3 */
    std::vector<float> depth_m;     /**< 与彩色同尺寸, size = H*W；无效为 nan */
    std::array<double, 9> K{};      /**< 3×3 行主序 */
    std::array<double, 5> dist{};   /**< k1,k2,p1,p2,k3 */
    bool ok = false;
    std::string message;
    std::uint64_t frame_number = 0;
    double capture_steady_s = 0.0;
    double timestamp_uncertainty_s = 1.0;
    double device_timestamp_ms = 0.0;
    int timestamp_source = 0; // 1 system epoch, 2 global epoch, 3 bounded hardware-clock estimate
};
