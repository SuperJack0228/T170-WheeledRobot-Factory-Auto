#pragma once

/**
 * M1–M5 公共类型别名。
 * Frame 仍是 CameraFrameData，避免全库改名；新代码优先用 vision::Frame。
 */
#include "realsense_get.h"

namespace vision
{

using Frame = CameraFrameData;

} // namespace vision
