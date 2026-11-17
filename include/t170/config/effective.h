#pragma once
#include "t170/common/config.h"
namespace t170::config {
// All loaded members, including C++ defaults and calibrated profiles. No SDK IO.
std::string effective_yaml(const MoveBoxConfig &cfg);
}
