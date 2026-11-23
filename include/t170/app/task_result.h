#pragma once
#include "t170/skills/types.h"
#include <json/json.h>
#include <optional>

namespace t170::app {
Json::Value task_command_result(int code,int completed,const std::optional<t170::skills::Result> &skill,
                               const std::string &error,const char *operation);
}
