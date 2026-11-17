#pragma once
#include "t170/app/options.h"
#include <string_view>
#include <vector>
namespace t170::app {
enum class CommandGroup { Operation, Calibration, Maintenance, Diagnostic, Retired };
struct CommandSpec {std::string_view name;CommandGroup group;bool production;};
const std::vector<CommandSpec> &command_catalog();
const CommandSpec *find_command(std::string_view);
bool command_allowed(Mode,std::string_view);
// Queries/control must not clear the arrival log of a concurrently running task.
bool command_collects_arrivals(std::string_view);
const char *group_name(CommandGroup);
}
