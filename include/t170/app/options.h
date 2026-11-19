#pragma once
#include <string>
namespace t170::app {
enum class Mode { Maintenance, Production };
struct Options {Mode mode=Mode::Maintenance;int port=8099;bool help=false,list_commands=false;};
Options parse_options(int argc,const char *const *argv);
const char *mode_name(Mode);
const char *usage();
}
