#pragma once
#include <string>
namespace orch {
std::string console_start();
void console_stop();
void print_event(const std::string &);
}

namespace orch {std::string now_hms();}
