#pragma once
#include "t170/skills/types.h"
#include <chrono>
#include <mutex>

namespace t170::skills::detail {
// Lease bits: left=1, right=2, shared tray selection/waist state=4.
// Opposite-arm conveyor retreat/contact may overlap; tray cycles own all three.
class Execution {
public:
    Execution(Skill, Metadata, Hands, Control, unsigned resources);
    ~Execution();
    Execution(const Execution &) = delete;
    Execution &operator=(const Execution &) = delete;
    bool entered() const { return owns_; }
    bool cancelled();
    void stage(Stage, Hands);
    void native(int, Hands, Stage);
    void fail(Error, std::string message, std::optional<int> native = {});
    Result finish();
    Result result;
private:
    Control control_;
    unsigned resources_ = 0;
    bool owns_ = false;
    bool finished_ = false;
    std::uint64_t sequence_ = 0;
    std::chrono::steady_clock::time_point began_ = std::chrono::steady_clock::now();
    std::recursive_mutex event_mutex_;
};
extern thread_local Execution *active_execution;
class ExecutionScope {
    Execution *previous_;
public:
    explicit ExecutionScope(Execution &execution) : previous_(active_execution) { active_execution = &execution; }
    ~ExecutionScope() { active_execution = previous_; }
};
void report(Stage, bool right, bool left);
void report_native(int right_code, int left_code, bool right, bool left, Stage);
void report_action(bool right, PartState part, Evidence evidence, bool complete);
} // namespace t170::skills::detail
