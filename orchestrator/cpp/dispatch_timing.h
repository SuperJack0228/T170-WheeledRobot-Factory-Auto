#pragma once

/** 调度两层计时：L1 粗步骤（对齐 board_step），L2 细相位（nav/apply/…）。
 *  T170_TIMING=0 关闭（默认）；1 只 L1；2 L1+L2。
 *  结果追加写入 logs/timing_YYYYMMDD_HHMMSS.jsonl。
 */

#include <cstdint>
#include <string>

namespace orch
{

/** 0=关，1=L1，2=L1+L2。读环境变量 T170_TIMING，非法值当 0。 */
int timing_level();

void timing_mission_begin(const std::string &mission);
void timing_mission_end(bool ok, const std::string &result);
/** 结束上一段 L1 步骤并开始新步骤；step 空则只结束。 */
void timing_step(const std::string &step);

/** L2 细相位。level<2 时构造/析构为空操作。 */
class TimingSpan
{
public:
    explicit TimingSpan(const char *phase);
    ~TimingSpan();
    TimingSpan(const TimingSpan &) = delete;
    TimingSpan &operator=(const TimingSpan &) = delete;
    void set_ok(bool ok);
    void fail() { set_ok(false); }

private:
    bool active_ = false;
    bool ok_ = true;
    std::string phase_;
    std::int64_t t0_ms_ = 0;
};

} // namespace orch
