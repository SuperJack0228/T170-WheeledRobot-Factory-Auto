#pragma once
#include <cstdint>
#include <map>
#include <string>

namespace orch {
// Dispatch structured telemetry, enabled by default. T170_TIMING=0/1/2:
// off / mission+step / mission+step+spans. No per-control-tick disk writes.
int timing_level();
struct TimingContext {std::string run_id,mission_id,mission,step;};
TimingContext timing_context();
void timing_run_begin(const std::string &robot,const std::string &server);
void timing_run_end(int code);
void timing_mission_begin(const std::string &mission);
void timing_mission_end(bool ok,const std::string &result);
void timing_step(const std::string &step);
void timing_event(const std::string &type,const std::string &name,const std::string &status,
                  const std::string &detail="",const std::map<std::string,double> &metrics={},
                  const TimingContext *context=nullptr);
class TimingSpan {
public:
    explicit TimingSpan(const char *phase);
    ~TimingSpan();
    TimingSpan(const TimingSpan &)=delete;
    TimingSpan &operator=(const TimingSpan &)=delete;
    void set_ok(bool ok);
    void fail(){set_ok(false);}
private:
    bool active_=false,ok_=true;
    int exceptions_=0;
    std::string phase_;
    TimingContext context_;
    std::int64_t t0_ms_=0;
};
}
