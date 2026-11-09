#pragma once
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>

namespace wrist_servo {
// A started worker or a planned trajectory is not physical clearance.
class RetreatClearanceGate {
public:
    explicit RetreatClearanceGate(double clearance_m): clearance_(clearance_m) {}
    void measured_lift(double gain_m) {
        std::lock_guard<std::mutex> lock(mu_);
        if(std::isfinite(gain_m) && gain_m>=clearance_) clear_=true;
        cv_.notify_all();
    }
    void release() {
        std::lock_guard<std::mutex> lock(mu_);
        clear_=true;
        cv_.notify_all();
    }
    void finish(int code) {
        std::lock_guard<std::mutex> lock(mu_);
        finished_=true;code_=code;cv_.notify_all();
    }
    // 0 permits the peer; negative means keep it outside the shared work area.
    int wait(double seconds) {
        std::unique_lock<std::mutex> lock(mu_);
        if(!cv_.wait_for(lock,std::chrono::duration<double>(seconds),[&]{return clear_ || finished_;})) return -8;
        if(finished_ && code_!=0) return code_;
        return clear_?0:-8;
    }
private:
    double clearance_;
    std::mutex mu_;std::condition_variable cv_;
    bool clear_=false,finished_=false;int code_=0;
};
}
