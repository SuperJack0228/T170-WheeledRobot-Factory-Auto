#pragma once
// Contract double for adapter tests ONLY. Never added to production include paths.
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
class RobotClient {
public:
    inline static bool is_map_updated=false;
    inline static std::string raw_material_point,well_done_point,standby_point,charge_point,
        sequence_one_port_one,sequence_one_port_two,sequence_two_port_one,sequence_two_port_two,current_point;
    inline static float x=0,y=0,angle=0;
    inline static int power_percent=0,last_timeout=0,acks=0;
    inline static std::string last_exception_message,next_exception_result="RETRY";
    inline static std::string last_type,last_serial;
    struct ReceiveState {std::mutex mutex;std::condition_variable ready;bool entered=false,closed=false,returned=false;};
    inline static std::shared_ptr<ReceiveState> last_receive;
    std::shared_ptr<ReceiveState> receive=std::make_shared<ReceiveState>();
    RobotClient(const std::string &type,const std::string &serial){last_type=type;last_serial=serial;last_receive=receive;}
    ~RobotClient(){std::lock_guard<std::mutex> lock(receive->mutex);receive->closed=true;receive->ready.notify_all();}
    std::string apply_mission(){
        auto state=receive;std::unique_lock<std::mutex> lock(state->mutex);
        state->entered=true;state->ready.notify_all();
        state->ready.wait(lock,[&]{return state->closed;});state->returned=true;return {};
    }
    bool apply_place_raw_material(int ms){last_timeout=ms;return true;}
    bool apply_pick_up_half_done(int ms){last_timeout=ms;return true;}
    bool apply_place_half_done(int ms){last_timeout=ms;return true;}
    bool apply_pick_up_well_done(int ms){last_timeout=ms;return true;}
    bool mission_done(){++acks;return true;}
    bool pick_up_done(){++acks;return true;}
    bool place_done(){++acks;return true;}
    bool raw_material_empty(){++acks;return true;}
    bool well_done_material_full(){++acks;return true;}
    std::string report_exception(const std::string &message){last_exception_message=message;return next_exception_result;}
};
