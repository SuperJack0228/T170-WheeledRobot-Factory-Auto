#include "t170/motion/arm.h"
#include "t170/motion/servo.h"
#include "t170/motion/trajectory_executor.h"
#include "t170/hardware/device_bus.h"
#include <cmath>
#include <mutex>
#include <chrono>
#include <thread>
#include <iostream>
#include "t170/motion/feedback_filter.h"
#include <stdexcept>

namespace wrist_servo {
bool read_joint_feedback(int hand,Joints &q) {
    if(hand<0 || hand>1)return false;
    static std::mutex mutex[2];static t170::motion::JointFeedbackFilter filter[2];
    static unsigned rejected[2]={0,0};
    std::lock_guard owner(mutex[hand]);
    const auto &d=t170::motion::arm_definition(hand);
    for(int attempt=0;attempt<3;++attempt) {
        Joints device;bool complete=true;
        for(int j=0;j<7;++j)if(!motor_read_pos_rad(d.can_ids[j],device(j))) {complete=false;break;}
        if(complete) {
            const Joints candidate=t170::motion::device_to_model(hand,device);
            const auto bounds=t170::motion::effective_arm_bounds(d);
            const double now=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
            if(filter[hand].accept(candidate,now,bounds.minimum,bounds.maximum,d.velocity)) {
                if(rejected[hand])std::clog<<"[joint_feedback] hand="<<hand<<" recovered rejected_frames="<<rejected[hand]<<'\n';
                rejected[hand]=0;q=candidate;return true;
            }
            if(++rejected[hand]<=3 || rejected[hand]%100==0)
                std::cerr<<"[joint_feedback] hand="<<hand<<" steady_s="<<now<<" rejected=invalid_or_discontinuous_model_frame q="<<candidate<<'\n';
        }
        if(attempt<2)std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false; // Never return the old frame with a new acquisition timestamp.

}
void send_joint_reference(int hand,const Joints &q) {
    if((hand!=0 && hand!=1) || !q.allFinite()) throw std::runtime_error("invalid_joint_reference");
    const auto bounds=t170::motion::effective_arm_bounds(t170::motion::arm_definition(hand));
    if((q.array()<bounds.minimum.array()-1e-8).any() || (q.array()>bounds.maximum.array()+1e-8).any())
        throw std::runtime_error("joint_reference_outside_effective_limits");
    const auto device=t170::motion::model_to_device(hand,q);
    const auto &d=t170::motion::arm_definition(hand);int counts[7];
    for(int j=0;j<7;++j) {
        if(!motor_set_pos_rad(d.can_ids[j],device(j))) throw std::runtime_error("joint_command_write_failed");
        // Legacy count cache is diagnostic only. Device SDK receives radians.
        counts[j]=static_cast<int>(device(j)*4.*65536./(2*M_PI));
    }
    save_arm_last_commanded_position(hand,counts);
}
}
