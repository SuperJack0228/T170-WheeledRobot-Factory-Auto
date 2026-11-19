#include "t170/motion/arm.h"
#include "t170/motion/servo.h"
#include "t170/motion/trajectory_executor.h"
#include "t170/hardware/device_bus.h"
#include <cmath>
#include <stdexcept>

namespace wrist_servo {
bool read_joint_feedback(int hand,Joints &q) {
    const auto &d=t170::motion::arm_definition(hand);Joints device;
    for(int j=0;j<7;++j) if(!motor_read_pos_rad(d.can_ids[j],device(j))) return false;
    const Joints candidate=t170::motion::device_to_model(hand,device);
    if(!candidate.allFinite())return false;
    q=candidate;return true;
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
