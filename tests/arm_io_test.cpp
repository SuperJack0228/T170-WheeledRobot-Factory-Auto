#include "t170/motion/arm.h"
#include "t170/motion/servo.h"
#include <map>
#include <iostream>
#include <stdexcept>
namespace {std::map<unsigned,double> state,writes;unsigned fail_id=0,corrupt_first=0;int corrupt_frames=0,reads=0;int saved=0;}
bool motor_read_pos_rad(unsigned id,double &q){if(id==fail_id)return false;++reads;q=state.at(id);if(corrupt_frames>0 && id>=corrupt_first+3 && id<=corrupt_first+6)q=0;
 if(corrupt_frames>0 && id==corrupt_first+6)--corrupt_frames;return true;}
bool motor_set_pos_rad(unsigned id,double q){if(id==fail_id)return false;writes[id]=q;return true;}
void save_arm_last_commanded_position(int,const int[7]){++saved;}
void check(bool v,const char *s){if(!v)throw std::runtime_error(s);}
int main(int argc,char **argv){
 try {
    check(argc==2,"runtime required");setenv("T170_ROOT",argv[1],1);unsetenv("T170_CONFIG_DIR");
    for(int hand:{0,1}) {
        const auto &d=t170::motion::arm_definition(hand);
        const t170::motion::ArmJoints q=(d.minimum+d.maximum)*.5;
        const auto raw=q.cwiseProduct(d.motor_direction)+d.motor_offset;
        for(int j=0;j<7;++j)state[d.can_ids[j]]=raw(j);
        wrist_servo::Joints actual;
        check(wrist_servo::read_joint_feedback(hand,actual) && actual.isApprox(q,1e-12),"feedback did not convert device to model exactly once");
        wrist_servo::send_joint_reference(hand,q);
        for(int j=0;j<7;++j)check(std::abs(writes.at(d.can_ids[j])-raw(j))<1e-12,"command zero/sign or units mismatch");
        const auto bounds=t170::motion::effective_arm_bounds(d);
        {std::lock_guard lock(d.limits->mutex);d.limits->bounds.maximum(6)=q(6)-.01;}
        const auto previous_writes=writes;bool rejected=false;
        try{wrist_servo::send_joint_reference(hand,q);}catch(const std::runtime_error &){rejected=true;}
        check(rejected && writes==previous_writes,"effective driver bound bypassed at IO boundary");
        {std::lock_guard lock(d.limits->mutex);d.limits->bounds=bounds;}
        const auto healthy=actual;corrupt_first=d.can_ids.front();corrupt_frames=1;
        const int prior_reads=reads;
        check(wrist_servo::read_joint_feedback(hand,actual) && actual==healthy && reads-prior_reads==14,
            "zero-filled frame was accepted or no complete retry occurred");
        corrupt_frames=10;
        check(!wrist_servo::read_joint_feedback(hand,actual) && actual==healthy,"persistent bad frames replaced feedback");
        corrupt_frames=0;
        check(wrist_servo::read_joint_feedback(hand,actual) && actual==healthy,"feedback did not recover after invalid samples");
        const auto before=actual;fail_id=d.can_ids[3];
        check(!wrist_servo::read_joint_feedback(hand,actual) && actual==before,"partial read replaced valid feedback");
        const int count=saved;bool threw=false;try{wrist_servo::send_joint_reference(hand,q);}catch(const std::runtime_error&){threw=true;}
        check(threw && saved==count,"failed write committed full command cache");fail_id=0;
    }
    std::cout<<"Real arm IO adapters, both sides: model/device zero/sign/radians and partial failure handling passed; fake raw transport only\n";
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
}
