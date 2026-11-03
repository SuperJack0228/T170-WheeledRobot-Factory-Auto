#include "servo/retreat_gate.h"
#include <future>
#include <iostream>
#include <stdexcept>

void check(bool value,const char *message) {if(!value) throw std::runtime_error(message);}
int main() {
    try {
        wrist_servo::RetreatClearanceGate gate(.05);
        auto peer=std::async(std::launch::async,[&]{return gate.wait(.5);});
        gate.measured_lift(.01);
        check(peer.wait_for(std::chrono::milliseconds(20))==std::future_status::timeout,
              "peer entered before measured vertical clearance");
        gate.measured_lift(.051);check(peer.get()==0,"measured clearance did not release peer");
        wrist_servo::RetreatClearanceGate failed(.05);failed.finish(-16);
        check(failed.wait(.1)==-16,"failed first-hand plan allowed peer entry");
        wrist_servo::RetreatClearanceGate false_done(.05);false_done.finish(0);
        check(false_done.wait(.1)<0,"thread completion was mistaken for physical clearance");
        wrist_servo::RetreatClearanceGate timeout(.05);
        check(timeout.wait(.01)<0,"no feedback allowed peer entry");
        std::cout<<"retreat_gate: measured clearance, failed plan, false completion and timeout passed\n";
    } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
}
