#include "t170/common/task_timing.h"
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>
int main(int argc,char **argv){if(argc<2||argc>3)return 2;setenv("T170_ROOT",argv[1],1);unsetenv("T170_TIMING");unsetenv("T170_LOG_DIR");if(argc==3)setenv("T170_TIMING","0",1);
    orch::timing_run_begin("synthetic_robot","offline");orch::timing_mission_begin("LOAD");
    const auto original=orch::timing_context();orch::timing_step("parallel step");
    auto span=std::make_unique<orch::TimingSpan>("long lived span");
    orch::timing_event("manual_intervention","operator","recorded","quote\" control\x01 newline\n",{{"count",1}});
    orch::timing_mission_end(true,"done");orch::timing_mission_begin("UNLOAD");span.reset();
    std::thread child([]{orch::timing_event("servo_end","belt2_grasp","success","synthetic",{{"hand",1},{"circle_accepted_frames",3}});});child.join();
    orch::timing_mission_end(false,"synthetic failure");orch::timing_run_end(0);
    std::cout<<original.run_id;return 0;
}
