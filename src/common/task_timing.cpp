#include "t170/common/task_timing.h"
#include "t170/common/paths.h"
#include "t170/common/logging.h"
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>
#include <unistd.h>
#ifndef T170_GIT_COMMIT
#define T170_GIT_COMMIT "unknown"
#endif
#ifndef T170_METRICS_BUILD_TYPE
#define T170_METRICS_BUILD_TYPE "unknown"
#endif
namespace orch {
namespace {
using Clock=std::chrono::steady_clock;
std::int64_t now_ms(){return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch()).count();}
std::string quote(const std::string &s) {
    std::ostringstream o;o<<'"';
    for(unsigned char c:s) switch(c) {
      case '"':o<<"\\\"";break;case '\\':o<<"\\\\";break;case '\n':o<<"\\n";break;case '\r':o<<"\\r";break;case '\t':o<<"\\t";break;
      default:if(c<32)o<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<int(c)<<std::dec;else o<<char(c);
    }
    o<<'"';return o.str();
}
std::string wall() {
    const auto t=std::chrono::system_clock::now();const auto sec=std::chrono::system_clock::to_time_t(t);
    std::tm tm{};gmtime_r(&sec,&tm);char b[32];std::strftime(b,sizeof b,"%Y-%m-%dT%H:%M:%S",&tm);
    std::ostringstream o;o<<b<<'.'<<std::setw(3)<<std::setfill('0')<<(std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch()).count()%1000)<<'Z';return o.str();
}
struct Writer {
    std::mutex mu;std::condition_variable cv;std::deque<std::string> queue;
    std::ofstream file;std::thread thread;bool stop=false;std::atomic<unsigned> dropped{0};std::atomic<bool> healthy{true};
    explicit Writer(const std::filesystem::path &path) {
        file.open(path,std::ios::out|std::ios::app);if(!file){healthy=false;return;}
        thread=std::thread([this]{
            for(;;){std::string line;
                {std::unique_lock<std::mutex> lk(mu);cv.wait_for(lk,std::chrono::milliseconds(250),[this]{return stop||!queue.empty();});
                 if(queue.empty()){file.flush();if(!file)healthy=false;if(stop)break;continue;}
                 line=std::move(queue.front());queue.pop_front();}
                file<<line<<'\n';if(!file)healthy=false;
            }
        });
    }
    void push(std::string line){std::lock_guard<std::mutex> lk(mu);if(queue.size()>=8192||!healthy){++dropped;return;}queue.push_back(std::move(line));cv.notify_one();}
    ~Writer(){{std::lock_guard<std::mutex> lk(mu);stop=true;}cv.notify_all();if(thread.joinable())thread.join();}
};
std::mutex mu;
std::unique_ptr<Writer> writer;
TimingContext current;
int level=-1;std::uint64_t sequence=0,mission_sequence=0;
std::int64_t run_t0=0,mission_t0=0,step_t0=0;bool step_open=false;
int read_level(){if(level<0){level=2;if(const char *p=std::getenv("T170_TIMING")){if(std::string(p)=="0")level=0;else if(std::string(p)=="1")level=1;}}return level;}
void event(const std::string &type,const std::string &name,const std::string &status,const std::string &detail,
           const std::map<std::string,double> &metrics={},const TimingContext *ctx=nullptr) {
    if(!writer)return;const auto &c=ctx?*ctx:current;
    std::ostringstream o;o<<"{\"schema_version\":2,\"seq\":"<<++sequence<<",\"type\":"<<quote(type)
      <<",\"run_id\":"<<quote(c.run_id)<<",\"mission_id\":"<<quote(c.mission_id)<<",\"mission\":"<<quote(c.mission)
      <<",\"step\":"<<quote(c.step)<<",\"name\":"<<quote(name)<<",\"status\":"<<quote(status)
      <<",\"detail\":"<<quote(detail)<<",\"wall\":"<<quote(wall())<<",\"steady_ms\":"<<now_ms()<<",\"metrics\":{";
    bool first=true;for(const auto &[key,value]:metrics){if(!first)o<<',';first=false;o<<quote(key)<<':';if(std::isfinite(value))o<<std::setprecision(15)<<value;else o<<"null";}
    o<<"}}";writer->push(o.str());
}
void close_step(const std::string &status){if(step_open){event("step",current.step,status,"",{{"dur_ms",double(now_ms()-step_t0)}});step_open=false;current.step.clear();}}
void begin_run(const std::string &robot,const std::string &server){
    const auto epoch=std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    current={};current.run_id=std::to_string(epoch)+"_"+std::to_string(getpid());run_t0=now_ms();sequence=mission_sequence=0;
    const auto dir=t170::paths::log_directory()/"dispatch_metrics";std::error_code ec;std::filesystem::create_directories(dir,ec);
    if(ec){std::cerr<<"[metrics] 不能创建目录: "<<ec.message()<<'\n';return;}
    const auto path=dir/("run_"+current.run_id+".jsonl");writer=std::make_unique<Writer>(path);
    if(!writer->healthy){std::cerr<<"[metrics] 不能写入 "<<path<<'\n';writer.reset();return;}
    // Snapshot external parameters/calibrations for later interpretation. Never copy credentials or vendor SDKs.
    const auto snapshot=dir/("run_"+current.run_id+"_config");std::filesystem::create_directories(snapshot,ec);
    unsigned errors=ec?1:0;
    for(const auto &name:{"move_box_params.yaml","realsense_cameras.yaml","Robot_Arm_Model.yaml","left_hand_to_base_result.yaml","right_hand_to_base_result.yaml"}) {
        ec.clear();const auto source=t170::paths::config_file(name);
        if(std::filesystem::exists(source)){std::filesystem::copy_file(source,snapshot/name,std::filesystem::copy_options::overwrite_existing,ec);if(ec)++errors;}
    }
    if(std::filesystem::exists(t170::paths::config_file("calibration"))) {
        ec.clear();std::filesystem::copy(t170::paths::config_file("calibration"),snapshot/"calibration",std::filesystem::copy_options::recursive,ec);if(ec)++errors;
    }
    event("run_start",robot,"running","server="+server+";git="+T170_GIT_COMMIT+";build="+T170_METRICS_BUILD_TYPE+";config="+snapshot.string(),{{"timing_level",double(read_level())},{"snapshot_errors",double(errors)}});
    std::cerr<<"[metrics] 调度统计 "<<path<<"（UTC时间，单调时钟计时）\n";
}
void end_mission(const std::string &status,const std::string &result){
    if(current.mission_id.empty())return;close_step(status);event("mission_end",current.mission,status,result,{{"dur_ms",double(now_ms()-mission_t0)}});
    current.mission_id.clear();current.mission.clear();
}
}
int timing_level(){std::lock_guard<std::mutex> lk(mu);return read_level();}
TimingContext timing_context(){std::lock_guard<std::mutex> lk(mu);return current;}
void timing_run_begin(const std::string &robot,const std::string &server){std::lock_guard<std::mutex> lk(mu);if(read_level()==0)return;
    if(writer){end_mission("interrupted","new run started");event("run_end","","interrupted","new run started");writer.reset();}begin_run(robot,server);}
void timing_run_end(int code){std::lock_guard<std::mutex> lk(mu);if(!writer)return;end_mission("interrupted","dispatch ended before mission result");
    event("run_end","",code==0?"completed":code==-4?"aborted":"failed","",{{"code",double(code)},{"dur_ms",double(now_ms()-run_t0)},{"dropped_events",double(writer->dropped.load())},{"writer_healthy",writer->healthy?1.:0.}});writer.reset();current={};}
void timing_mission_begin(const std::string &mission){std::lock_guard<std::mutex> lk(mu);if(read_level()==0)return;if(!writer)begin_run("unknown","standalone");
    end_mission("interrupted","next mission began without prior result");current.mission=mission;current.mission_id=current.run_id+"_m"+std::to_string(++mission_sequence);mission_t0=now_ms();
    event("mission_start",mission,"running","");}
void timing_mission_end(bool ok,const std::string &result){std::lock_guard<std::mutex> lk(mu);end_mission(ok?"success":"failed",result);}
void timing_step(const std::string &step){std::lock_guard<std::mutex> lk(mu);if(!writer||current.mission_id.empty())return;close_step("success");if(!step.empty()){current.step=step;step_t0=now_ms();step_open=true;event("step_start",step,"running","");}}
void timing_event(const std::string &type,const std::string &name,const std::string &status,const std::string &detail,const std::map<std::string,double> &metrics,const TimingContext *context){
    std::lock_guard<std::mutex> lk(mu);if(!writer || (context && context->run_id!=current.run_id))return;event(type,name,status,detail,metrics,context);}
TimingSpan::TimingSpan(const char *phase){std::lock_guard<std::mutex> lk(mu);if(!phase||!*phase||read_level()<2||current.mission_id.empty())return;
    active_=true;phase_=phase;context_=current;t0_ms_=now_ms();exceptions_=std::uncaught_exceptions();}
void TimingSpan::set_ok(bool ok){ok_=ok;}
TimingSpan::~TimingSpan(){if(active_)timing_event("span",phase_,ok_ && std::uncaught_exceptions()==exceptions_?"success":"failed","",{{"dur_ms",double(now_ms()-t0_ms_)}},&context_);}
}
