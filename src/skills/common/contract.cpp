#include "t170/skills/types.h"
#include "../internal/execution.h"
#include <atomic>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <utility>

namespace t170::skills {
const char *name(Skill v) {
    switch(v) {case Skill::TrayGrasp:return "tray.grasp";case Skill::TrayPlace:return "tray.place";
    case Skill::ConveyorGrasp:return "conveyor.grasp_contact";case Skill::ConveyorPlace:return "conveyor.place_contact";
    case Skill::ConveyorRetreat:return "conveyor.retreat";} return "unknown";
}
const char *name(Stage v) {
    switch(v) {case Stage::Validate:return "validate";case Stage::Prepare:return "prepare";
    case Stage::Observe:return "observe";case Stage::Approach:return "approach";
    case Stage::AlignDescend:return "align_descend";case Stage::Gripper:return "gripper";
    case Stage::Lift:return "lift";case Stage::Return:return "return";case Stage::Complete:return "complete";}return "unknown";
}
const char *name(Status v) {
    switch(v) {case Status::Running:return "running";case Status::Succeeded:return "succeeded";
    case Status::Partial:return "partial";case Status::NoTarget:return "no_target";
    case Status::Failed:return "failed";case Status::Cancelled:return "cancelled";case Status::Rejected:return "rejected";}return "unknown";
}
const char *name(PartState v) {switch(v) {case PartState::Unknown:return "unknown";case PartState::Empty:return "empty";case PartState::Holding:return "holding";}return "unknown";}
const char *name(Evidence v) {switch(v) {case Evidence::Unknown:return "unknown";case Evidence::CallerReported:return "caller_reported";case Evidence::LegacyPolicy:return "legacy_policy";case Evidence::GripperFeedback:return "gripper_feedback";}return "unknown";}
Error from_native_code(int code, Stage stage) {
    if(code==0) return Error::None;
    if(code==-4) return Error::Cancelled;
    if(code==-2) return Error::Device;
    if(code==-38) return Error::ReleaseUnconfirmed;
    if(code==11) return Error::EmptyGrasp; // Existing belt empty-grasp code.
    if(code==-36) return Error::Servo;
    if(code==-30 || code==-31 || code==-33 || code==-34 || code==-35 || code==-37 || code==-39)
        return Error::Servo;
    if(stage==Stage::Observe) return Error::Perception;
    if(stage==Stage::Gripper) return Error::Gripper;
    return Error::Motion;
}
namespace {
std::string quoted(const std::string &s) {
    std::ostringstream out;out<<'"';
    for(unsigned char c:s) {
        if(c=='"' || c=='\\') out<<'\\'<<char(c);
        else if(c<32) out<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<unsigned(c)<<std::dec;
        else out<<char(c);
    }out<<'"';return out.str();
}
void optional(std::ostream &out, const std::optional<int> &v) {if(v) out<<*v;else out<<"null";}
}
std::string to_json(const Result &r) {
    std::ostringstream o;o.imbue(std::locale::classic());
    o<<"{\"interface_version\":"<<r.version<<",\"request_id\":"<<quoted(r.metadata.request_id)
     <<",\"workcell_id\":"<<quoted(r.metadata.workcell_id)<<",\"target_id\":"<<quoted(r.metadata.target_id)
     <<",\"skill\":"<<quoted(name(r.skill))<<",\"status\":"<<quoted(name(r.status))
     <<",\"stage\":"<<quoted(name(r.stage))<<",\"error_code\":"<<int(r.error)<<",\"native_code\":";
    optional(o,r.native_code);o<<",\"failure_stage\":";if(r.failure_stage)o<<quoted(name(*r.failure_stage));else o<<"null";o<<",\"legacy_status\":";optional(o,r.legacy_status);o<<",\"message\":"<<quoted(r.message)<<",\"elapsed_s\":";
    if(std::isfinite(r.elapsed_s)) o<<std::setprecision(12)<<r.elapsed_s;else o<<"null";
    o<<",\"waist_adjusted\":"<<(r.waist_adjusted?"true":"false")<<",\"available_left\":";
    optional(o,r.available_left);o<<",\"available_right\":";optional(o,r.available_right);
    o<<",\"hands\":{";
    for(int h=0;h<2;++h) {
        const auto &x=r.hands[h];if(h) o<<',';
        o<<'"'<<(h?"right":"left")<<"\":{\"requested\":"<<(x.requested?"true":"false")
         <<",\"started\":"<<(x.started?"true":"false")<<",\"action_completed\":"<<(x.action_completed?"true":"false")
         <<",\"retreat_completed\":"<<(x.retreat_completed?"true":"false")
         <<",\"part_state\":"<<quoted(name(x.part))<<",\"evidence\":"<<quoted(name(x.evidence))
         <<",\"stage\":"<<quoted(name(x.stage))<<",\"error_code\":"<<int(x.error)<<",\"native_code\":";
        optional(o,x.native_code);o<<",\"failure_stage\":";if(x.failure_stage)o<<quoted(name(*x.failure_stage));else o<<"null";
        o<<",\"nominal_target\":";
        if(!x.nominal_target) o<<"null";
        else {
            const auto &target=*x.nominal_target;
            o<<"{\"frame\":"<<quoted(target.frame)<<",\"pose_m_rad\":[";
            for(int i=0;i<6;++i) {if(i)o<<',';if(std::isfinite(target.pose[i]))o<<target.pose[i];else o<<"null";}
            o<<"],\"tray_row_aruco\":";optional(o,target.tray_row_aruco);
            o<<",\"tray_column_aruco\":";optional(o,target.tray_column_aruco);o<<'}';
        }o<<'}';
    }o<<"}}";return o.str();
}
namespace detail {
namespace { std::atomic<unsigned> resources_in_use{0};std::atomic<std::uint64_t> ids{0}; }
thread_local Execution *active_execution=nullptr;
Execution::Execution(Skill skill, Metadata meta, Hands hands, Control control, unsigned resources)
    :control_(std::move(control)),resources_(resources) {
    if(meta.request_id.empty()) meta.request_id="skill-"+std::to_string(++ids);
    result.metadata=std::move(meta);result.skill=skill;
    for(int h=0;h<2;++h) result.hands[h].requested=selected(hands,Hand(h));
    if((!hands.left && !hands.right) || resources==0 || (resources&~7u)) {
        fail(Error::InvalidRequest,"no selected hand or invalid resources");return;
    }
    unsigned current=resources_in_use.load();
    do {
        if(current&resources) {fail(Error::Busy,"requested arm/tray resources are busy");return;}
    } while(!resources_in_use.compare_exchange_weak(current,current|resources));
    owns_=true;stage(Stage::Validate,hands);
}
Execution::~Execution() {if(owns_) resources_in_use.fetch_and(~resources_);}
bool Execution::cancelled() {
    if(result.error==Error::Cancelled) return true;
    try {if(!control_.cancelled || !control_.cancelled()) return false;}
    catch(...) {fail(Error::Exception,"cancellation callback threw");return true;}
    fail(Error::Cancelled,"cooperative cancellation requested",-4);return true;
}
void Execution::stage(Stage stage, Hands hands) {
    std::lock_guard<std::recursive_mutex> lock(event_mutex_);
    result.stage=stage;
    for(int h=0;h<2;++h) if(selected(hands,Hand(h))) {
        result.hands[h].stage=stage;
        if(stage==Stage::Approach || stage==Stage::AlignDescend || stage==Stage::Lift || stage==Stage::Return)
            result.hands[h].started=true;
    }
    Progress event{result.metadata,result.skill,stage,hands,++sequence_,
        std::chrono::duration<double>(std::chrono::steady_clock::now()-began_).count()};
    try {if(control_.progress) control_.progress(event);}
    catch(...) {if(result.message.empty()) result.message="progress observer threw; execution continued";}
}
void Execution::fail(Error error, std::string message, std::optional<int> native) {
    std::lock_guard<std::recursive_mutex> lock(event_mutex_);
    if(result.error!=Error::None) return;
    result.error=error;result.failure_stage=result.stage;result.message=std::move(message);if(native) result.native_code=native;
}
void Execution::native(int code, Hands hands, Stage stage) {
    std::lock_guard<std::recursive_mutex> lock(event_mutex_);
    for(int h=0;h<2;++h) if(selected(hands,Hand(h))) {
        auto &out=result.hands[h];if(out.error==Error::None) out.native_code=code;
        if(code!=0 && out.error==Error::None) {out.error=from_native_code(code,stage);out.failure_stage=stage;out.stage=stage;}
    }
    if(code!=0) fail(from_native_code(code,stage),"native phase returned failure",code);
}
Result Execution::finish() {
    if(finished_) return result;
    result.elapsed_s=std::chrono::duration<double>(std::chrono::steady_clock::now()-began_).count();
    if(result.error==Error::Busy || result.error==Error::InvalidRequest) result.status=Status::Rejected;
    else if(result.error==Error::Cancelled) result.status=Status::Cancelled;
    else if(result.error!=Error::None) result.status=Status::Failed;
    else {
        int requested=0,done=0;
        for(const auto &h:result.hands) if(h.requested) {++requested;done+=h.action_completed;}
        result.status=done==requested?Status::Succeeded:done?Status::Partial:Status::NoTarget;
        if(done<requested) result.error=Error::NoTarget;
        if(result.status==Status::Succeeded) {
            Hands hands{result.hands[0].requested,result.hands[1].requested};stage(Stage::Complete,hands);
        }
    }finished_=true;return result;
}
void report(Stage stage, bool right, bool left) {if(active_execution) active_execution->stage(stage,{left,right});}
void report_native(int right_code,int left_code,bool right,bool left,Stage stage) {
    if(!active_execution) return;
    if(right) active_execution->native(right_code,{false,true},stage);
    if(left) active_execution->native(left_code,{true,false},stage);
}
void report_action(bool right, PartState part, Evidence evidence, bool complete) {
    if(!active_execution) return;
    auto &h=active_execution->result.hands[right?1:0];
    h.started=true;h.part=part;h.evidence=evidence;h.action_completed=complete;
}
} // namespace detail
} // namespace t170::skills
