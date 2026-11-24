#include "t170/app/task_result.h"
#include "t170/tasks/skill_failure.h"
#include <memory>

namespace t170::app {
Json::Value task_command_result(int code,int completed,const std::optional<t170::skills::Result> &skill,
                               const std::string &error,const char *operation) {
    const bool fatal=skill && t170::tasks::terminal_skill_failure(*skill)!=0;
    const bool ok=code==0 && completed>0 && !fatal;
    Json::Value out(Json::objectValue);
    out["ok"]=ok;out["code"]=code;out["completed_count"]=completed;
    const char *status="failed";
    if(ok)status="succeeded";
    else if(code==-4 || (skill && skill->status==t170::skills::Status::Cancelled))status="cancelled";
    else if(fatal && skill->status==t170::skills::Status::Rejected)status="rejected";
    else if(!fatal && code>=0) {
        if(completed>0)status="partial";
        else if(code==1)status="no_target";
    }
    out["status"]=status;
    if(skill) {
        Json::CharReaderBuilder builder;std::string parse_error;
        const std::string text=t170::skills::to_json(*skill);
        std::unique_ptr<Json::CharReader> reader(builder.newCharReader());Json::Value detail;
        if(reader->parse(text.data(),text.data()+text.size(),&detail,&parse_error))out["skill_result"]=detail;
        if(skill->native_code)out["native_code"]=*skill->native_code;
        if(skill->failure_stage)out["failure_stage"]=t170::skills::name(*skill->failure_stage);
        out["error_code"]=int(skill->error);
    }
    if(!ok) {
        std::string message=error;
        if(message.empty() && fatal) {
            message=std::string(operation)+"失败："+skill->message;
            if(skill->failure_stage)message+="，阶段="+std::string(t170::skills::name(*skill->failure_stage));
            if(skill->native_code)message+="，原始码="+std::to_string(*skill->native_code);
        }
        if(message.empty())message=std::string(operation)+(completed>0?"部分完成，仍有未完成的手臂":code==1?"未执行：工作区没有目标":"失败，未确认动作完成，code="+std::to_string(code));
        out["error"]=message;
    }
    return out;
}
}
