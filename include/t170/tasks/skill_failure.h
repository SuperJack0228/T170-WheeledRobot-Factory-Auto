#pragma once
#include "t170/skills/types.h"

namespace t170::tasks {
inline t170::skills::Result configuration_failure(t170::skills::Skill skill,const std::string &message) {
    t170::skills::Result result;result.skill=skill;
    result.status=t170::skills::Status::Rejected;result.error=t170::skills::Error::InvalidRequest;
    result.failure_stage=t170::skills::Stage::Validate;result.native_code=-30;result.message=message;
    return result;
}
// Empty grasp/no target may use the task's existing bounded retry policy.
// Configuration, perception, motion, servo and device failures are terminal.
inline int terminal_skill_failure(const t170::skills::Result &result) {
    using namespace t170::skills;
    if(result.error==Error::None && result.status!=Status::Failed && result.status!=Status::Rejected && result.status!=Status::Cancelled)return 0;
    if(result.error==Error::NoTarget || result.error==Error::EmptyGrasp)return 0;
    if(result.native_code && *result.native_code<0)return *result.native_code;
    switch(result.error) {
        case Error::Cancelled:return -4;
        case Error::Perception:return -3;
        case Error::Servo:return -30;
        case Error::ReleaseUnconfirmed:return -38;
        case Error::Device:return -2;
        default:return -37;
    }
}
}
