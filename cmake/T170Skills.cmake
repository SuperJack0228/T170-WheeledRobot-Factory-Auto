set(T170_SKILLS_SOURCES
    src/skills/common/state.cpp
    src/skills/common/waist_frame.cpp
    src/skills/common/waist_coordination.cpp
    src/skills/common/preparation.cpp
    src/skills/common/gripper_feedback.cpp
    src/skills/common/grasp_finish.cpp
    src/skills/common/lift.cpp
    src/skills/tray/profile_policy.cpp
    src/skills/tray/target_policy.cpp
    src/skills/tray/selection.cpp
    src/skills/tray/perception.cpp
    src/skills/tray/hand_refinement.cpp
    src/skills/tray/approach.cpp
    src/skills/tray/descent.cpp
    src/skills/tray/reach_coordination.cpp
    src/skills/tray/cycle.cpp
    src/skills/tray/service.cpp
    src/skills/conveyor/targets.cpp
    src/skills/conveyor/execution.cpp
)
add_library(t170_skill_contract STATIC src/skills/common/contract.cpp)
add_library(t170::skill_contract ALIAS t170_skill_contract)
target_include_directories(t170_skill_contract PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(t170_skill_contract PUBLIC Threads::Threads)
