# One source manifest shared by robot builds and synthetic runtime regression tests.
# Compatibility .cpp paths intentionally contain no implementations and are not built.
set(T170_ARM_MOTION_SOURCES
    src/motion/arm_state.cpp
    src/motion/kinematics_adapter.cpp
    src/motion/feedback.cpp
    src/motion/planning.cpp
    src/motion/trajectory_executor.cpp
    src/motion/cartesian_tracking.cpp
    src/motion/retreat.cpp
    src/motion/servo_handoff.cpp
)
set(T170_SERVO_RUNTIME_SOURCES
    src/motion/servo_state.cpp
    src/motion/servo_feedback.cpp
    src/motion/servo_session.cpp
    src/motion/observation_session.cpp
    src/motion/motion_events.cpp
    src/motion/final_hold.cpp
    src/motion/servo_executor.cpp
)
