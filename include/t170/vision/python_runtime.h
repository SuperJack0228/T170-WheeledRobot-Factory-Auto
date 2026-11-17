#pragma once
#include <functional>
class PosePipeline;
// Interpreter/GIL details stay beside the Python bridge, outside application/server code.
void with_python_gil_released(const std::function<void()> &);
void shutdown_pose_pipeline(PosePipeline &,bool finalize_python);
