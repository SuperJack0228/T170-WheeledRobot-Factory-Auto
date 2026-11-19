#include "t170/vision/python_runtime.h"
#include "t170/vision/python_bridge.h"
#include <Python.h>
#include <pybind11/embed.h>
void with_python_gil_released(const std::function<void()> &body) {
    pybind11::gil_scoped_release release;
    body();
}
void shutdown_pose_pipeline(PosePipeline &pipeline,bool finalize_python) {
    if(Py_IsInitialized()) {
        pybind11::gil_scoped_acquire acquire;
        pipeline.shutdown(finalize_python);
    } else pipeline.shutdown(finalize_python);
}
