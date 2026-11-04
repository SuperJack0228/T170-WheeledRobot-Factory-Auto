#include "t170/vision/python_bridge.h"
#include "t170/vision/marker.h"

namespace vision
{

BridgeMarkerEngine::BridgeMarkerEngine(SegPoseBridge &bridge) : bridge_(bridge) {}

bool BridgeMarkerEngine::ready_tray() const
{
    return bridge_.tray_engine_ready();
}

bool BridgeMarkerEngine::ready_belt() const
{
    return bridge_.belt_engine_ready();
}

bool BridgeMarkerEngine::ready_aruco() const
{
    return bridge_.aruco_engine_ready();
}

ArucoDetectResult BridgeMarkerEngine::detect_aruco(
    const Frame &frame,
    bool show_visualization,
    CameraSlot vis_slot)
{
    return bridge_.run_aruco(frame, show_visualization, vis_slot);
}

} // namespace vision
