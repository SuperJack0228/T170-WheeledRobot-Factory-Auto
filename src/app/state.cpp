#include "internal/state.h"
#include "t170/config/access.h"
MoveBoxConfig g_move_cfg;
namespace t170::app::detail {
State &state(){static State value;return value;}
std::atomic<bool> &g_abort=state().abort;
std::mutex &g_hw_mu=state().hw_mu;
std::mutex &g_config_mu=t170::config::access_mutex();
t170::calibration::PlacementSession &g_placement_calibration=state().placement_calibration;
orch::HwSession &g_session=state().session;
std::unique_ptr<gripper::Gripper> &g_grip=state().grip;
std::unique_ptr<Arm> &g_arm_r=state().arm_r;
std::unique_ptr<Arm> &g_arm_l=state().arm_l;
std::unique_ptr<WaistRobot> &g_waist=state().waist;
std::unique_ptr<PosePipeline> &g_pipeline=state().pipeline;
Eigen::Matrix<double, 1, 6> &g_layer3=state().layer3;
}
