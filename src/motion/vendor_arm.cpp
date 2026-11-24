#include "t170/motion/arm.h"
#include "Ti5_Robotics.h"
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <mutex>

namespace {
class VendorArm final : public t170::motion::ArmBackend {
    Ti5_Robotics sdk_;
    mutable std::mutex mutex_;
public:
    explicit VendorArm(const t170::motion::ArmDefinition &d):sdk_("T7_170",d.side) {
        // The SDK currently loads cwd/config/Robot_Model.yaml. Verify the
        // in-memory parameters; a correct-looking external snapshot is not enough.
        if(sdk_.DH_Para().rows()!=7 || sdk_.DH_Para().cols()!=4 || !sdk_.DH_Para().allFinite() ||
           (sdk_.DH_Para()-d.mdh).cwiseAbs().maxCoeff()>1e-12 ||
           sdk_.Tool().size()!=6 || !sdk_.Tool().allFinite() || (sdk_.Tool()-d.tool).cwiseAbs().maxCoeff()>1e-12)
            throw std::runtime_error("Vendor SDK loaded different DH/tool parameters than selected deployment");
    }
    Eigen::Matrix4d forward(const t170::motion::ArmJoints &q) const override {std::lock_guard lock(mutex_);return sdk_.Forward_Kinematics(Eigen::RowVectorXd(q));}
    Eigen::Matrix4d flange(const t170::motion::ArmJoints &q) const override {std::lock_guard lock(mutex_);return sdk_.Flange_Transform(Eigen::RowVectorXd(q));}
    int inverse(const Eigen::Matrix4d &p,t170::motion::ArmJoints &q,std::optional<double> j2) override {
        std::lock_guard lock(mutex_);
        Eigen::RowVectorXd work=q;const int rc=sdk_.Inverse_Kinematics(p,work,j2);if(rc==0)q=work;return rc;
    }
};
}
Arm::Arm(const std::string &sdk_model,const std::string &robot_model,const std::string &side,const std::string &file)
    :definition_(t170::motion::arm_definition(side=="right"?1:0)) {
    if(side!="right" && side!="left")throw std::runtime_error("Invalid arm side");
    if(sdk_model!="T7_170" || robot_model!=definition_.model_id)throw std::runtime_error("Arm model selection mismatch");
    if(!definition_.home.allFinite())throw std::runtime_error("Missing model motion policy/home joints; regenerate runtime");
    if(!std::filesystem::equivalent(std::filesystem::path(file),std::filesystem::current_path()/"config/Robot_Model.yaml"))
        throw std::runtime_error("Start from the resolved deployment directory; SDK configuration is relative to cwd");
    if(!std::filesystem::equivalent(std::filesystem::path(file),definition_.config_path))
        throw std::runtime_error("Arm IO and solver selected different configuration files");
    can_id_list=definition_.can_ids;backend_=std::make_shared<VendorArm>(definition_);
    std::clog<<"[arm_sdk] backend=Ti5_Robotics version=20261109 model="<<robot_model<<" side="<<side
        <<" joints=sdk_model_rad dh_tool_readback=matched line_solver=bounded_qp servo_solver=velocity_qp posture=outward_elbow_wrist\n";
}
