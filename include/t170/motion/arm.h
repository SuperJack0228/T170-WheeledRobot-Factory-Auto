#pragma once
#include <Eigen/Dense>
#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <mutex>
#include <limits>
#include "t170/motion/bounded_ik.h"

namespace t170::motion {
using ArmJoints = Eigen::Matrix<double,1,7>;
struct ArmBounds { ArmJoints minimum, maximum;bool device_read=false; };
struct ArmLimitState { mutable std::mutex mutex;ArmBounds bounds; };
struct ArmDefinition {
    int hand = 1;
    std::string model_id, side, config_path;
    std::vector<int> can_ids;
    ArmJoints motor_direction = ArmJoints::Ones();
    ArmJoints motor_offset = ArmJoints::Zero();
    ArmJoints minimum = ArmJoints::Constant(-3.14), maximum = ArmJoints::Constant(3.14);
    Eigen::Matrix<double,7,4> mdh = Eigen::Matrix<double,7,4>::Zero();
    Eigen::Matrix<double,1,6> tool = Eigen::Matrix<double,1,6>::Zero();
    double period = .005, velocity = .9, acceleration = 2.5, jerk = 20;
    bool ideal_t7 = false;
    ArmJoints home=ArmJoints::Constant(std::numeric_limits<double>::quiet_NaN()), preferred=ArmJoints::Zero();
    double wrist_margin_rad=.14,wrist_gain=2.,elbow_clearance_m=.08;
    double posture_gain=.15, elbow_gain=8, limit_gain=.002, elbow_outward_ratio=.3;
    std::shared_ptr<ArmLimitState> limits;
};
// SDK implementation is private to vendor_arm.cpp. Test backends are explicitly
// injected; the production constructor always creates the vendor backend.
class ArmBackend {
public:
    virtual ~ArmBackend() = default;
    virtual Eigen::Matrix4d forward(const ArmJoints &) const = 0;
    virtual Eigen::Matrix4d flange(const ArmJoints &) const = 0;
    virtual int inverse(const Eigen::Matrix4d &, ArmJoints &, std::optional<double>) = 0;
};
ArmDefinition load_arm_definition(const std::string &model_file,const std::string &side);
const ArmDefinition &arm_definition(int hand);
ArmJoints model_to_device(int hand,const ArmJoints &q);
ArmJoints device_to_model(int hand,const ArmJoints &q);
Eigen::Matrix4d arm_pose_transform(const Eigen::Matrix<double,1,6> &pose);
ArmBounds effective_arm_bounds(const ArmDefinition &);
ArmBounds intersect_device_bounds(const ArmDefinition &,const ArmJoints &device_min,const ArmJoints &device_max);
}

// Application-owned arm adapter. ALL public joint arguments/results are SDK
// MODEL radians, including IK seeds, trajectories, limits and held references.
// Method spellings are retained at existing call sites; this is not the legacy
// vendor class and has no binary/API dependency on Robot_Arm/Ti5_Arm_2204.so.
class Arm {
public:
    Arm(const std::string &sdk_model,const std::string &robot_model,const std::string &side,
        const std::string &model_file = "");
    Arm(t170::motion::ArmDefinition,std::shared_ptr<t170::motion::ArmBackend>);
    Arm(const Arm &) = delete;
    Arm &operator=(const Arm &) = delete;
    std::vector<int> can_id_list;
    double Cart_Linear_Velocity = .10;
    const t170::motion::ArmDefinition &definition() const { return definition_; }
    t170::motion::ArmJoints to_model(const t170::motion::ArmJoints &q) const {
        return (q-definition_.motor_offset).cwiseQuotient(definition_.motor_direction);
    }
    t170::motion::ArmJoints to_device(const t170::motion::ArmJoints &q) const {
        return q.cwiseProduct(definition_.motor_direction)+definition_.motor_offset;
    }
    Eigen::Matrix4d Forward_Kinematics(const Eigen::Ref<const Eigen::RowVectorXd> &q);
    Eigen::Matrix4d Flange_Transform(const Eigen::Ref<const Eigen::RowVectorXd> &q);
    Eigen::MatrixXd Jacobian(const Eigen::Ref<const Eigen::RowVectorXd> &q);
    bool Check_Joint_Limit(const Eigen::Ref<const Eigen::RowVectorXd> &q);
    t170::motion::ArmBounds effective_limits() const { return t170::motion::effective_arm_bounds(definition_); }
    void install_device_limits(const t170::motion::ArmJoints &minimum,const t170::motion::ArmJoints &maximum);
    Eigen::Vector3d elbow_position(const t170::motion::ArmJoints &) const;
    t170::motion::IKPolicy ik_policy(const Eigen::Matrix4d &target) const;
    int Inverse_Kinematics(Eigen::Matrix<double,1,6> &target,Eigen::Ref<Eigen::RowVectorXd> q);
    int Inverse_Kinematics_Numeric(Eigen::Matrix<double,1,6> &target,Eigen::Ref<Eigen::RowVectorXd> q);
    int Inverse_Kinematics_Analytic(const Eigen::Matrix<double,1,6> &target,Eigen::Ref<Eigen::RowVectorXd> q,double j2);
    int Joint_Trajectory(const Eigen::MatrixXd &start,const Eigen::MatrixXd &end,Eigen::MatrixXd &out);
    // Free-space transit: joint interpolation with a checked Cartesian envelope;
    // falls back to a Cartesian line if the joint arc leaves that envelope.
    int Approach_Trajectory(const Eigen::MatrixXd &start,Eigen::Matrix<double,1,6> target,
                            Eigen::MatrixXd &out,double tcp_speed);
    int Approach_Stroke(const Eigen::MatrixXd &start,Eigen::Matrix<double,1,6> guide,
                        const Eigen::Matrix<double,1,6> &finish,Eigen::MatrixXd &out,double speed);
    int Line_Trajectory(const Eigen::MatrixXd &start,Eigen::Matrix<double,1,6> &target,
                        Eigen::MatrixXd &out,std::optional<double> j2=std::nullopt,
                        std::optional<t170::motion::ArmJoints> endpoint=std::nullopt);
private:
    // lateral_margin_m is the XY room outside the endpoint box. Preparation
    // transits use 0.15 m; work strokes keep 0.03 m. The vertical fence is shared.
    int approach_to_joints(const Eigen::MatrixXd &,Eigen::Matrix<double,1,6>,const t170::motion::ArmJoints &,Eigen::MatrixXd &,double,double lateral_margin_m);
    std::vector<t170::motion::ArmJoints> inverse_candidates(const Eigen::Matrix<double,1,6> &,const t170::motion::ArmJoints &);
    double posture_cost(const t170::motion::ArmJoints &,const Eigen::Matrix4d &,const t170::motion::ArmJoints &) const;
    int local_inverse(const Eigen::Matrix4d &,t170::motion::ArmJoints &,std::optional<double>);
    bool valid_solution(const Eigen::Matrix4d &,const t170::motion::ArmJoints &);
    t170::motion::ArmDefinition definition_;
    std::shared_ptr<t170::motion::ArmBackend> backend_;
};
