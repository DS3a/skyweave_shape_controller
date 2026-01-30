#include <gamma_surface.hpp>
#include <state_estimator.hpp>
#include <inverse_kinematics.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <Eigen/Core>

#include <casadi/casadi.hpp>

#include <map>
#include <memory>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace skyweave {
namespace controller {

using namespace casadi;
class ShapeController {
public:

    std::shared_ptr<skyweave::StateEstimator> state_estimator_;
    std::shared_ptr<skyweave::controller::GammaSurface> gamma_surface_;
    std::shared_ptr<pinocchio::Model> pin_model_;
    pinocchio::Data pin_data_;
    Eigen::VectorXd required_joint_positions_; // size nq
    Eigen::VectorXd desired_joint_acceleration_;
    Eigen::VectorXd spring_torques_;
    std::unique_ptr<skyweave::ConstrainedIKSolver> ik_solver_;
    double kp_ = 50.0;
    double kd_ = 12.0;
    double ki_ = 0.0;
    Eigen::VectorXd integral_error_; // size nv

    // needs the current state (q, v) : which you can get from state estimator
    // needs the setpoints from gamma surface
    // calculate the required q from IK using gamma surface
    ShapeController(std::shared_ptr<skyweave::StateEstimator> state_estimator,
                    std::shared_ptr<skyweave::controller::GammaSurface> gamma_surface,
                    std::shared_ptr<pinocchio::Model> pin_model);

    void setSpringTorques(const Eigen::VectorXd& spring_torques);
    void ComputeRequiredJointPosAndAccel();
    std::map<skyweave::GridIndex, double> ComputeControlStep();


};

} // namespace controller
} // namespace skyweave
