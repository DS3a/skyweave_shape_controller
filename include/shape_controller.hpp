#include <gamma_surface.hpp>
#include <state_estimator.hpp>
#include <inverse_kinematics.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>

#include <Eigen/Core>

#include <casadi/casadi.hpp>

#include <map>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace skyweave {

class ShapeController {
public:

    std::shared_ptr<skyweave::StateEstimator> state_estimator_;
    std::shared_ptr<skyweave::controller::GammaSurface> gamma_surface_;
    std::shared_ptr<pinocchio::Model> pin_model_;
    pinocchio::Data pin_data_;
    Eigen::VectorXd required_joint_positions_; // size nq

    // needs the current state (q, v) : which you can get from state estimator
    // needs the setpoints from gamma surface
    // calculate the required q from IK using gamma surface
    ShapeController(std::shared_ptr<skyweave::StateEstimator> state_estimator,
                    std::shared_ptr<skyweave::controller::GammaSurface> gamma_surface,
                    std::shared_ptr<pinocchio::Model> pin_model)
        : state_estimator_(state_estimator),
          gamma_surface_(gamma_surface),
          pin_data_(*pin_model) {

        this->required_joint_positions_ = Eigen::VectorXd::Zero(pin_model->nq);
    }

    void compute_required_joint_positions() {

    }

    std::map<skyweave::GridIndex, double> ComputeControlStep() {

    }


};

} // namespace skyweave