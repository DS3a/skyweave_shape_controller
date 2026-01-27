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
    std::unique_ptr<skyweave::ConstrainedIKSolver> ik_solver_;

    // needs the current state (q, v) : which you can get from state estimator
    // needs the setpoints from gamma surface
    // calculate the required q from IK using gamma surface
    ShapeController(std::shared_ptr<skyweave::StateEstimator> state_estimator,
                    std::shared_ptr<skyweave::controller::GammaSurface> gamma_surface,
                    std::shared_ptr<pinocchio::Model> pin_model)
        : state_estimator_(state_estimator),
          gamma_surface_(gamma_surface),
          pin_data_(*pin_model) {

        this->required_joint_positions_ = pinocchio::neutral(*(this->pin_model_));// neutral;
        this->ik_solver_ = std::make_unique<skyweave::ConstrainedIKSolver>(
            pin_model,
            state_estimator->frame_ids_);
    }

    void compute_required_joint_positions() {
        // get current positions from state estimator
        Eigen::VectorXd current_q = this->state_estimator_->CurrentJointPositions();
        Eigen::VectorXd current_v = this->state_estimator_->CurrentJointVelocities();
        // compute goal positions from gamma surface
        skyweave::PositionMap goal_positions;

        // another option is to set the InitialJointPositions to pinocchio::neutral(*pin_model_)
        // and then calculate dq_ as the pinocchio::difference() between current_q and required_joint_positions_
        // have noticed that the IK solver converges better when starting from neutral instead of current_q

        this->ik_solver_->setInitialJointPositions(current_q);
        goal_positions = this->gamma_surface_->get_goals();
        this->ik_solver_->Solve(goal_positions, 1);

        Eigen::VectorXd v_dot = Eigen::VectorXd::Zero(current_v.size());
        // calulate v_dot as a PD controller of dq_ from ik_solver
        Eigen::VectorXd dq_ = this->ik_solver_->dq_;
        double kp = 10.0;
        double kd = 2.0;
        v_dot = kp * (dq_ - current_v) - kd * current_v;


        // TODO use casadi to formulate an optimization problem
        // we want v_dot in the system dynamics equations to be equal to the above v_dot
        // v and q from state estimator goes into h(q, v)
        // q from state estimator into M(q)
        // spring forces are calculated from skyweave_sim::Springs and applied as generalized torques
        // the thrusts from the thrusters are also applied as generalized torques using jacobians
        // and then we solve for thrusts which make the v_dot equal to the desired v_dot
        

    }

    std::map<skyweave::GridIndex, double> ComputeControlStep() {

    }


};

} // namespace skyweave