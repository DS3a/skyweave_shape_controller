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
    double kp_ = 90.0;
    double kd_ = 50.0;
    double ki_ = 0.1;
    Eigen::VectorXd integral_error_; // size nv

    // needs the current state (q, v) : which you can get from state estimator
    // needs the setpoints from gamma surface
    // calculate the required q from IK using gamma surface
    ShapeController(std::shared_ptr<skyweave::StateEstimator> state_estimator,
                    std::shared_ptr<skyweave::controller::GammaSurface> gamma_surface,
                    std::shared_ptr<pinocchio::Model> pin_model)
        : state_estimator_(state_estimator),
          gamma_surface_(gamma_surface),
          pin_model_(pin_model),
          pin_data_(*pin_model) {

        this->required_joint_positions_ = pinocchio::neutral(*(this->pin_model_));// neutral;
        this->desired_joint_acceleration_ = Eigen::VectorXd::Zero(this->pin_model_->nv);
        this->integral_error_ = Eigen::VectorXd::Zero(this->pin_model_->nv);
        this->spring_torques_ = Eigen::VectorXd::Zero(this->pin_model_->nv);
        this->ik_solver_ = std::make_unique<skyweave::ConstrainedIKSolver>(
            pin_model,
            state_estimator->FrameIds());
    }

    void setSpringTorques(const Eigen::VectorXd& spring_torques) {
        if (spring_torques.size() != this->pin_model_->nv) {
            throw std::invalid_argument("Spring torques dimension does not match model nv.");
        }
        this->spring_torques_ = spring_torques;
    }

    void ComputeRequiredJointPosAndAccel() {
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
        this->ik_solver_->Solve(goal_positions, 3);

        Eigen::VectorXd v_dot = Eigen::VectorXd::Zero(current_v.size());
        // calulate v_dot as a PD controller of dq_ from ik_solver
        Eigen::VectorXd dq_ = pinocchio::difference(*(this->pin_model_), current_q, this->ik_solver_->CurrentJointPositions());
        this->integral_error_ += dq_ * 0.02; // assuming control step is 0.02s
        v_dot = kp_ * (dq_) + kd_ * current_v + ki_ * this->integral_error_;


        this->desired_joint_acceleration_ = v_dot;

    }

    std::map<skyweave::GridIndex, double> ComputeControlStep() {
        this->ComputeRequiredJointPosAndAccel();
        // TODO this seems underwhelming, maybe i should run more IK iterations, and take acceleration as the difference
        // std::cout << "computed desired joint acceleration: " << this->desired_joint_acceleration_.transpose() << "\n";

        Eigen::VectorXd current_q = this->state_estimator_->CurrentJointPositions();
        Eigen::VectorXd current_v = this->state_estimator_->CurrentJointVelocities();

        pinocchio::crba(*(this->pin_model_), this->pin_data_, current_q);
        Eigen::MatrixXd M = this->pin_data_.M;
        M.triangularView<Eigen::StrictlyLower>() =
            M.transpose().triangularView<Eigen::StrictlyLower>();

        pinocchio::nonLinearEffects(*(this->pin_model_), this->pin_data_, current_q, current_v);
        Eigen::VectorXd h = this->pin_data_.nle;

        if (this->spring_torques_.size() != this->pin_model_->nv) {
            this->spring_torques_ = Eigen::VectorXd::Zero(this->pin_model_->nv);
        }

        pinocchio::forwardKinematics(*(this->pin_model_), this->pin_data_, current_q, current_v);
        pinocchio::updateFramePlacements(*(this->pin_model_), this->pin_data_);
        pinocchio::computeJointJacobians(*(this->pin_model_), this->pin_data_, current_q);

        const auto& frame_ids = this->state_estimator_->FrameIds();
        std::vector<skyweave::GridIndex> ordered_indices;
        ordered_indices.reserve(frame_ids.size());

        const int nv = this->pin_model_->nv;
        const int num_thrusters = static_cast<int>(frame_ids.size());
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nv, num_thrusters);


        int col = 0;
        for (const auto& [index, frame_id] : frame_ids) {
            ordered_indices.push_back(index);
            const Eigen::MatrixXd J = pinocchio::getFrameJacobian(
                *(this->pin_model_), this->pin_data_, frame_id, pinocchio::ReferenceFrame::LOCAL);
            // const Eigen::MatrixXd J_trans = J.topRows(3);
            // const Eigen::Matrix3d R = this->pin_data_.oMf[frame_id].rotation();
            // const Eigen::Vector3d force_world = R * Eigen::Vector3d(0.0, 0.0, 1.0);
            // A.col(col) = J_trans.transpose() * Eigen::Vector3d(0.0, 0.0, 1.0);
            A.col(col) = J.row(2).transpose(); // only the z thrust
            // std::cout << "the jacobian is " << J_trans << "\n";
            ++col;

        }


        const Eigen::VectorXd desired_tau = M * this->desired_joint_acceleration_ + h - this->spring_torques_;
        // std::cout << "the desired tau is: " << desired_tau.transpose() << "\n";

        casadi::DM A_dm = casadi::DM::zeros(nv, num_thrusters);
        for (int r = 0; r < nv; ++r) {
            for (int c = 0; c < num_thrusters; ++c) {
                A_dm(r, c) = A(r, c);
            }
        }

        casadi::DM b_dm = casadi::DM::zeros(nv, 1);
        for (int r = 0; r < nv; ++r) {
            b_dm(r) = desired_tau(r);
        }

        casadi::MX u = casadi::MX::sym("u", num_thrusters);
        // std::cout << "try to multiply A with u\n";

        // std::cout << "A size: " << A_dm.size1() << " x " << A_dm.size2() << "\n";
        // std::cout << "u size: " << u.size1() << " x " << u.size2() << "\n";
        // std::cout << "b size: " << b_dm.size1() << " x " << b_dm.size2() << "\n";

        // casadi::MX residual = A_dm * u - b_dm;
        casadi::MX residual = mtimes(A_dm, u) - b_dm;
        std::cout << "computed residual\n";
        // casadi::MX residual = casadi::mtimes(A_dm, u) - b_dm;
        // casadi::MX objective = casadi::sumsqr(residual, residual);
        casadi::MX objective = 0.5 * sumsqr(residual);

        // std::cout << objective << "\n";
        
        casadi::MXDict nlp;
        nlp["x"] = u;
        nlp["f"] = objective;
        // {{"x", u}, {"f", objective}};
        casadi::MXDict opts;
        opts["print_time"] = false;
        opts["ipopt.print_level"] = 0;

        casadi::Function solver = casadi::nlpsol(std::string("shape_thrust_solver"), std::string("ipopt"), nlp);

        // casadi::DM lbx = -casadi::DM::ones(num_thrusters, 1) *
        //                  std::numeric_limits<double>::infinity();
        // casadi::DM ubx = casadi::DM::ones(num_thrusters, 1) *
        //                  std::numeric_limits<double>::infinity();
        casadi::DMDict solution = solver(casadi::DMDict{{"x0", casadi::DM::zeros(num_thrusters, 1)}});

        casadi::DM u_opt = solution.at("x");
        std::map<skyweave::GridIndex, double> thrusts;
        for (int i = 0; i < num_thrusters; ++i) {
            thrusts[ordered_indices[static_cast<std::size_t>(i)]] = static_cast<double>(u_opt(i));
        }

        return thrusts;
    }


};

} // namespace controller
} // namespace skyweave
