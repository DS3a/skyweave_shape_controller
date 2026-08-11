#include <skyweave_controller_linearized.hpp>

namespace skyweave::controller {


ShapeControllerLinearized::ShapeControllerLinearized(
    std::shared_ptr<skyweave::StateEstimator> state_estimator,
    std::shared_ptr<skyweave::controller::GammaSurface> gamma_surface,
    std::shared_ptr<pinocchio::Model> pin_model)
    : state_estimator_(std::move(state_estimator)),
      gamma_surface_(std::move(gamma_surface)),
      pin_model_(std::move(pin_model)),
      pin_data_(*pin_model_) {
  this->required_joint_positions_ =
      pinocchio::neutral(*(this->pin_model_));  // neutral;
  this->desired_joint_acceleration_ =
      Eigen::VectorXd::Zero(this->pin_model_->nv);
  this->integral_error_ = Eigen::VectorXd::Zero(this->pin_model_->nv);
  this->spring_torques_ = Eigen::VectorXd::Zero(this->pin_model_->nv);
  this->ik_solver_ = std::make_unique<skyweave::ConstrainedIKSolver>(
      pin_model_, state_estimator_->FrameIds());

  this->gamma_surface_->init_ik_solver(
      pin_model_, state_estimator_->FrameIds());
  this->constraints_ =
      skyweave::BuildConstraintFrames(
          this->gamma_surface_->num_elements_,
          this->gamma_surface_->num_elements_);
  this->previous_thrusts_ =
      Eigen::VectorXd::Zero(static_cast<int>(state_estimator_->FrameIds().size()));
  this->num_thrusters_ =
      static_cast<int>(state_estimator_->FrameIds().size());

  for (auto constraint : this->constraints_) {
    std::cout << "Constraint between (" << constraint.first.first << ", "
              << constraint.first.second << ") and ("
              << constraint.second.first << ", " << constraint.second.second
              << ")\n";
  }

  this->ordered_indices.reserve(state_estimator_->FrameIds().size());
  for (const auto& [index, frame_id] : state_estimator_->FrameIds()) {
    this->ordered_indices.push_back(index);
  }
}

void linearize_model() {

    int nv = this->pin_model_->nv;

    this->q_c = this->state_estimator_->CurrentJointPositions();
    this->v_c = this->state_estimator_->CurrentJointVelocities();
    this->u_c = Eigen::VectorXd::Zero(nv); // TODO get the last output from the thingi. or get the open loop controllers input for the first time
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(nv);// TODO spring torques + fext passed through jacobians


    /*
        We are linearizing this system in the perturbation space of x and u, not in x and u.
        As such, we don't take the derivatives of f(x, u). Rather, we are taking the derivative of f_1(deltax, deltau) = f(x circle+ deltax, u circle+ deltau)

        x contains (q, v), and so deltax contains (deltaq, deltav). an important note is that deltaq is in the tangent space/perturbation space of the specific q.

        the partial derivative of f_1(deltax, deltau) with respect to deltav is I.
        this is because \dot {(delta q)} = v + delta v (when delta q is zero, i.e., we are linearizing around the reference point from which delta q deviates)
        they are already in the tangent space, so it is just a simple vector addition, and therefore the partial derivative is just I.



    */

    Eigen::MatrixXd dq_dot_dv = Eigen::Matrix::Identity(nv, nv); // N(q) (also wrong, check above)
    // Avq being the identity matrix is correct.
    
    
    Eigen::MatrixXd dq_dot_dq = Eigen::MatrixXd::Zero(nv, nv); // d(N(q)v)/dq (this is wrong, check the explanation above)
    // N(q) and N(q) v are a little difficult to calculate
    // TODO check this. this isn't correct because q contains quaternions.
    // q_dot = N(q)v
    // if q_dot = v, then the derivative with respect to q and v would have been 0 and I respectively
    // but now they are v dN(q)/dq and N(q) respectively

    Eigen::MatrixXd ddv_dq = Eigen::MatrixXd(nv, nv);
    Eigen::MatrixXd ddv_dv = Eigen::MatrixXd(nv, nv);
    pinocchio::computeABADerivatives(this->pin_model_, this->pin_data_, q_c, q_v, tau);
    ddv_dq = this->pin_data_->ddq_dq;
    ddv_dv = this->pin_data_->ddq_dv;

    this->Afx = Eigen::MatrixXd(2*nv, 2*nv);
    this->Afx.topLeftCorner(nv, nv) = dq_dot_dq;
    this->Afx.topRightCorner(nv, nv) = dq_dot_dv;
    this->Afx.bottomLeftCorner(nv, nv) = ddv_dq;
    this->Afx.bottomRightCorner(nv, nv) = ddv_dv;


}


}