#include <shape_controller.hpp>

namespace skyweave::controller {

ShapeController::ShapeController(
    std::shared_ptr<skyweave::StateEstimator> state_estimator,
    std::shared_ptr<skyweave::controller::GammaSurface> gamma_surface,
    std::shared_ptr<pinocchio::Model> pin_model)
    : state_estimator_(state_estimator),
      gamma_surface_(gamma_surface),
      pin_model_(pin_model),
      pin_data_(*pin_model) {
  this->required_joint_positions_ =
      pinocchio::neutral(*(this->pin_model_));
  this->desired_joint_acceleration_ =
      Eigen::VectorXd::Zero(this->pin_model_->nv);
  this->integral_error_ = Eigen::VectorXd::Zero(this->pin_model_->nv);
  this->spring_torques_ = Eigen::VectorXd::Zero(this->pin_model_->nv);
  this->ik_solver_ = std::make_unique<skyweave::ConstrainedIKSolver>(
      pin_model, state_estimator->FrameIds());
}

void ShapeController::setSpringTorques(const Eigen::VectorXd& spring_torques) {
  if (spring_torques.size() != this->pin_model_->nv) {
    throw std::invalid_argument(
        "Spring torques dimension does not match model nv.");
  }
  this->spring_torques_ = spring_torques;
}

void ShapeController::ComputeRequiredJointPosAndAccel() {
  Eigen::VectorXd current_q = this->state_estimator_->CurrentJointPositions();
  Eigen::VectorXd current_v = this->state_estimator_->CurrentJointVelocities();
  skyweave::PositionMap goal_positions;

  this->ik_solver_->setInitialJointPositions(current_q);
  goal_positions = this->gamma_surface_->get_goals();
  this->ik_solver_->Solve(goal_positions, 3);

  Eigen::VectorXd v_dot = Eigen::VectorXd::Zero(current_v.size());
  Eigen::VectorXd dq_ =
      pinocchio::difference(*(this->pin_model_), current_q,
                            this->ik_solver_->CurrentJointPositions());
  this->integral_error_ += dq_ * 0.02;
  v_dot = kp_ * (dq_) - kd_ * current_v + ki_ * this->integral_error_;

  this->desired_joint_acceleration_ = v_dot;
}

std::map<skyweave::GridIndex, double> ShapeController::ComputeControlStep() {
  this->ComputeRequiredJointPosAndAccel();

  Eigen::VectorXd current_q = this->state_estimator_->CurrentJointPositions();
  Eigen::VectorXd current_v = this->state_estimator_->CurrentJointVelocities();

  pinocchio::crba(*(this->pin_model_), this->pin_data_, current_q);
  Eigen::MatrixXd M = this->pin_data_.M;
  M.triangularView<Eigen::StrictlyLower>() =
      M.transpose().triangularView<Eigen::StrictlyLower>();

  pinocchio::nonLinearEffects(*(this->pin_model_), this->pin_data_, current_q,
                              current_v);
  Eigen::VectorXd h = this->pin_data_.nle;

  if (this->spring_torques_.size() != this->pin_model_->nv) {
    this->spring_torques_ = Eigen::VectorXd::Zero(this->pin_model_->nv);
  }

  pinocchio::forwardKinematics(*(this->pin_model_), this->pin_data_, current_q,
                               current_v);
  pinocchio::updateFramePlacements(*(this->pin_model_), this->pin_data_);
  pinocchio::computeJointJacobians(*(this->pin_model_), this->pin_data_,
                                   current_q);

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
        *(this->pin_model_), this->pin_data_, frame_id,
        pinocchio::ReferenceFrame::LOCAL);
    A.col(col) = J.row(2).transpose();
    ++col;
  }

  const Eigen::VectorXd desired_tau =
      M * this->desired_joint_acceleration_ + h - this->spring_torques_;

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

  casadi::MX residual = mtimes(A_dm, u) - b_dm;
  std::cout << "computed residual\n";
  casadi::MX objective = 0.5 * sumsqr(residual);

  casadi::MXDict nlp;
  nlp["x"] = u;
  nlp["f"] = objective;
  casadi::MXDict opts;
  opts["print_time"] = false;
  opts["ipopt.print_level"] = 0;

  casadi::Function solver =
      casadi::nlpsol(std::string("shape_thrust_solver"),
                    std::string("ipopt"), nlp);

  casadi::DMDict solution =
      solver(casadi::DMDict{{"x0", casadi::DM::zeros(num_thrusters, 1)}});

  casadi::DM u_opt = solution.at("x");
  std::map<skyweave::GridIndex, double> thrusts;
  for (int i = 0; i < num_thrusters; ++i) {
    thrusts[ordered_indices[static_cast<std::size_t>(i)]] =
        static_cast<double>(u_opt(i));
  }

  return thrusts;
}

}  // namespace skyweave::controller
