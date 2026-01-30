#include <state_estimator.hpp>

namespace skyweave {

StateEstimator::StateEstimator(std::shared_ptr<pinocchio::Model> pin_model,
                               skyweave::FrameIndexMap frame_ids)
    : pin_model_(pin_model),
      joint_positions_(pin_model_->nq),
      joint_velocities_(pin_model_->nv) {
  std::cout << "received pinocchio model with nq " << pin_model_->nq
            << " and nv " << pin_model_->nv << "\n";
  this->frame_ids_ = frame_ids;
  this->pin_data_ = std::make_shared<pinocchio::Data>(*pin_model_);
  this->joint_velocities_.setZero();

  this->joint_positions_ = pinocchio::neutral(*(this->pin_model_));
  int num_links = this->pin_model_->names.size();
  this->num_elements_x_ = static_cast<int>(std::sqrt(num_links));
  this->num_elements_y_ = this->num_elements_x_;

  this->constraints =
      skyweave::BuildConstraintFrames(this->num_elements_x_,
                                      this->num_elements_y_);
}

void StateEstimator::setPositionMeasurements(
    PositionMap measurements, Eigen::Quaterniond centre_orientation) {
  this->position_measurements = std::move(measurements);
  for (auto& [grid_index, position] : this->position_measurements) {
    position = centre_orientation.inverse() * position;
  }
}

void StateEstimator::setRate(double rate) {
  this->rate_ = rate;
  this->time_step_ = 1.0 / rate_;
}

void StateEstimator::updateEstimations() {
  Eigen::VectorXd prev_pos = this->joint_positions_;

#ifdef CHECK_RMSE
  double prev_rmse = std::numeric_limits<double>::infinity();

  this->joint_positions_ = pinocchio::neutral(*(this->pin_model_));
  pinocchio::forwardKinematics(*(this->pin_model_), *(this->pin_data_),
                               this->joint_positions_);
  pinocchio::updateFramePlacements(*(this->pin_model_), *(this->pin_data_));

  double sum_squared_error = 0.0;
  std::size_t sample_count = 0;
  for (const auto& [grid_index, measurement] : this->position_measurements) {
    auto frame_it = this->frame_ids_.find(grid_index);
    if (frame_it == this->frame_ids_.end()) {
      continue;
    }

    const pinocchio::SE3& frame_placement =
        this->pin_data_->oMf[frame_it->second];
    const Eigen::Vector3d current_position = frame_placement.translation();
    this->current_positions_[grid_index] = current_position;

    const Eigen::Vector3d diff = current_position - measurement;
    sum_squared_error += diff.squaredNorm();
    ++sample_count;
  }

  const double rmse =
      sample_count > 0
          ? std::sqrt(sum_squared_error / static_cast<double>(sample_count))
          : 0.0;

  prev_rmse = rmse;

  std::cout << "RMSE before IK : " << rmse << "\n";

#endif

  for (int iter = 0; iter < 3; ++iter) {
    Eigen::VectorXd dq = skyweave::SolveIKStep(
        *(this->pin_model_), *(this->pin_data_), this->joint_positions_,
        this->constraints, this->position_measurements, this->frame_ids_);

    this->joint_positions_ = pinocchio::integrate(
        *(this->pin_model_), this->joint_positions_, dq);

#ifdef CHECK_RMSE
    pinocchio::forwardKinematics(*(this->pin_model_), *(this->pin_data_),
                                 this->joint_positions_);
    pinocchio::updateFramePlacements(*(this->pin_model_), *(this->pin_data_));

    double sum_squared_error = 0.0;
    std::size_t sample_count = 0;
    for (const auto& [grid_index, measurement] : this->position_measurements) {
      auto frame_it = this->frame_ids_.find(grid_index);
      if (frame_it == this->frame_ids_.end()) {
        continue;
      }

      const pinocchio::SE3& frame_placement =
          this->pin_data_->oMf[frame_it->second];
      const Eigen::Vector3d current_position = frame_placement.translation();
      this->current_positions_[grid_index] = current_position;

      const Eigen::Vector3d diff = current_position - measurement;
      sum_squared_error += diff.squaredNorm();
      ++sample_count;
    }

    const double rmse =
        sample_count > 0
            ? std::sqrt(sum_squared_error / static_cast<double>(sample_count))
            : 0.0;
    if (rmse > prev_rmse) {
      std::cout << "RMSE increased from " << prev_rmse << " to " << rmse
                << " after iteration " << iter << ".\n";
    }
    prev_rmse = rmse;

    std::cout << "RMSE after iteration " << iter << ": " << rmse << "\n";

#endif
  }

  if (!this->first_update_done) {
    this->joint_velocities_.setZero();
    this->first_update_done = true;
  } else {
    Eigen::VectorXd delta = Eigen::VectorXd::Zero(this->pin_model_->nv);
    pinocchio::difference(*(this->pin_model_), prev_pos, this->joint_positions_,
                          delta);

    this->joint_velocities_ = delta / this->time_step_;
  }
}

Eigen::VectorXd StateEstimator::CurrentJointPositions() const {
  return joint_positions_;
}

Eigen::VectorXd StateEstimator::CurrentJointVelocities() const {
  return joint_velocities_;
}

const FrameIndexMap& StateEstimator::FrameIds() const { return frame_ids_; }

void StateEstimator::UpdateLinkPosition(int x, int y,
                                        const Eigen::Vector3d& position) {
  current_positions_[{x, y}] = position;
}

const PositionMap& StateEstimator::CurrentPositions() const {
  return current_positions_;
}

}  // namespace skyweave
