#pragma once


#define CHECK_RMSE
#include <Eigen/Core>

#include <pinocchio/parsers/sdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/contact-info.hpp>  // for RigidConstraintModel (depending on version)

#include <inverse_kinematics.hpp>



#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <utility>

namespace skyweave {

using GridIndex = std::pair<int, int>;
using PositionMap = std::map<GridIndex, Eigen::Vector3d>;
using FrameIndexMap = std::map<GridIndex, pinocchio::FrameIndex>;

class StateEstimator {
 public:
  StateEstimator(std::shared_ptr<pinocchio::Model> pin_model, skyweave::FrameIndexMap frame_ids)
      : pin_model_(pin_model),
        joint_positions_(pin_model_->nq),
        joint_velocities_(pin_model_->nv) {
        

    std::cout << "received pinocchio model with nq " << pin_model_->nq 
              << " and nv " << pin_model_->nv << "\n";
    this->frame_ids_ = frame_ids;
    this->pin_data_ = std::make_shared<pinocchio::Data>(*pin_model_);
    // this->joint_positions_.setZero();
    this->joint_velocities_.setZero();

    this->joint_positions_ = pinocchio::neutral(*(this->pin_model_));
    int num_links = this->pin_model_->names.size();
    this->num_elements_x_ = static_cast<int>(std::sqrt(num_links));
    this->num_elements_y_ = this->num_elements_x_;

    this->constraints = skyweave::BuildConstraintFrames(
        this->num_elements_x_, this->num_elements_y_);
  }

  // void SetGoalPositions(PositionMap goals) { goal_positions_ = std::move(goals); }
  void setPositionMeasurements(PositionMap measurements, Eigen::Quaterniond centre_orientation) {
    this->position_measurements = std::move(measurements);
    // rotate all measurements to be in the model frame
    for (auto& [grid_index, position] : this->position_measurements) {
      position = centre_orientation.inverse() * position;
      // position = centre_orientation * position;
    }
  }

  void setRate(double rate) {
    this->rate_ = rate;
    this->time_step_ = 1.0 / rate_;
  }

  void updateEstimations() {

    Eigen::VectorXd prev_pos = this->joint_positions_;

    #ifdef CHECK_RMSE
    double prev_rmse = std::numeric_limits<double>::infinity();

      this->joint_positions_ = pinocchio::neutral(*(this->pin_model_));
      // TODO check the link positions and compare them to the current model state
      pinocchio::forwardKinematics(*(this->pin_model_), *(this->pin_data_), this->joint_positions_);
      pinocchio::updateFramePlacements(*(this->pin_model_), *(this->pin_data_));

      double sum_squared_error = 0.0;
      std::size_t sample_count = 0;
      for (const auto& [grid_index, measurement] : this->position_measurements) {
        auto frame_it = this->frame_ids_.find(grid_index);
        if (frame_it == this->frame_ids_.end()) {
          continue;
        }

        const pinocchio::SE3& frame_placement = this->pin_data_->oMf[frame_it->second];
        const Eigen::Vector3d current_position = frame_placement.translation();
        this->current_positions_[grid_index] = current_position;

        const Eigen::Vector3d diff = current_position - measurement;
        sum_squared_error += diff.squaredNorm();
        ++sample_count;
      }

      const double rmse = sample_count > 0
                              ? std::sqrt(sum_squared_error / static_cast<double>(sample_count))
                              : 0.0;

      prev_rmse = rmse;

      std::cout << "RMSE before IK : " << rmse << "\n";
    
    #endif

    for (int iter=0; iter<3; ++iter) {
      Eigen::VectorXd dq = skyweave::SolveIKStep(
          *(this->pin_model_), 
          *(this->pin_data_),
          this->joint_positions_,
          /* add constraints=*/this->constraints,
          this->position_measurements,
          this->frame_ids_);

      // Eigen::VectorXd next_pos = Eigen::VectorXd::Zero(this->pin_model_->nq);
      this->joint_positions_ = pinocchio::integrate(
          *(this->pin_model_), this->joint_positions_, dq);
 
      // this->joint_positions_ = next_pos;
      
      #ifdef CHECK_RMSE
      // TODO check the link positions and compare them to the current model state
      pinocchio::forwardKinematics(*(this->pin_model_), *(this->pin_data_), this->joint_positions_);
      pinocchio::updateFramePlacements(*(this->pin_model_), *(this->pin_data_));


      double sum_squared_error = 0.0;
      std::size_t sample_count = 0;
      for (const auto& [grid_index, measurement] : this->position_measurements) {
        auto frame_it = this->frame_ids_.find(grid_index);
        if (frame_it == this->frame_ids_.end()) {
          continue;
        }

        const pinocchio::SE3& frame_placement = this->pin_data_->oMf[frame_it->second];
        const Eigen::Vector3d current_position = frame_placement.translation();
        this->current_positions_[grid_index] = current_position;

        const Eigen::Vector3d diff = current_position - measurement;
        sum_squared_error += diff.squaredNorm();
        ++sample_count;
      }

      const double rmse = sample_count > 0
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
    // Here you would implement the state estimation logic using
    // position_measurements_ and updating joint_positions_ and joint_velocities_
    if (!this->first_update_done) {
      // we check for the first update as the velocity is calculated by 
      // checking the difference between q and last_q and dividing by time step
      // in other words, the IK solver's dqs are aggregated and divided by time step
      // to get the velocity estimate
      // for the first iteration, since we don't have a last_q, we can't do this
      // and so, we just set the velocity to zero
      this->joint_velocities_.setZero();
      this->first_update_done = true;
    } else {
      Eigen::VectorXd delta = Eigen::VectorXd::Zero(this->pin_model_->nv);
      pinocchio::difference(
          *(this->pin_model_),
          prev_pos,
          this->joint_positions_,
          delta);

      this->joint_velocities_ = delta / this->time_step_;
      // std::cout << "Estimated mean joint velocities : " << this->joint_velocities_.transpose().mean() << "\n";
    }

  }

  // const PositionMap& GoalPositions() const { return goal_positions_; }

  void UpdateLinkPosition(int x, int y, const Eigen::Vector3d& position) {
    current_positions_[{x, y}] = position;
  }

  const PositionMap& CurrentPositions() const { return current_positions_; }

 private:
  double rate_ = 100.0; // example rate
  double time_step_ = 0.01; // example time step
  std::shared_ptr<pinocchio::Model> pin_model_;
  std::shared_ptr<pinocchio::Data> pin_data_;

  int num_elements_x_;
  int num_elements_y_;

  std::vector<skyweave::ConstraintPair> constraints;

  skyweave::FrameIndexMap frame_ids_;
  Eigen::VectorXd joint_positions_; // needs to have size of pin_model_->nq
  Eigen::VectorXd joint_velocities_; // needs to have size of pin_model_->nv
  PositionMap position_measurements;
  PositionMap current_positions_;
  bool first_update_done = false;
};

}  // namespace skyweave
