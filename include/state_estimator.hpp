#pragma once

#include <Eigen/Core>

#include <pinocchio/parsers/sdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/contact-info.hpp>  // for RigidConstraintModel (depending on version)

#include <inverse_kinematics.hpp>

#include <map>
#include <utility>

namespace skyweave {

using GridIndex = std::pair<int, int>;
using PositionMap = std::map<GridIndex, Eigen::Vector3d>;

class StateEstimator {
 public:
  StateEstimator(std::shared_ptr<pinocchio::Model> pin_model, double rate = 100.0)
      : pin_model_(pin_model),
        joint_positions_(pin_model_->nq),
        joint_velocities_(pin_model_->nv) {
        
    this->rate_ = rate;
    this->time_step_ = 1.0 / rate_;
    this->pin_data_ = std::make_shared<pinocchio::Data>(*pin_model_);
    this->joint_positions_.setZero();
    this->joint_velocities_.setZero();

  }

  // void SetGoalPositions(PositionMap goals) { goal_positions_ = std::move(goals); }
  void setPositionMeasurements(PositionMap measurements) {
    this->position_measurements = std::move(measurements);
  }

  void updateEstimations() {
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
  Eigen::VectorXd joint_positions_; // needs to have size of pin_model_->nq
  Eigen::VectorXd joint_velocities_; // needs to have size of pin_model_->nv
  PositionMap position_measurements;
  PositionMap current_positions_;
  bool first_update_done = false;
};

}  // namespace skyweave
