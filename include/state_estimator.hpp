#pragma once


// #define CHECK_RMSE
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
#include <memory>
#include <utility>
#include <vector>

namespace skyweave {

using GridIndex = std::pair<int, int>;
using PositionMap = std::map<GridIndex, Eigen::Vector3d>;
using FrameIndexMap = std::map<GridIndex, pinocchio::FrameIndex>;

class StateEstimator {
 public:
  StateEstimator(std::shared_ptr<pinocchio::Model> pin_model,
                 skyweave::FrameIndexMap frame_ids);

  // void SetGoalPositions(PositionMap goals) { goal_positions_ = std::move(goals); }
  void setPositionMeasurements(PositionMap measurements,
                               Eigen::Quaterniond centre_orientation);
  void setRate(double rate);
  void updateEstimations();

  Eigen::VectorXd CurrentJointPositions() const;
  Eigen::VectorXd CurrentJointVelocities() const;
  const FrameIndexMap& FrameIds() const;

  // const PositionMap& GoalPositions() const { return goal_positions_; }

  void UpdateLinkPosition(int x, int y, const Eigen::Vector3d& position);
  const PositionMap& CurrentPositions() const;

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
