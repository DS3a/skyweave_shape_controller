#pragma once

#include <Eigen/Core>

#include <pinocchio/algorithm/contact-info.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/sdf.hpp>

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
  StateEstimator(std::shared_ptr<pinocchio::Model> pin_model,
                 skyweave::FrameIndexMap frame_ids);

  void setPositionMeasurements(PositionMap measurements,
                               Eigen::Quaterniond centre_orientation);

  void setRate(double rate);

  void updateEstimations();

  Eigen::VectorXd CurrentJointPositions() const;
  Eigen::VectorXd CurrentJointVelocities() const;
  const FrameIndexMap& FrameIds() const;

  void UpdateLinkPosition(int x, int y, const Eigen::Vector3d& position);

  const PositionMap& CurrentPositions() const;

 private:
  double rate_ = 100.0;
  double time_step_ = 0.01;
  std::shared_ptr<pinocchio::Model> pin_model_;
  std::shared_ptr<pinocchio::Data> pin_data_;

  int num_elements_x_;
  int num_elements_y_;

  std::vector<skyweave::ConstraintPair> constraints;

  skyweave::FrameIndexMap frame_ids_;
  Eigen::VectorXd joint_positions_;
  Eigen::VectorXd joint_velocities_;
  PositionMap position_measurements;
  PositionMap current_positions_;
  bool first_update_done = false;
};

}  // namespace skyweave
