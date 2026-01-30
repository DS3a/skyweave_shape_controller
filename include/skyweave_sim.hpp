#pragma once

#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <Eigen/Core>

#include <state_estimator.hpp>

#include <map>
#include <utility>
#include <vector>

namespace skyweave_sim {

class Springs {
 public:
  Springs(const pinocchio::Model& model,
          const skyweave::FrameIndexMap& frame_ids,
          const std::map<skyweave::GridIndex, int>& gz_links_idx_map,
          const std::vector<gazebo::physics::LinkPtr>& links,
          double k_rot = 0.00005);

  std::map<skyweave::GridIndex, Eigen::Vector3d> CalculateSpringTorques(
      const Eigen::VectorXd& q_state);

  Eigen::VectorXd getGeneralizedSpringForces() const;

  void ApplySpringTorques(
      const std::map<skyweave::GridIndex, Eigen::Vector3d>& link_torques) const;

 private:
  std::vector<skyweave::GridIndex> GetNeighbors(
      const skyweave::GridIndex& index) const;

  const pinocchio::Model& model_;
  pinocchio::Data data_;
  const skyweave::FrameIndexMap& frame_ids_;
  const std::map<skyweave::GridIndex, int>& gz_links_idx_map_;
  const std::vector<gazebo::physics::LinkPtr>& links_;
  double k_rot_;
  Eigen::VectorXd generalized_spring_forces_;
};

}  // namespace skyweave_sim
