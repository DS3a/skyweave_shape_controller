#include <skyweave_sim.hpp>
#include <iostream>

namespace skyweave_sim {

Springs::Springs(const pinocchio::Model& model,
                 const skyweave::FrameIndexMap& frame_ids,
                 const std::map<skyweave::GridIndex, int>& gz_links_idx_map,
                 const std::vector<gazebo::physics::LinkPtr>& links,
                 double k_rot)
    : model_(model),
      data_(model),
      frame_ids_(frame_ids),
      gz_links_idx_map_(gz_links_idx_map),
      links_(links),
      k_rot_(k_rot) {
  this->generalized_spring_forces_ = Eigen::VectorXd::Zero(this->model_.nv);
}

std::map<skyweave::GridIndex, Eigen::Vector3d> Springs::CalculateSpringTorques(
    const Eigen::VectorXd& q_state) {
  pinocchio::forwardKinematics(model_, data_, q_state);
  pinocchio::updateFramePlacements(model_, data_);
  pinocchio::computeJointJacobians(model_, data_, q_state);

  const Eigen::Matrix3d S = Eigen::Matrix3d::Identity() * k_rot_;
  // Eigen::VectorXd tau_stiff = Eigen::VectorXd::Zero(model_.nv);
  this->generalized_spring_forces_.setZero();
  std::map<skyweave::GridIndex, Eigen::Vector3d> link_torques;

  for (const auto& [key, frame_id] : frame_ids_) {
    const Eigen::Matrix3d R_i = data_.oMf[frame_id].rotation();
    Eigen::Vector3d delta_sum = Eigen::Vector3d::Zero();

    for (const auto& neighbor : GetNeighbors(key)) {
      const pinocchio::FrameIndex neighbor_id = frame_ids_.at(neighbor);
      const Eigen::Matrix3d R_j = data_.oMf[neighbor_id].rotation();
      // const Eigen::Vector3d delta = pinocchio::log3(R_i.transpose() * R_j);
      const Eigen::Vector3d delta = pinocchio::log3(R_j.transpose() * R_i);
      delta_sum += S * delta;
    }

    const Eigen::MatrixXd J = pinocchio::getFrameJacobian(
        model_, data_, frame_id, pinocchio::ReferenceFrame::WORLD);
    const Eigen::MatrixXd Jw = J.middleRows(3, 3);
    // tau_stiff += Jw.transpose() * delta_sum;
    this->generalized_spring_forces_ += Jw.transpose() * delta_sum;
    link_torques[key] = delta_sum;
  }
  return link_torques;
}

Eigen::VectorXd Springs::getGeneralizedSpringForces() const {
  return this->generalized_spring_forces_;
}

void Springs::ApplySpringTorques(
    const std::map<skyweave::GridIndex, Eigen::Vector3d>& link_torques) const {
  for (const auto& [key, torque] : link_torques) {
    auto link_it = gz_links_idx_map_.find(key);
    if (link_it == gz_links_idx_map_.end()) {
      continue;
    }
    const int link_index = link_it->second;
    if (link_index < 0 || static_cast<std::size_t>(link_index) >= links_.size()) {
      continue;
    }
    const gazebo::physics::LinkPtr& link = links_[link_index];
    if (!link) {
      continue;
    }
    link->AddForce(
        ignition::math::Vector3d(torque.x(), torque.y(), torque.z()));
  }
}

std::vector<skyweave::GridIndex> Springs::GetNeighbors(
    const skyweave::GridIndex& index) const {
  std::vector<skyweave::GridIndex> neighbors;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      skyweave::GridIndex neighbor{index.first + dx, index.second + dy};
      if (frame_ids_.find(neighbor) != frame_ids_.end()) {
        neighbors.push_back(neighbor);
      }
    }
  }
  return neighbors;
}

}  // namespace skyweave_sim
