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
  data_neutral = pinocchio::Data(model_);
  pinocchio::forwardKinematics(model_, data_neutral, pinocchio::neutral(model_));
  pinocchio::updateFramePlacements(model_, data_neutral);
}

std::map<skyweave::GridIndex, Eigen::Vector3d> Springs::CalculateSpringTorques(
    const Eigen::VectorXd& q_state,
    const Eigen::VectorXd& q_velocities) {
  pinocchio::forwardKinematics(model_, data_, q_state, q_velocities);
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
      const Eigen::Vector3d p_j_neutral = data_neutral.oMf[neighbor_id].translation();
      const Eigen::Vector3d p_i_neutral = data_neutral.oMf[frame_id].translation();

      const Eigen::Vector3d p_j_current = data_.oMf[neighbor_id].translation();
      const Eigen::Vector3d p_i_current = data_.oMf[frame_id].translation();

      // get the displacement vector from i to j
      Eigen::Vector3d a_neutral = p_j_neutral - p_i_neutral;
      Eigen::Vector3d a_current = p_j_current - p_i_current;

      Eigen::Vector3d delta = a_neutral.normalized().cross(a_current.normalized());
      Eigen::Vector3d restoring_force = k_rot_ * delta.cross(a_neutral / 2);

      // get frame rotational velocities for damping
      pinocchio::Motion i_vel = pinocchio::getFrameVelocity(model_, data_, frame_id, pinocchio::ReferenceFrame::WORLD);
      pinocchio::Motion j_vel = pinocchio::getFrameVelocity(model_, data_, neighbor_id, pinocchio::ReferenceFrame::WORLD);

      Eigen::Vector3d i_angular_vel = i_vel.linear();
      Eigen::Vector3d j_angular_vel = j_vel.linear();
      Eigen::Vector3d relative_angular_vel = j_angular_vel - i_angular_vel;
      Eigen::Vector3d damping_force = -0.01 * a_current.cross(relative_angular_vel); // damping coefficient is 0.001, can be tuned

      // std::cout << "the restoring force for link (" << key.first << ", " << key.second << ") from neighbor ("
      //           << neighbor.first << ", " << neighbor.second << ") is: "
      //           << restoring_force.transpose() << "\n";

      // const Eigen::Vector3d delta = pinocchio::log3(R_j.transpose() * R_i);
      // delta is the rotation from the neighbour to the current frame
      // the delta is used to calculate the torque that the imaginary 3D torque spring will apply on frame i from j
      // this torque is proportional to the rotation difference
      
      delta_sum += restoring_force;
      // delta_sum += damping_force;
      // also add the force to make sure the links don't stretch, the force is proportional to the difference in the distance between the current and neutral positions of the links
      double stretch = a_neutral.norm() - a_current.norm();
      // std::cout << "the stretch for link (" << key.first << ", " << key.second << ") from neighbor ("
      //           << neighbor.first << ", " << neighbor.second << ") is: "
      //           << stretch << "\n";
      
      // apply stretching force only if the neighbour is in the diagonal
      if (std::abs(neighbor.first - key.first) == 1 && std::abs(neighbor.second - key.second) == 1) {
         delta_sum += -1 * stretch * a_current.normalized();
      }

    }

    const Eigen::MatrixXd J = pinocchio::getFrameJacobian(
        model_, data_, frame_id, pinocchio::ReferenceFrame::WORLD);
    // we want to get the translational part of the Jacobian, which maps the forces on the frame to the generalized forces
    Eigen::MatrixXd Jt = J.topRows(3);
    const Eigen::MatrixXd Jw = J.middleRows(3, 3);
    // tau_stiff += Jw.transpose() * delta_sum;
    this->generalized_spring_forces_ += Jt.transpose() * delta_sum;
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
