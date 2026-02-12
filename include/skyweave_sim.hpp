
// # update placements at the current state calculated by IEKF state estimtor in state_estimator.hpp
// pin.forwardKinematics(model, data, q_state)
// pin.updateFramePlacements(model, data)

// # neighbor list (max 8 neighbors on the grid)
// def get_neighbors(index):
//     i, j = index
//     neighbors = []
//     for dx in (-1, 0, 1):
//         for dy in (-1, 0, 1):
//             if dx == 0 and dy == 0:
//                 continue
//             neighbor = (i + dx, j + dy)
//             if neighbor in frame_ids:
//                 neighbors.append(neighbor)
//     return neighbors

// # stiffness matrix in tangent space (diagonal)
// k_rot = 0.5
// S = np.diag([k_rot, k_rot, k_rot])

// frame_keys = list(frame_ids.keys())
// num_frames = len(frame_keys)

// tau_stiff = np.zeros(model.nv)

// for idx, key in enumerate(frame_keys):
//     frame_id = frame_ids[key]
//     R_i = data.oMf[frame_id].rotation

//     # sum neighbor orientation errors in tangent space
//     delta_sum = np.zeros(3)
//     for neighbor in get_neighbors(key):
//         neighbor_id = frame_ids[neighbor]
//         R_j = data.oMf[neighbor_id].rotation
//         # quaternion log difference (SO(3) log map)
//         delta = pin.log3(R_j.T @ R_i)
//         delta_sum += S @ delta

//     J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.WORLD)
//     Jw = J[3:6, :]

//     # stiffness contribution (maps to generalized forces)
//     tau_stiff += Jw.T @ delta_sum
// # then apply the tau stiffnesses to each link in the model

#pragma once

#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <Eigen/Core>

#include <state_estimator.hpp>

#include <map>
#include <utility>
#include <vector>

namespace skyweave_sim {
// needs gz_links_idx_map std::map<skyweave::GridIndex, int>
// also needs std::vector<physics::LinkPtr> links
// needs the state from the IEKF state estimator q_state
// needs the number of elements per side
// needs the keys in skyweave::FrameIndexMap frame_ids from skyweave_link_tracker.cpp
class Springs {
 public:
  Springs(const pinocchio::Model& model,
          const skyweave::FrameIndexMap& frame_ids,
          const std::map<skyweave::GridIndex, int>& gz_links_idx_map,
          const std::vector<gazebo::physics::LinkPtr>& links,
          double k_rot = 0.00005);

  std::map<skyweave::GridIndex, Eigen::Vector3d> CalculateSpringTorques(
      const Eigen::VectorXd& q_state,
      const Eigen::VectorXd& q_velocities);

  Eigen::VectorXd getGeneralizedSpringForces() const;

  void ApplySpringTorques(
      const std::map<skyweave::GridIndex, Eigen::Vector3d>& link_torques) const;

 private:
  std::vector<skyweave::GridIndex> GetNeighbors(
      const skyweave::GridIndex& index) const;

  const pinocchio::Model& model_;
  pinocchio::Data data_;

  pinocchio::Data data_neutral;
  const skyweave::FrameIndexMap& frame_ids_;
  const std::map<skyweave::GridIndex, int>& gz_links_idx_map_;
  const std::vector<gazebo::physics::LinkPtr>& links_;
  double k_rot_;
  Eigen::VectorXd generalized_spring_forces_;
};
}  // namespace skyweave_sim
