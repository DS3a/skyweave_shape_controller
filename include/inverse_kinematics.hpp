#pragma once

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>

#include <Eigen/Core>

#include <memory>
#include <map>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace skyweave {

using GridIndex = std::pair<int, int>;
using FrameIndexMap = std::map<GridIndex, pinocchio::FrameIndex>;
using PositionMap = std::map<GridIndex, Eigen::Vector3d>;

struct ConstraintPair {
  GridIndex first;
  GridIndex second;
};

std::vector<ConstraintPair> BuildConstraintFrames(int num_elements_x,
                                                  int num_elements_y);

Eigen::RowVectorXd GetConstraintJacobian(const pinocchio::Model& model,
                                         pinocchio::Data& data,
                                         const Eigen::VectorXd& q,
                                         pinocchio::FrameIndex frame_id1,
                                         pinocchio::FrameIndex frame_id2);

Eigen::RowVectorXd GetConstraintJacobianForPair(
    const pinocchio::Model& model, pinocchio::Data& data,
    const Eigen::VectorXd& q, const ConstraintPair& pair,
    const FrameIndexMap& frame_ids);

Eigen::MatrixXd GetConstraintJacobianMatrix(
    const pinocchio::Model& model, pinocchio::Data& data,
    const Eigen::VectorXd& q, const std::vector<ConstraintPair>& constraints,
    const FrameIndexMap& frame_ids);

Eigen::MatrixXd GetTaskJacobians(const pinocchio::Model& model,
                                 pinocchio::Data& data,
                                 const Eigen::VectorXd& q,
                                 const PositionMap& goal_positions,
                                 const FrameIndexMap& frame_ids);

std::pair<Eigen::MatrixXd, Eigen::VectorXd> GetTaskJacobiansWithErrors(
    const pinocchio::Model& model, pinocchio::Data& data,
    const Eigen::VectorXd& q,
    const std::map<GridIndex, Eigen::Vector3d>& goal_positions,
    const FrameIndexMap& frame_ids);

Eigen::MatrixXd MakeKKTMatrix(const Eigen::MatrixXd& Jc,
                              const Eigen::MatrixXd& Jt, double reg = 1e-6);

Eigen::VectorXd SolveIKStep(const pinocchio::Model& model,
                            pinocchio::Data& data, const Eigen::VectorXd& q,
                            const std::vector<ConstraintPair>& constraints,
                            const PositionMap& goal_positions,
                            const FrameIndexMap& frame_ids,
                            double reg = 1e-6);

class ConstrainedIKSolver {
 public:
  ConstrainedIKSolver(std::shared_ptr<pinocchio::Model> pin_model,
                      const FrameIndexMap& frame_ids);

  void setInitialJointPositions(const Eigen::VectorXd& q);
  void Solve(const PositionMap& goals, int iterations);
  Eigen::VectorXd CurrentJointPositions() const;

  std::shared_ptr<pinocchio::Model> pin_model_;
  std::shared_ptr<pinocchio::Data> pin_data_;
  int num_elements_x_ = 0;
  int num_elements_y_ = 0;
  std::vector<ConstraintPair> constraints_;
  FrameIndexMap frame_ids_;
  PositionMap goal_positions_;
  Eigen::VectorXd joint_positions_;
  Eigen::VectorXd dq_;
};

}  // namespace skyweave
