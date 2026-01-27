#pragma once

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>

#include <Eigen/Core>

#include <map>
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

inline std::vector<ConstraintPair> BuildConstraintFrames(int num_elements_x,
                                                         int num_elements_y) {
  std::vector<ConstraintPair> constraints;
  const int x_half = num_elements_x / 2;
  const int y_half = num_elements_y / 2;

  constraints.reserve(static_cast<size_t>(num_elements_x) *
                      static_cast<size_t>(num_elements_y));

  for (int i = -x_half; i <= x_half; ++i) {
    for (int j = -y_half; j < y_half; ++j) {
      if (i != 0) {
        constraints.push_back({{i, j}, {i, j + 1}});
      }
    }
  }
  return constraints;
}

inline Eigen::RowVectorXd GetConstraintJacobian(const pinocchio::Model& model,
                                                pinocchio::Data& data,
                                                const Eigen::VectorXd& q,
                                                pinocchio::FrameIndex frame_id1,
                                                pinocchio::FrameIndex frame_id2) {
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::forwardKinematics(model, data, q);

  const Eigen::MatrixXd J1 = pinocchio::getFrameJacobian(
      model, data, frame_id1, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
  const Eigen::MatrixXd J2 = pinocchio::getFrameJacobian(
      model, data, frame_id2, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);

  const Eigen::Vector3d p1 = data.oMf[frame_id1].translation();
  const Eigen::Vector3d p2 = data.oMf[frame_id2].translation();

  const Eigen::RowVector3d d_constraint = 2.0 * (p1 - p2).transpose();
  const Eigen::MatrixXd J_rel = (J2 - J1).topRows<3>();

  return d_constraint * J_rel;
}

inline Eigen::RowVectorXd GetConstraintJacobianForPair(
    const pinocchio::Model& model, pinocchio::Data& data,
    const Eigen::VectorXd& q, const ConstraintPair& pair,
    const FrameIndexMap& frame_ids) {
  const auto frame_id1 = frame_ids.at(pair.first);
  const auto frame_id2 = frame_ids.at(pair.second);
  return GetConstraintJacobian(model, data, q, frame_id1, frame_id2);
}

inline Eigen::MatrixXd GetConstraintJacobianMatrix(
    const pinocchio::Model& model, pinocchio::Data& data,
    const Eigen::VectorXd& q, const std::vector<ConstraintPair>& constraints,
    const FrameIndexMap& frame_ids) {
  if (constraints.empty()) {
    return Eigen::MatrixXd();
  }

  std::vector<Eigen::RowVectorXd> rows;
  rows.reserve(constraints.size());

  for (const auto& pair : constraints) {
    rows.push_back(GetConstraintJacobianForPair(model, data, q, pair, frame_ids));
  }

  const Eigen::Index cols = rows.front().cols();
  Eigen::MatrixXd J(rows.size(), cols);
  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(rows.size()); ++i) {
    J.row(i) = rows[static_cast<size_t>(i)];
  }
  return J;
}

inline Eigen::MatrixXd GetTaskJacobians(const pinocchio::Model& model,
                                        pinocchio::Data& data,
                                        const Eigen::VectorXd& q,
                                        const PositionMap&
                                            goal_positions,
                                        const FrameIndexMap& frame_ids) {
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::forwardKinematics(model, data, q);

  std::vector<Eigen::MatrixXd> rows;
  rows.reserve(frame_ids.size());

  for (const auto& entry : frame_ids) {
    const auto frame_id = entry.second;
    const Eigen::Vector3d frame_pos = data.oMf[frame_id].translation();
    const Eigen::Vector3d goal_pos = goal_positions.at(entry.first);
    (void)frame_pos;
    (void)goal_pos;

    Eigen::MatrixXd J = pinocchio::getFrameJacobian(
        model, data, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
    J = -J.topRows<3>();
    rows.push_back(J);
  }

  if (rows.empty()) {
    return Eigen::MatrixXd();
  }

  const Eigen::Index cols = rows.front().cols();
  Eigen::MatrixXd J(rows.size() * 3, cols);
  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(rows.size()); ++i) {
    J.block(i * 3, 0, 3, cols) = rows[static_cast<size_t>(i)];
  }
  return J;
}

inline std::pair<Eigen::MatrixXd, Eigen::VectorXd> GetTaskJacobiansWithErrors(
    const pinocchio::Model& model, pinocchio::Data& data,
    const Eigen::VectorXd& q,
    const std::map<GridIndex, Eigen::Vector3d>& goal_positions,
    const FrameIndexMap& frame_ids) {
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::forwardKinematics(model, data, q);

  std::vector<Eigen::MatrixXd> rows;
  std::vector<Eigen::Vector3d> errors;
  rows.reserve(frame_ids.size());
  errors.reserve(frame_ids.size());

  for (const auto& entry : frame_ids) {
    const auto frame_id = entry.second;
    const Eigen::Vector3d frame_pos = data.oMf[frame_id].translation();
    const Eigen::Vector3d goal_pos = goal_positions.at(entry.first);
    const Eigen::Vector3d err = goal_pos - frame_pos;

    Eigen::MatrixXd J = pinocchio::getFrameJacobian(
        model, data, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
    J = -J.topRows<3>();
    rows.push_back(J);
    errors.push_back(err);
  }

  Eigen::MatrixXd J;
  if (!rows.empty()) {
    const Eigen::Index cols = rows.front().cols();
    J.resize(rows.size() * 3, cols);
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(rows.size()); ++i) {
      J.block(i * 3, 0, 3, cols) = rows[static_cast<size_t>(i)];
    }
  }

  Eigen::VectorXd error_vec(3 * static_cast<Eigen::Index>(errors.size()));
  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(errors.size()); ++i) {
    error_vec.segment<3>(i * 3) = errors[static_cast<size_t>(i)];
  }

  return {J, error_vec};
}

inline Eigen::MatrixXd MakeKKTMatrix(const Eigen::MatrixXd& Jc,
                                     const Eigen::MatrixXd& Jt,
                                     double reg = 1e-6) {
  if (Jc.cols() != Jt.cols()) {
    throw std::invalid_argument(
        "Jc and Jt must have the same number of columns (variables)");
  }

  const Eigen::Index num_vars = Jc.cols();
  const Eigen::Index num_constraints = Jc.rows();

  Eigen::MatrixXd G = Jt.transpose() * Jt;
  G += reg * Eigen::MatrixXd::Identity(num_vars, num_vars);

  Eigen::MatrixXd KKT_left(num_vars + num_constraints, num_vars);
  KKT_left << G, Jc;

  Eigen::MatrixXd KKT_right(num_vars + num_constraints, num_constraints);
  KKT_right << Jc.transpose(),
      Eigen::MatrixXd::Zero(num_constraints, num_constraints);

  Eigen::MatrixXd KKT(num_vars + num_constraints,
                      num_vars + num_constraints);
  KKT << KKT_left, KKT_right;
  return KKT;
}

inline Eigen::VectorXd SolveIKStep(const pinocchio::Model& model,
                                  pinocchio::Data& data,
                                  const Eigen::VectorXd& q,
                                  const std::vector<ConstraintPair>& constraints,
                                  // const std::map<GridIndex, Eigen::Vector3d>&
                                  const PositionMap&
                                      goal_positions,
                                  const FrameIndexMap& frame_ids,
                                  double reg = 1e-6) {
  const Eigen::MatrixXd Jc =
      GetConstraintJacobianMatrix(model, data, q, constraints, frame_ids);
  const auto task =
      GetTaskJacobiansWithErrors(model, data, q, goal_positions, frame_ids);
  const Eigen::MatrixXd& Jt = task.first;
  const Eigen::VectorXd& errors = task.second;

  const Eigen::MatrixXd KKT = MakeKKTMatrix(Jc, Jt, reg);
  const Eigen::Index num_vars = Jc.cols();
  const Eigen::Index num_constraints = Jc.rows();

  Eigen::VectorXd b(num_vars + num_constraints);
  b.head(num_vars) = -Jt.transpose() * errors;
  b.tail(num_constraints).setZero();

  const Eigen::VectorXd sol = KKT.fullPivLu().solve(b);
  return sol.head(num_vars);
}

class ConstrainedIKSolver() {

};

}  // namespace skyweave
