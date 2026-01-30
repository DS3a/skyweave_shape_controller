#include <inverse_kinematics.hpp>

#include <cmath>

namespace skyweave {

std::vector<ConstraintPair> BuildConstraintFrames(int num_elements_x,
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

Eigen::RowVectorXd GetConstraintJacobian(const pinocchio::Model& model,
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

Eigen::RowVectorXd GetConstraintJacobianForPair(
    const pinocchio::Model& model, pinocchio::Data& data,
    const Eigen::VectorXd& q, const ConstraintPair& pair,
    const FrameIndexMap& frame_ids) {
  const auto frame_id1 = frame_ids.at(pair.first);
  const auto frame_id2 = frame_ids.at(pair.second);
  return GetConstraintJacobian(model, data, q, frame_id1, frame_id2);
}

Eigen::MatrixXd GetConstraintJacobianMatrix(
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

Eigen::MatrixXd GetTaskJacobians(const pinocchio::Model& model,
                                 pinocchio::Data& data,
                                 const Eigen::VectorXd& q,
                                 const PositionMap& goal_positions,
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

std::pair<Eigen::MatrixXd, Eigen::VectorXd> GetTaskJacobiansWithErrors(
    const pinocchio::Model& model, pinocchio::Data& data,
    const Eigen::VectorXd& q, const PositionMap& goal_positions,
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

Eigen::MatrixXd MakeKKTMatrix(const Eigen::MatrixXd& Jc,
                              const Eigen::MatrixXd& Jt,
                              double reg) {
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

Eigen::VectorXd SolveIKStep(const pinocchio::Model& model,
                            pinocchio::Data& data,
                            const Eigen::VectorXd& q,
                            const std::vector<ConstraintPair>& constraints,
                            const PositionMap& goal_positions,
                            const FrameIndexMap& frame_ids,
                            double reg) {
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

ConstrainedIKSolver::ConstrainedIKSolver(
    std::shared_ptr<pinocchio::Model> pin_model,
    const FrameIndexMap& frame_ids)
    : pin_model_(pin_model),
      joint_positions_(pin_model_->nq),
      dq_(pin_model_->nv),
      frame_ids_(frame_ids) {
  if (!pin_model_) {
    throw std::invalid_argument("pin_model must not be null");
  }

  pin_data_ = std::make_shared<pinocchio::Data>(*pin_model_);
  joint_positions_ = pinocchio::neutral(*pin_model_);
  dq_.setZero();

  const int num_links = static_cast<int>(pin_model_->names.size());
  num_elements_x_ = static_cast<int>(std::sqrt(num_links));
  num_elements_y_ = num_elements_x_;
  constraints_ = BuildConstraintFrames(num_elements_x_, num_elements_y_);
}

void ConstrainedIKSolver::setInitialJointPositions(const Eigen::VectorXd& q) {
  if (q.size() != joint_positions_.size()) {
    throw std::invalid_argument("Initial joint position size mismatch");
  }
  joint_positions_ = q;
}

void ConstrainedIKSolver::Solve(const PositionMap& goals, int iterations) {
  if (goals.empty()) {
    throw std::invalid_argument("Goal positions must be set before solving");
  }
  if (iterations < 1) {
    throw std::invalid_argument("Iteration count must be at least 1");
  }

  goal_positions_ = goals;

  for (int iter = 0; iter < iterations; ++iter) {
    dq_ = skyweave::SolveIKStep(*pin_model_, *pin_data_, joint_positions_,
                               constraints_, goal_positions_, frame_ids_);
    joint_positions_ =
        pinocchio::integrate(*pin_model_, joint_positions_, dq_);
  }
}

Eigen::VectorXd ConstrainedIKSolver::CurrentJointPositions() const {
  return joint_positions_;
}

}  // namespace skyweave
