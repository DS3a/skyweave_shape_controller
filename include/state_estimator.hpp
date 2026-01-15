#pragma once

#include <Eigen/Core>

#include <map>
#include <utility>

namespace skyweave {

using GridIndex = std::pair<int, int>;

class StateEstimator {
 public:
  using PositionMap = std::map<GridIndex, Eigen::Vector3d>;

  void SetGoalPositions(PositionMap goals) { goal_positions_ = std::move(goals); }

  const PositionMap& GoalPositions() const { return goal_positions_; }

  void UpdateLinkPosition(int x, int y, const Eigen::Vector3d& position) {
    current_positions_[{x, y}] = position;
  }

  const PositionMap& CurrentPositions() const { return current_positions_; }

 private:
  PositionMap goal_positions_;
  PositionMap current_positions_;
};

}  // namespace skyweave
