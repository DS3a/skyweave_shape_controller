#pragma once
#define STATE_ESTIMATOR_H

#include <Eigen/Core>
#include <pinocchio/multibody/model.hpp>

namespace state_estimator {

class StateEstimator {
 public:
  StateEstimator();
  ~StateEstimator();

  void Initialize(std::shared_ptr<pinocchio::Model> pin_model);
  void Update(const std::map<std::pair<int, int>, Eigen::Vector3d>& link_positions);

  Eigen::VectorXd GetState() const;
  Eigen::MatrixXd GetCovariance() const;

 private:
  std::shared_ptr<pinocchio::Model> pin_model_;
  std::shared_ptr<pinocchio::Data> pin_data_;

  Eigen::VectorXd state_;
  Eigen::MatrixXd covariance_;
};

}