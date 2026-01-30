#pragma once

#include <gamma_surface.hpp>
#include <inverse_kinematics.hpp>
#include <state_estimator.hpp>

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <Eigen/Core>

#include <casadi/casadi.hpp>

#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace skyweave {
namespace controller {

class ShapeController {
 public:
  ShapeController(std::shared_ptr<skyweave::StateEstimator> state_estimator,
                  std::shared_ptr<skyweave::controller::GammaSurface>
                      gamma_surface,
                  std::shared_ptr<pinocchio::Model> pin_model);

  void setSpringTorques(const Eigen::VectorXd& spring_torques);

  void ComputeRequiredJointPosAndAccel();

  std::map<skyweave::GridIndex, double> ComputeControlStep();

  std::shared_ptr<skyweave::StateEstimator> state_estimator_;
  std::shared_ptr<skyweave::controller::GammaSurface> gamma_surface_;
  std::shared_ptr<pinocchio::Model> pin_model_;
  pinocchio::Data pin_data_;
  Eigen::VectorXd required_joint_positions_;
  Eigen::VectorXd desired_joint_acceleration_;
  Eigen::VectorXd spring_torques_;
  std::unique_ptr<skyweave::ConstrainedIKSolver> ik_solver_;
  double kp_ = 50.0;
  double kd_ = 12.0;
  double ki_ = 0.0;
  Eigen::VectorXd integral_error_;
};

}  // namespace controller
}  // namespace skyweave
