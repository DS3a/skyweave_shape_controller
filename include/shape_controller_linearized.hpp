#pragma once
#define SHAPE_CONTROLLER_LINEARIZED


#include <gamma_surface.hpp>
#include <state_estimator.hpp>
#include <inverse_kinematics.hpp>
#include <skyweave_sim.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <Eigen/Core>

#include <casadi/casadi.hpp>

#include <map>
#include <memory>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace skyweave {
namespace controller {


using namespace casadi;
class ShapeControllerLinearized {
    std::unique_ptr<skyweave_sim::Springs> springs_;
    std::vector<skyweave::ConstraintPair> constraints_;
    std::shared_ptr<skyweave::StateEstimator> state_estimator_;
    std::shared_ptr<skyweave::controller::GammaSurface> gamma_surface_;
    std::shared_ptr<pinocchio::Model> pin_model_;
    pinocchio::Data pin_data_;
    Eigen::VectorXd required_joint_positions_; // size nq
    Eigen::VectorXd desired_joint_acceleration_;
    Eigen::VectorXd spring_torques_;
    Eigen::Quaterniond centre_orientation_;


    // linearization variables
    Eigen::VectorXd q_c;
    Eigen::VectorXd v_c;
    Eigen::VectorXd u_c;
    
    // linearizing the forward dynamics equality constraint
    Eigen::MatrixXd Afx;
    Eigen::MatrixXd Bfu;
    Eigen::VectorXd c_fxu;

    // linearizing the inequality constraints
    Eigen::MatrixXd H;
    Eigen::VectorXd H_c;

    // linearizing the 0 gaussian curvature equality constraints
    Eigen::MatrixXd G;
    Eigen::VectorXd G_c;



    Eigen::VectorXd previous_thrusts_;
    casadi::DM A_dm_; // maping from thruster forces to joint torques, size nv x num_thrusters
    casadi::DM thrusts_dm_; // size num_thrusters x 1
    std::unique_ptr<skyweave::ConstrainedIKSolver> ik_solver_;

    std::vector<skyweave::GridIndex> ordered_indices;
    double kp_ = 10.0;
    double kd_ = 2.0;
    double ki_ = 0.01;
    Eigen::VectorXd integral_error_; // size nv
    int num_thrusters_=25;

    // needs the current state (q, v) : which you can get from state estimator
    // needs the setpoints from gamma surface
    // calculate the required q from IK using gamma surface
    ShapeControllerLinearized(std::shared_ptr<skyweave::StateEstimator> state_estimator,
                    std::shared_ptr<skyweave::controller::GammaSurface> gamma_surface,
                    std::shared_ptr<pinocchio::Model> pin_model);

    void linearize_model(); // acquire the current state from state_estimator_
    // use that to calculate A, B, c, H, H_c, G, G_c


    void acquire_spring_model(std::unique_ptr<skyweave_sim::Springs> springs) {
        this->springs_ = std::move(springs);

    }
    
    void setSpringTorques(const Eigen::VectorXd& spring_torques);
    void ComputeRequiredJointPosAndAccel();
    std::map<skyweave::GridIndex, double> ComputeControlStep();

};



}    
}