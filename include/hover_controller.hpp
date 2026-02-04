/* 
    take in the u which was calculated by the shape controller
    and modify it in such a way that the delta u remains in the null space
    or stays as small as possible to stay close to the original command
    with the additional constraint of controlling the acceleration of the base link
    

    we start by creating a new pinocchio model, this time, a floating base one.
    the dynamics equation will still remain
    M(q) * dot_v + C(q, v) * v + G(q) = S(q) + A(q)^T * u + A(q)^T * delta_u

    but now, q and v includes the base link 6DOF

    the QP will solve for delta_u and dot_v
    The constraints will be
            M(q) * dot_v - A(q) * delta_u = - C(q, v) * v - G(q) + S(q) + A(q) * u
    this is basically just rearranging the dynamics equation
    A(q)^T comes from the shape controller. We just add a 6x6 0 matrix on top of it to 
    account for the floating base which is not directly actuated
    same goes for the S(q) spring forces, we just add 6 zeros on top of the spring model we have


            dot_v(0:6) = desired_base_acceleration
    this controls the linear and angular acceleration of the base link

            A(q) * delta_u = s
    s is the slack variable, ideally it should be zero, to keep it within the 
    null space of A(q)^T and hence not affect the shape of the surface, 
    while keeping the acceleration of it's base link controlled.

    the cost will be minimize || delta_u ||^2 + || dot_v ||^2 + || s ||^2

*/

#define SDF_PATH "/home/ds3a/dev/wadiyan_carpet/models/mesh/mass_mesh_open_tree.sdf"

#include <state_estimator.hpp>
#include <inverse_kinematics.hpp>
#include <skyweave_sim.hpp>
#include <shape_controller.hpp>


#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/sdf.hpp>

#include <Eigen/Core>

#include <casadi/casadi.hpp>

#include <map>
#include <memory>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

using namespace casadi;

namespace skyweave {
namespace controller {

class HoverController {
public: // make everything public cuz I'm lazy to make setter and getter functions
    std::shared_ptr<skyweave::StateEstimator> state_estimator_; // also need to get the base link pose and orientation and velocity
    std::shared_ptr<pinocchio::Model> pin_model_; // needs to be free flyer
    pinocchio::Data pin_data_;

    std::shared_ptr<skyweave::controller::ShapeController> shape_controller_; // to get the A matrix and spring forces

    Eigen::VectorXd desired_base_acceleration_; // size 6
    pinocchio::JointModelFreeFlyer root_joint;  // <- floating base (7D q, 6D v)
    const std::string root_joint_name = "base_link";
    Eigen::VectorXd base_link_twist; // size 6 (linear vel + angular velocity)
    Eigen::VectorXd base_link_pose; // size 7 (position + orientation as quaternion)
    Eigen::VectorXd thrusts; // size = number of thrusters
    int num_thrusters_ = 25;



    HoverController(std::shared_ptr<skyweave::StateEstimator> state_estimator,
                    std::shared_ptr<skyweave::controller::ShapeController> shape_controller) {
        this->state_estimator_ = state_estimator;
        this->shape_controller_ = shape_controller;

        PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) contact_models;
        const std::string root_link_name = "mass_x0_y0";
        std::vector<std::string> package_dirs; // empty for now, can be filled later if needed


        this->pin_model_ = std::make_shared<pinocchio::Model>();

          pinocchio::sdf::buildModel(
            std::string(SDF_PATH),
            root_joint,
            root_joint_name,
            *(this->pin_model_),
            contact_models,
            root_link_name,
            package_dirs,
            /*verbose=*/true
        );
        this->pin_data_ = pinocchio::Data(*(this->pin_model_));
        this->desired_base_acceleration_ = Eigen::VectorXd::Zero(6);
        this->num_thrusters_ = shape_controller_->num_thrusters_;

        std::cout << "Loaded model: " << this->pin_model_->name << "\n";
        std::cout << "nq = " << this->pin_model_->nq << ", nv = " << this->pin_model_->nv << "\n";
        // NOTE checked this. it is indeed a floating base model now

    }

    void setDesiredBaseAcceleration(const Eigen::VectorXd& desired_accel) {
        if (desired_accel.size() != 6) {
            throw std::invalid_argument("Desired base acceleration must be of size 6.");
        }
        this->desired_base_acceleration_ = desired_accel;
    }


    void setBaseLinkPose(const Eigen::VectorXd& pose) {
        if (pose.size() != 7) {
            throw std::invalid_argument("Base link pose must be of size 7.");
        }
        this->base_link_pose = pose;
    }

    void setBaseLinkTwist(const Eigen::VectorXd& twist) {
        if (twist.size() != 6) {
            throw std::invalid_argument("Base link twist must be of size 6.");
        }
        this->base_link_twist = twist;
    }

    // this function needs the thrusts calculated from the shape controller to be able to
    // calculate the delta_u that will achieve the desired base acceleration while keeping 
    // the shape of the surface as much as possible
    void ComputeControlStep() {
        // DOING implement the QP here as per the plan outlined above
        // we assume that the shape control QP has been solved and we have the A matrix
        //   and the thrusts u
        casadi::DM A_dm = this->shape_controller_->A_dm_; // to be filled from shape controller
        casadi::DM shape_thrusts = this->shape_controller_->thrusts_dm_; // to be filled from shape controller

        casadi::MX delta_u = casadi::MX::sym("u", this->num_thrusters_);
        casadi::MX dot_v = casadi::MX::sym("dot_v", this->pin_model_->nv);
        casadi::MX s = casadi::MX::sym("s", 6); // slack variable for base acceleration constraint

        casadi::MX decision_vars = casadi::MX::vertcat({delta_u, dot_v, s});
        
        casadi::DM delta_u_w = casadi::DM::ones(this->num_thrusters_, 1)  * 5.0;
        casadi::DM dot_v_w = casadi::DM::ones(this->pin_model_->nv, 1)    * 5.0;
        casadi::DM s_w = casadi::DM::ones(6, 1)                           * 20.0; 
        casadi::MX objective = dot(delta_u_w, pow(delta_u, 2)) + 
                               dot(dot_v_w, pow(dot_v, 2)) + 
                               dot(s_w, pow(s, 2));

        // add the dynamics constraint and the base acceleration constraint as per the plan above
        // as well as the null space constraint
        casadi::MXDict nlp;
        nlp["x"] = decision_vars;
        nlp["f"] = objective;


        // accel of base link is in dot_v(0:6)
        // NOTE another option is to add this in the objective function as a penalty term
        casadi::DM desired_base_accel_dm = casadi::DM::zeros(6, 1);
        casadi::MX accel_constraint = dot_v(casadi::Slice(0,6)) - desired_base_accel_dm;

        // get the A matrix from the shape_controller (stored in A_dm)
        // the thrust from the shape_controller is stored in shape_thrusts
        // dynamics constraint
        // TODO get M, C, G for the floating base model from pinocchio
        // TODO get S from the spring model
        // we have A and u from the shape controller already
        // M(q) * dot_v - A(q) * delta_u = - C(q, v) * v - G(q) + S(q) + A(q) * u
        // need to add 6 zeros on top of S and A^T * u to account for floating base
        // start with constructing q and v
        Eigen::VectorXd current_q = Eigen::VectorXd::Zero(this->pin_model_->nq);
        Eigen::VectorXd current_v = Eigen::VectorXd::Zero(this->pin_model_->nv);
        // set the base link pose and twist
        current_q.segment(0,7) = this->base_link_pose; // TODO check this
        current_v.segment(0,6) = this->base_link_twist; // TODO IKD if segment is the right function to use here
        // set the rest from the state estimator
        Eigen::VectorXd est_q = this->state_estimator_->CurrentJointPositions();
        Eigen::VectorXd est_v = this->state_estimator_->CurrentJointVelocities();
        current_q.segment(7, est_q.size()) = est_q;
        current_v.segment(6, est_v.size()) = est_v;
        // compute M
        // pinocchio::crba(*(this->pin_model_), this->pin_data_, current_q);
        // Eigen::MatrixXd M = this->pin_data_.M;
        // M.triangularView<Eigen::StrictlyLower>() =
        //     M.transpose().triangularView<Eigen::StrictlyLower>();
        pinocchio::crba(*(this->pin_model_), this->pin_data_, current_q);
        Eigen::MatrixXd M = this->pin_data_.M;
        M.triangularView<Eigen::StrictlyLower>() =
            M.transpose().triangularView<Eigen::StrictlyLower>();
        // this is done because pinocchio only computes the upper triangular part of M
        
        // compute the non-linear effects
        pinocchio::nonLinearEffects(*(this->pin_model_), this->pin_data_, current_q,
                              current_v);
        Eigen::VectorXd h = this->pin_data_.nle;
        // reframing the equation to make rhs 0
        // 


        // null space constraint with slack
        // A * delta_u - s = 0
        casadi::MX null_space_constraint = mtimes(A_dm, delta_u) - s;

        casadi::MX g = casadi::MX::vertcat({accel_constraint, null_space_constraint /*, dynamics_constraint */});


    }


};

} // namespace controller
} // namespace skyweave