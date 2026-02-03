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
            M(q) * dot_v - A(q)^T * delta_u = - C(q, v) * v - G(q) + S(q) + A(q)^T * u
    this is basically just rearranging the dynamics equation
    A(q)^T comes from the shape controller. We just add a 6x6 0 matrix on top of it to 
    account for the floating base which is not directly actuated
    same goes for the S(q) spring forces, we just add 6 zeros on top of the spring model we have


            dot_v(0:6) = desired_base_acceleration
    this controls the linear and angular acceleration of the base link

            A(q)^T * delta_u = s
    s is the slack variable, ideally it should be zero, to keep it within the 
    null space of A(q)^T and hence not affect the shape of the surface, 
    while keeping the acceleration of it's base link controlled.

    the cost will be minimize || delta_u ||^2 + || dot_v ||^2 + || s ||^2

*/

#define SDF_PATH "/home/ds3a/dev/wadiyan_carpet/models/mesh/mass_mesh_open_tree.sdf"

#include <state_estimator.hpp>
#include <inverse_kinematics.hpp>
#include <skyweave_sim.hpp>


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


namespace skyweave {
namespace controller {

class HoverController {
public: // make everything public cuz I'm lazy to make setter and getter functions
    std::shared_ptr<skyweave::StateEstimator> state_estimator_; // also need to get the base link pose and orientation and velocity
    std::shared_ptr<pinocchio::Model> pin_model_; // needs to be free flyer
    pinocchio::Data pin_data_;
    Eigen::VectorXd desired_base_acceleration_; // size 6
    pinocchio::JointModelFreeFlyer root_joint;  // <- floating base (7D q, 6D v)
    const std::string root_joint_name = "base_link";
    Eigen::VectorXd base_link_twist; // size 6 (linear vel + angular velocity)
    Eigen::VectorXd base_link_pose; // size 7 (position + orientation as quaternion)
    Eigen::VectorXd thrusts; // size = number of thrusters



    HoverController(std::shared_ptr<skyweave::StateEstimator> state_estimator) {
        this->state_estimator_ = state_estimator;

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

    void setBaseLinkTwist(const Eigen::VectorXd& twist) {
        if (twist.size() != 6) {
            throw std::invalid_argument("Base link twist must be of size 6.");
        }
        this->base_link_twist = twist;
    }

    void ComputeControlStep() {
        // TODO implement the QP here as per the plan outlined above
    }


};

} // namespace controller
} // namespace skyweave