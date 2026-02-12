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
            M(q) * dot_v - A_b(q) * delta_u = - C(q, v) * v - G(q) + S(q) + A_b(q) * u
    this is basically just rearranging the dynamics equation
    A(q) comes from the shape controller. We just add a 6xnv matrix on top of it to make A_b(q). This
     accounts for the floating base which is not directly actuated, but rather has an acceleration 
     which is controlled by the thrusters.

    for the S(q) spring forces, we can just add 6 zeros on top of the spring model we have
    this is because the springs themselves don't directly affect the base link.
    It is because of these spring forces that we can \assume that the surface is rigid,
     and the thrusters are fixed relative to the base_link, which makes it apply a wrench on the base 
     link when it applies a force on the surface.


            dot_v(0:6) = desired_base_acceleration
    this controls the linear and angular acceleration of the base link

            A(q) * delta_u = s
    As mentioned above A(q) comes from the shape_controller
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
    std::map<skyweave::GridIndex, double> ComputeControlStep() {
        // DOING implement the QP here as per the plan outlined above
        // we assume that the shape control QP has been solved and we have the A matrix
        //   and the thrusts u
        casadi::DM A_dm = this->shape_controller_->A_dm_; // to be filled from shape controller
        casadi::DM shape_thrusts = this->shape_controller_->thrusts_dm_; // to be filled from shape controller


        // decision variables
        casadi::MX delta_u = casadi::MX::sym("u", this->num_thrusters_);
        casadi::MX dot_v = casadi::MX::sym("dot_v", this->pin_model_->nv);
        casadi::MX s = casadi::MX::sym("s", A_dm.size1()); // slack variable for base acceleration constraint
        // casadi::MX s_acc = casadi::MX::sym("s_acc", 6); // slack variable for base acceleration constraint

        // casadi::MX decision_vars = casadi::MX::vertcat({delta_u, dot_v, s, s_acc});
        casadi::MX decision_vars = casadi::MX::vertcat({delta_u, dot_v, s});
        
        
        // weights for the cost function, these can be tuned as hyperparameters
        casadi::DM delta_u_w = casadi::DM::ones(this->num_thrusters_, 1)  * 200.005;
        casadi::DM dot_v_w = casadi::DM::ones(this->pin_model_->nv, 1)    * 500.005;
        casadi::DM s_w = casadi::DM::ones(A_dm.size1(), 1)                * 1.01;
        casadi::DM s_acc_w = casadi::DM::ones(6, 1)                       * 10.0;

        // the objective
        casadi::MX objective = dot(delta_u_w, pow(delta_u, 2)) + 
                               dot(dot_v_w, pow(dot_v, 2)) + 
                               dot(s_w, pow(s, 2))
                               ;
                            //    +
                            //    dot(s_acc_w, pow(s_acc, 2));

        // add the dynamics constraint and the base acceleration constraint as per the plan above
        // as well as the null space constraint
        casadi::MXDict nlp;
        nlp["x"] = decision_vars;
        nlp["f"] = objective;


        // accel of base link is in dot_v(0:6)
        // NOTE another option is to add this in the objective function as a penalty term
        casadi::DM desired_base_accel_dm = casadi::DM::zeros(6, 1);
        desired_base_accel_dm(2) = 0.0;
        // casadi::MX accel_constraint = dot_v(casadi::Slice(0,6)) - desired_base_accel_dm - s_acc_w;
        casadi::MX accel_constraint = dot_v(casadi::Slice(0,6)) - desired_base_accel_dm;

        // get the A matrix from the shape_controller (stored in A_dm)
        // the thrust from the shape_controller is stored in shape_thrusts
        // dynamics constraint
        // get M, C, G for the floating base model from pinocchio
        // get S from the spring model
        // we have A and u from the shape controller already
        // M(q) * dot_v - A(q) * delta_u = - C(q, v) * v - G(q) + S(q) + A(q) * u
        // need to add 6 zeros on top of S and A^T * u to account for floating base
        // start with constructing q and v
        Eigen::VectorXd current_q = Eigen::VectorXd::Zero(this->pin_model_->nq);
        Eigen::VectorXd current_v = Eigen::VectorXd::Zero(this->pin_model_->nv);
        // set the base link pose and twist
        current_q.segment(0,7) = this->base_link_pose;
        current_v.segment(0,6) = this->base_link_twist;
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

        pinocchio::forwardKinematics(*(this->pin_model_), this->pin_data_, current_q);
        pinocchio::updateFramePlacements(*(this->pin_model_), this->pin_data_);
        pinocchio::computeJointJacobians(*(this->pin_model_), this->pin_data_, current_q);


        this->shape_controller_->springs_->CalculateSpringTorques(current_q.segment(7, est_q.size()), current_v.segment(6, est_v.size()));
        Eigen::VectorXd spring_forces_shape = this->shape_controller_->springs_->getGeneralizedSpringForces(); 
        // this is the S(q) term in the dynamics equation, we need to add 6 zeros on top of it to account for the floating base
        Eigen::VectorXd spring_generalized_forces = Eigen::VectorXd::Zero(this->pin_model_->nv);
        spring_generalized_forces.segment(6, spring_forces_shape.size()) = spring_forces_shape;
        casadi::DM spring_generalized_forces_dm = casadi::DM::zeros(this->pin_model_->nv, 1);
        for (int r = 6; r < this->pin_model_->nv; ++r) {
            spring_generalized_forces_dm(r) = spring_generalized_forces(r);
        }
        // convert M to casadi DM
        casadi::DM M_dm = casadi::DM::zeros(this->pin_model_->nv, this->pin_model_->nv);
        for (int r = 0; r < this->pin_model_->nv; ++r) {
            for (int c = 0; c < this->pin_model_->nv; ++c) {
                M_dm(r, c) = M(r, c);
            }
        }

        // convert h to casadi DM
        casadi::DM h_dm = casadi::DM::zeros(this->pin_model_->nv, 1);
        for (int r = 0; r < this->pin_model_->nv; ++r) {
            h_dm(r) = h(r);
        }

        // reframing the equation to make rhs 0
        // M(q) * dot_v - A(q) * delta_u + C(q, v) * v + G(q) - S(q) - A(q) * u = 0
        // M(q) * dot_v - A(q) * delta_u + h - S(q) - A(q) * u = 0
        // casadi::DM shape_thrusts_with_zeros = casadi::DM::zeros(this->pin_model_->nv, 1);
        // shape_thrusts_with_zeros(casadi::Slice(6, this->pin_model_->nv), casadi::Slice()) = shape_thrusts;
        
        // need to accumulate the forces from all the thrusters, and describe how it affects the base link linear and angular acceleration.
        // A_base which gets appended on top of A_dm to account for the floating base. It should be a 6 x num_thrusters matrix, describing the mapping from thruster forces to base link acceleration
       // std::cout << "and the jacobian it's size is "    << J_base.rows() << " x " << J_base.cols() << "\n";
        casadi::DM A_dm_fb = casadi::DM::zeros(this->pin_model_->nv, this->num_thrusters_);
        // calculate a new A_dm with the floating base, by adding Jacobians of all frames the same as the shape controller.

        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(this->pin_model_->nv, this->num_thrusters_);
        int col = 0;
        
        Eigen::Quaterniond base_link_orientation(this->base_link_pose(6), //w
                            this->base_link_pose(3), //x
                            this->base_link_pose(4), //y
                            this->base_link_pose(5)); //z
        Eigen::Matrix3d base_link_to_world_rot = base_link_orientation.toRotationMatrix();
 

        for (const auto& [index, frame_id] : this->state_estimator_->FrameIds()) {
            // ordered_indices.push_back(index);
            const Eigen::MatrixXd J = pinocchio::getFrameJacobian(
                *(this->pin_model_), this->pin_data_, frame_id,
                // pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
               pinocchio::ReferenceFrame::WORLD);
            const Eigen::MatrixXd J_trans = J.topRows(3);
            const Eigen::Matrix3d R = this->pin_data_.oMf[frame_id].rotation();
            const Eigen::Vector3d force_world = R * Eigen::Vector3d(0.0, 0.0, 1.0);
           Eigen::Vector3d force_body = base_link_to_world_rot.transpose() * force_world;
            A.col(col) = J_trans.transpose() * force_body;
            // A.col(col) = J_trans.transpose() * Eigen::Vector3d(0, 0, 1);
            // A.col(col) = J.row(2).transpose();  // only the z thrust
            // std::cout << "the jacobian is " << J_trans << "\n";
            ++col;
        }

        for (int r = 0; r < this->pin_model_->nv; ++r) {
            for (int c = 0; c < this->num_thrusters_; ++c) {
                A_dm_fb(r, c) = A(r, c);
            }
        }

        // take the first 6 rows of A_dm_fb and put it on top of A_dm to form A_base
        casadi::DM A_base = casadi::DM::zeros(6, this->num_thrusters_);
        for (int r = 0; r < 6; ++r) {
            for (int c = 0; c < this->num_thrusters_; ++c) {
                A_base(r, c) = A_dm_fb(r, c);
            }
        }


        // TODO we wanna modify this. We treat the shape as a rigid body, take only the first 6x6 slice of the mass matrix,
        // and the new dynamics constraint will just be the thrusts acting on the base link (producing a torque wrench based on)
        // the relative position of the thruster (w.r.t the base link).
        // M_base * dot_v_base - A_base * delta_u + h_base - A_base * shape_thrusts = 0
        // A_base is the matrix that transforms thrusts in the frames of the thrusters (basically in the form (0, 0, fz)) to
        //  a wrench on the base link, based on the relative position of the thrusters to the base link and the orientation of the surface 
        //  (which determines the direction of the thrust force in the world frame)
        casadi::DM M_base_dm = casadi::DM::zeros(6, 6);
        M_base_dm = M_dm(casadi::Slice(0, 6), casadi::Slice(0, 6));
        casadi:DM h_base_dm = casadi::DM::zeros(6, 1);
        h_base_dm = h_dm(casadi::Slice(0, 6), casadi::Slice());
        // to construct A_dm I need the rotational transform from the thruster to base link
        // I also need both their positions in the world frame to calculate the moment arm for the torque part of the wrench
        // I need these positions in the world frame because I am trying to control the direction of the accereration in the world frame
        // I can get the rotational tranform from the thruster to the base from the pinocchio model in the shape controller.
        // I have the orientation of the base link which I can use to get the transform from the thruster frame to world frame,
        // this I can use to calculate the world frame force vector of the wrench
        
        casadi::DM A_base_wrench_dm = casadi::DM::zeros(6, this->num_thrusters_);
        Eigen::MatrixXd A_base_wrench_eig = Eigen::MatrixXd::Zero(6, this->num_thrusters_);
        int cols = 0;
        // TODO construct the A matrix which is a 6xnum_thrusters matrix that maps the thrusts in the thruster frames to a wrench in base link
        for (auto frame: this->state_estimator_->FrameIds()) {
            skyweave::GridIndex grid_index = frame.first;
            pinocchio::FrameIndex frame_id = frame.second;
            Eigen::Vector3d force_thruster = Eigen::Vector3d(0.0, 0.0, 1.0); // in the thruster frame, the force is always in the +z direction
            // for each thruster frame, get the rotation from the thruster frame to the base link frame

            Eigen::Matrix3d frame_to_world_rot = this->pin_data_.oMf[frame_id].rotation();
            Eigen::Vector3d force_world = frame_to_world_rot * force_thruster;
            Eigen::Vector3d force_base = base_link_to_world_rot.transpose() * force_world;

            Eigen::Vector3d position_thruster_in_world = this->pin_data_.oMf[frame_id].translation();
            Eigen::Vector3d position_base_in_world = this->base_link_pose.segment(0, 3);
            Eigen::Vector3d torque_arm = position_thruster_in_world - position_base_in_world;
            Eigen::Vector3d torque_world = torque_arm.cross(force_world);
            Eigen::Vector3d torque_base = base_link_to_world_rot.transpose() * torque_world;
            Eigen::VectorXd wrench_base = Eigen::VectorXd::Zero(6);
            wrench_base.segment(0, 3) = force_base;
            wrench_base.segment(3, 3) = torque_base;
            // now I can put this wrench in the corresponding column of A_base_wrench_dm
            // each element in u must go into (0, 0, u)^T
            // 
            A_base_wrench_eig.col(cols) = wrench_base;
            cols++;


            // get the position of the thruster in the world frame
            // get the position of the base link in the world frame
            // calculate the force vector in the world frame by rotating the (0, 0, fz) vector in the thruster frame to the world frame using the rotation from thruster to world
            // calculate the torque vector in the world frame by taking the cross product of the position vector from base link to thruster (in world frame) and the force vector in world frame
            // combine the force and torque vector to get the wrench in world frame
            // this wrench is what contributes to the acceleration of the base link in world frame, so we can put it in A_base matrix which maps thrusts to base link acceleration in world frame
        }

        for (int r = 0; r < 6; ++r) {
            for (int c = 0; c < this->num_thrusters_; ++c) {
                A_base_wrench_dm(r, c) = A_base_wrench_eig(r, c);
            }
        }

        // casadi::MX dynamics_constraint = mtimes(M_base_dm, dot_v(casadi::Slice(0, 6))) - mtimes(A_base_wrench_dm, delta_u) + 
        //                 h_base_dm - mtimes(A_base_wrench_dm, shape_thrusts);

        // I have R(thruster -> base). so if I multiply it to the force vector in the thruster frame (0, 0, fz), 
        // I can get the force vector in the base link frame (let us say it is force_base). 
        // I have R(base -> world). so to get the force vector in the world frame. I need to multiply R(base->world) to force_base to get force_world

        // now for the torque part of the wrench (which is in the world frame), I need to get the position of the base link in the world frame.
        // I also need to get the position of the thruster in the world frame.
        // I can get the position of the thruster with respect to the base link from the pinocchio model in the shape controller.
        // I can get the position of the thruster in the world frame by rotating the position of the thruster with respect to 
        // the base link into the world frame using the orientation of the base link, and then adding the position of the base link in the world frame.

        // from the shape controller pinocchio model I can get the displacement vector from the base to the thruster.
        //  let us say that vector is P(base -> thruster). If I multiply R(base -> world) to this vector, I can get the displacement in the world frame.
        // that displacement can be used to calculate the torque produced by the thruster force on the base link in the world frame as 
        // torque_world = cross(P(base->thruster)_world, force_world)





        A_dm_fb = casadi::DM::vertcat({A_base_wrench_dm, A_dm});

        // casadi::MX dynamics_constraint = mtimes(M_dm, dot_v) - mtimes(A_base, delta_u) + 
                        // h_dm - mtimes(A_base, shape_thrusts) - spring_generalized_forces_dm;
        // casadi::MX dynamics_constraint = mtimes(M_dm, dot_v) - casadi::MX::vertcat({casadi::MX::zeros(6, 1), mtimes(A_dm, delta_u)}) + 
        //                 h_dm - casadi::MX::vertcat({casadi::MX::zeros(6, 1), mtimes(A_dm, shape_thrusts)}) - spring_generalized_forces_dm;
        casadi::MX dynamics_constraint = mtimes(M_dm, dot_v) - mtimes(A_dm_fb, delta_u) + 
                        h_dm - mtimes(A_dm_fb, shape_thrusts) - spring_generalized_forces_dm;


        // null space constraint with slack
        // A * delta_u - s = 0
        casadi::MX null_space_constraint = mtimes(A_dm, delta_u);

        // casadi::MX g = casadi::MX::vertcat({accel_constraint, /*null_space_constraint,*/ dynamics_constraint});
        casadi::MX g = casadi::MX::vertcat({accel_constraint, null_space_constraint, dynamics_constraint});

        nlp["g"] = g;
        casadi::Dict opts;
        opts["print_time"] = false;
        opts["ipopt.print_level"] = 5;
        opts["ipopt.max_iter"] = 100;

 /*


        casadi::Function solver =
            casadi::nlpsol(std::string("hover_controller_solver"), std::string("ipopt"), nlp, opts);
        

        casadi::DMDict solution = 
            solver(casadi::DMDict{{"x0", casadi::DM::zeros(this->num_thrusters_ + this->pin_model_->nv + A_dm.size1(), 1)},
                                  {"lbg", casadi::DM::zeros(g.size1(), 1)},
                                  {"ubg", casadi::DM::zeros(g.size1(), 1)}
            });

        casadi::DM delta_u_dm = solution.at("x")(casadi::Slice(0, this->num_thrusters_), casadi::Slice());


        */
        Eigen::VectorXd delta_u_eigen = Eigen::VectorXd::Zero(this->num_thrusters_);
        
        /*
        for (int i = 0; i < this->num_thrusters_; ++i) {
            delta_u_eigen(i) = delta_u_dm(i).scalar();
        }

        std::cout << "calculated deltas " << delta_u_eigen.transpose() << "\n";
*/
        // TODO apply the delta_u to the original thrusts to get the final thrust command
        std::map<skyweave::GridIndex, double> final_thrusts;
        for (int i = 0; i < this->num_thrusters_; ++i) {
            double final_thrust = shape_thrusts(i).scalar() + delta_u_eigen(i);
            // we can also add some saturation here if needed
            final_thrusts[this->shape_controller_->ordered_indices[i]] = final_thrust;
        }

        // make PD controller for final_thrusts[{0, 0}] to track the desired base position (0, 0, 1.3)
        double kp_hover = 0.05;
        double kd_hover = 0.001;
        Eigen::Vector3d current_base_linear_position = this->base_link_pose.segment(0, 3);
        double desired_base_z_position = 1.0;
        double current_base_z_position = current_base_linear_position(2);
        double position_error = desired_base_z_position - current_base_z_position;
        double current_base_z_velocity = this->base_link_twist(2);
        double velocity_error = - current_base_z_velocity;
        double pd_correction = kp_hover * position_error + kd_hover * velocity_error;


        // final_thrusts[{0, 0}] += 1 + pd_correction; // only z direction thrust

        // final_thrusts[{0, 0}] += 1;

        return final_thrusts;
    }
};

} // namespace controller
} // namespace skyweave