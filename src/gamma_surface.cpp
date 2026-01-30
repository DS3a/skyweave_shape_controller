#include <gamma_surface.hpp>
#include <iostream>
#include <cmath>

namespace skyweave::controller {

GammaSurface::GammaSurface() = default;
GammaSurface::~GammaSurface() = default;

void GammaSurface::Initialize(float side_length, int num_elements) {
    // parameters to accept
    // side length of the model
    // number of elements in each side
    this->side_length_ = side_length;
    this->num_elements_ = num_elements;

    // after accepting everything, calculate element distance
    this->element_distance_ = this->side_length_ / (this->num_elements_ - 1);
}

Eigen::Vector3d GammaSurface::gamma_sur(float u, float v, Eigen::Vector3d base_pos) {
    /*
    u, v are in meters
    base_pos is the center position of the surface in the world frame
    */
    float x = u;
    float y = v;
    // s = u * np.cos(angle) + v * np.sin(angle)

    // z = A * np.cos(frequency*s + phase) # bump

    // offset = base_pos - np.array([0, 0, A * np.cos(frequency*(0) + phase)])
    // #                                                          s = 0 at (0,0)
    // x += offset[0]
    // y += offset[1]
    // z += offset[2]
    float s = u * std::cos(this->angle_) + v * std::sin(this->angle_);
    float z = this->amplitude_ * std::cos(this->frequency_ * s + this->phase_);
    Eigen::Vector3d offset =
        base_pos -
        Eigen::Vector3d(0, 0,
                        this->amplitude_ *
                            std::cos(this->frequency_ * (0) + this->phase_));
    x += offset.x();
    y += offset.y();
    z += offset.z();

    return Eigen::Vector3d(x, y, z);
}

void GammaSurface::update_amplitude(float amplitude) {
    if (this->amplitude_ == amplitude) {
        return;
    }
    this->amplitude_ = amplitude; 
    this->calculated_q = false;    
}

void GammaSurface::update_angle(float angle) { 
    if (this->angle_ == angle) {
        return;
    }
    this->angle_ = angle; 
    this->calculated_q = false;
}

void GammaSurface::update_phase(float phase) { 
    if (this->phase_ == phase) {
        return;
    }
    this->phase_ = phase;
    this->calculated_q = false;
}

void GammaSurface::update_frequency(float frequency) { 
    if (this->frequency_ == frequency) {
        return;
    }
    this->frequency_ = frequency; 
    this->calculated_q = false;    
}

PositionMap GammaSurface::get_goals() {
    // goal_positions = dict()
    // for i in range(int(-num_elements_x/2), int(num_elements_x/2)+1):
    //     for j in range(int(-num_elements_y/2), int(num_elements_y/2)+1):
    //         x, y, z = gamma_sur(i*elem_dist, j*elem_dist, A=A,
    //         base_pos=np.array([0, 0, 0]))
    //         goal_positions[(i, j)] = np.array([x, y, z])
    // PositionMap goal_positions;
    if (!this->calculated_q) {
        std::cout << "the params were changed, recalculating the goal positions and joint positions\n";
        for (int i = -num_elements_ / 2; i <= num_elements_ / 2; ++i) {
            for (int j = -num_elements_ / 2; j <= num_elements_ / 2; ++j) {
                float u = i * element_distance_;
                float v = j * element_distance_;
                Eigen::Vector3d pos = gamma_sur(u, v, Eigen::Vector3d::Zero());
                this->goal_positions_[{i, j}] = pos;
            }
        }
        this->goal_joint_positions_ = pinocchio::neutral(*this->pin_model_);
        this->ik_solver_->setInitialJointPositions(
            pinocchio::neutral(*this->pin_model_));
        this->ik_solver_->Solve(this->goal_positions_, 5);
        this->goal_joint_positions_ = this->ik_solver_->CurrentJointPositions();
        this->calculated_q = true;

    } // if it is already calculated then just return the previously calculated stuff
    return this->goal_positions_;
}

Eigen::VectorXd GammaSurface::get_goal_joint_positions() {
    return this->goal_joint_positions_;
}

void GammaSurface::init_ik_solver(std::shared_ptr<pinocchio::Model> pin_model,
                                 const FrameIndexMap& frame_ids) {

    this->pin_model_ = pin_model;
    this->pin_data_ = pinocchio::Data(*this->pin_model_);
    this->ik_solver_ =
        std::make_unique<skyweave::ConstrainedIKSolver>(this->pin_model_, frame_ids);

    this->goal_joint_positions_ = pinocchio::neutral(*this->pin_model_);

}

}  // namespace skyweave::controller
