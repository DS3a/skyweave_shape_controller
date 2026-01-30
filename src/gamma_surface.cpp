#include <gamma_surface.hpp>

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

void GammaSurface::update_amplitude(float amplitude) { this->amplitude_ = amplitude; }

void GammaSurface::update_angle(float angle) { this->angle_ = angle; }

void GammaSurface::update_phase(float phase) { this->phase_ = phase; }

void GammaSurface::update_frequency(float frequency) { this->frequency_ = frequency; }

PositionMap GammaSurface::get_goals() {
    // goal_positions = dict()
    // for i in range(int(-num_elements_x/2), int(num_elements_x/2)+1):
    //     for j in range(int(-num_elements_y/2), int(num_elements_y/2)+1):
    //         x, y, z = gamma_sur(i*elem_dist, j*elem_dist, A=A,
    //         base_pos=np.array([0, 0, 0]))
    //         goal_positions[(i, j)] = np.array([x, y, z])
    PositionMap goal_positions;
    for (int i = -num_elements_ / 2; i <= num_elements_ / 2; ++i) {
        for (int j = -num_elements_ / 2; j <= num_elements_ / 2; ++j) {
            float u = i * element_distance_;
            float v = j * element_distance_;
            Eigen::Vector3d pos = gamma_sur(u, v, Eigen::Vector3d::Zero());
            goal_positions[{i, j}] = pos;
        }
    }
    return goal_positions;
}

}  // namespace skyweave::controller
