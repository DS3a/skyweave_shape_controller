#pragma once

#include <map>
#include <utility>
#include <Eigen/Core>

#define PI 3.141592653589793f

namespace skyweave {

using GridIndex = std::pair<int, int>;
using PositionMap = std::map<GridIndex, Eigen::Vector3d>;
namespace controller {

class GammaSurface {
    /*
    Provides the mapping from R^2 to R^3
    this functions as a setpoint for the 
    */

  public:
    GammaSurface();
    ~GammaSurface();

    void Initialize(float side_length, int num_elements);

    Eigen::Vector3d gamma_sur(float u, float v,
                              Eigen::Vector3d base_pos = Eigen::Vector3d::Zero());

    void update_amplitude(float amplitude);
    void update_angle(float angle);
    void update_phase(float phase);
    void update_frequency(float frequency);

    PositionMap get_goals();

  private:
    /* parameters for the gamma surface controller
     * Amplitude
     * angle
     * phase
     * frequency
     */
    float amplitude_ = 0.05f; // in meters, so this is 5cm
    float angle_ = 0.0f; // in radians
    float phase_ = 0.0f; // in radians
    float frequency_ = PI/0.4f; // in Hz

    /*parameters of the surface
     * side length
     * number of elements in each side
     * element distance = side length / (number of elements - 1)
     */
    float side_length_ = 0.4f; // in meters
    int num_elements_ = 5;
    float element_distance_ = side_length_ / (num_elements_ - 1);

};

} // namespace controller
}  // namespace skyweave
