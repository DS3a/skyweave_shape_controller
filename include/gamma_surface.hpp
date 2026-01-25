#pragma once

#include <map>
#include <utility>
#include <Eigen/Core>

#define PI 3.141592653589793

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
    GammaSurface() = default;
    ~GammaSurface() = default;

    void Initialize() {
    // Initialization code here
    }

    void update_amplitude(float amplitude) {
        this->amplitude_ = amplitude;
    }

    void update_angle(float angle) {
        this->angle_ = angle;
    }

    void update_phase(float phase) {
        this->phase_ = phase;
    }

    void update_frequency(float frequency) {
        this->frequency_ = frequency;
    }

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
    float frequency_ = 0.5f; // in Hz


};

} // namespace controller
}  // namespace skyweave