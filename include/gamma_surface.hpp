#pragma once

#include <Eigen/Core>

#include <map>
#include <utility>

#define PI 3.141592653589793f

namespace skyweave {

using GridIndex = std::pair<int, int>;
using PositionMap = std::map<GridIndex, Eigen::Vector3d>;

namespace controller {

class GammaSurface {
  public:
  GammaSurface() = default;
  ~GammaSurface() = default;

  void Initialize(float side_length, int num_elements);

  Eigen::Vector3d gamma_sur(
      float u, float v, Eigen::Vector3d base_pos = Eigen::Vector3d::Zero());

  void update_amplitude(float amplitude);

  void update_angle(float angle);

  void update_phase(float phase);

  void update_frequency(float frequency);

  PositionMap get_goals();

 private:
  float amplitude_ = 0.05f;
  float angle_ = 0.0f;
  float phase_ = 0.0f;
  float frequency_ = PI / 0.4f;

  float side_length_ = 0.4f;
  int num_elements_ = 5;
  float element_distance_ = side_length_ / (num_elements_ - 1);
};

}  // namespace controller
}  // namespace skyweave
