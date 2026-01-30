#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

#include <map>
#include <memory>
#include <utility>

namespace skyweave {

using GridIndex = std::pair<int, int>;

class Thruster {
 public:
  explicit Thruster(gazebo::physics::LinkPtr link);

  void ApplyThrust(double thrust_newtons);

 private:
  gazebo::physics::LinkPtr link_;
};

void apply_thrusts(
    const std::map<skyweave::GridIndex, double>& thrust_commands,
    const std::map<skyweave::GridIndex, std::shared_ptr<Thruster>>&
        thruster_map);

}  // namespace skyweave
