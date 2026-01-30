#pragma once

// there is a thruster at each mass link.
// thrusts are applied in the +z direction of the link
// take in a map from std::map<skyweave::GridIndex, double> // thrust in newtons
// map from std::map<skyweave::GridIndex, int> // link index in the links vector (gazebo)

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

#include <inverse_kinematics.hpp>

#include <map>
#include <memory>

namespace skyweave {

class Thruster {
  private:
    gazebo::physics::LinkPtr link_;
  public:
    explicit Thruster(gazebo::physics::LinkPtr link);

    void ApplyThrust(double thrust_newtons);
};

void apply_thrusts(const std::map<skyweave::GridIndex, double>& thrust_commands,
                   const std::map<skyweave::GridIndex, std::shared_ptr<Thruster>>&
                       thruster_map);

} // namespace skyweave
