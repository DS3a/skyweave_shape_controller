#include <thrusters.hpp>

#include <Eigen/Core>

namespace skyweave {

Thruster::Thruster(gazebo::physics::LinkPtr link) : link_(link) {}

void Thruster::ApplyThrust(double thrust_newtons) {
  Eigen::Vector3d force(0, 0, thrust_newtons);
  link_->AddRelativeForce(
      ignition::math::Vector3(force.x(), force.y(), force.z()));
}

void apply_thrusts(
    const std::map<skyweave::GridIndex, double>& thrust_commands,
    const std::map<skyweave::GridIndex, std::shared_ptr<Thruster>>&
        thruster_map) {
  for (const auto& command : thrust_commands) {
    const auto& index = command.first;
    const double thrust = command.second;
    const auto thruster_it = thruster_map.find(index);
    if (thruster_it == thruster_map.end()) {
      continue;
    }
    thruster_it->second->ApplyThrust(thrust);
  }
}

}  // namespace skyweave
