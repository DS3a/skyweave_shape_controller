// there is a thruster at each mass link.
// thrusts are applied in the +z direction of the link
// take in a map from std::map<skyweave::GridIndex, double> // thrust in newtons
// map from std::map<skyweave::GridIndex, int> // link index in the links vector (gazebo)

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

namespace skyweave {

class Thruster {
  private:
    gazebo::physics::LinkPtr link_;
  public:
    Thruster(gazebo::physics::LinkPtr link) : link_(link) {}

    void ApplyThrust(double thrust_newtons) {
        // Apply force in the +Z direction of the link frame
        Eigen::Vector3d force(0, 0, thrust_newtons);
        link_->AddRelativeForce(
            ignition::math::Vector3(force.x(), force.y(), force.z()));
    }
};

void apply_thrusts(const std::map<skyweave::GridIndex, double>& thrust_commands,
                   const std::map<skyweave::GridIndex, Thruster>& thruster_map) {
    // TODO apply corresponding thrusts to the thrusters
    for (const auto& command : thrust_commands) {
        const auto& index = command.first;
        const double thrust = command.second;
        const auto thruster_it = thruster_map.find(index);
        if (thruster_it == thruster_map.end()) {
            continue;
        }
        thruster_it->second.ApplyThrust(thrust);
    }
}

} // namespace skyweave
