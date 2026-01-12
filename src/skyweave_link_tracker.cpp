#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <Eigen/Core>

#include <algorithm>
#include <map>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

namespace gazebo {
class SkyweaveLinkTracker : public ModelPlugin {
 public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
    if (!_model) {
      gzerr << "SkyweaveLinkTracker plugin requires a model.\n";
      return;
    }

    this->model = _model;
    this->world = _model->GetWorld();
    this->CollectLinks(_sdf);

    if (_sdf && _sdf->HasElement("print_rate")) {
      this->printRate = _sdf->Get<double>("print_rate");
    }
    if (this->printRate > 0.0) {
      this->printPeriod = 1.0 / this->printRate;
    }

    this->lastPrintTime = this->world->SimTime();
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&SkyweaveLinkTracker::OnUpdate, this, std::placeholders::_1));

    gzmsg << "SkyweaveLinkTracker loaded with " << this->links.size()
          << " links." << std::endl;
  }

 private:
  void CollectLinks(const sdf::ElementPtr& _sdf) {
    std::string prefix;
    if (_sdf && _sdf->HasElement("link_name_prefix")) {
      prefix = _sdf->Get<std::string>("link_name_prefix");
    }

    this->links.clear();
    for (const auto& link : this->model->GetLinks()) {
      if (!link) {
        continue;
      }
      if (!prefix.empty()) {
        const std::string& name = link->GetName();
        if (name.rfind(prefix, 0) != 0) {
          continue;
        }
      }
      this->links.push_back(link);
    }

    std::sort(this->links.begin(), this->links.end(),
              [](const physics::LinkPtr& a, const physics::LinkPtr& b) {
                return a->GetName() < b->GetName();
              });
  }

  static bool ParseMassLinkName(const std::string& name, int& xIndex,
                                int& yIndex) {
    static const std::regex pattern(R"(^mass_x(-?\d+)_y(-?\d+)$)");
    std::smatch match;
    if (!std::regex_match(name, match, pattern) || match.size() != 3) {
      return false;
    }

    xIndex = std::stoi(match[1].str());
    yIndex = std::stoi(match[2].str());
    return true;
  }

  void OnUpdate(const common::UpdateInfo& info) {
    if (this->printPeriod > 0.0) {
      const double elapsed = (info.simTime - this->lastPrintTime).Double();
      if (elapsed < this->printPeriod) {
        return;
      }
    }

    this->lastPrintTime = info.simTime;
    this->positions.clear();

    std::ostringstream stream;
    stream << "Skyweave link positions at " << info.simTime.Double() << "s ("
           << this->links.size() << " links)" << std::endl;
    for (const auto& link : this->links) {
      if (!link) {
        continue;
      }
      const auto pose = link->WorldPose();
      stream << " - " << link->GetName() << ": " << pose.Pos().X() << " "
             << pose.Pos().Y() << " " << pose.Pos().Z() << std::endl;

      int xIndex = 0;
      int yIndex = 0;
      if (ParseMassLinkName(link->GetName(), xIndex, yIndex)) {
        this->positions[{xIndex, yIndex}] =
            Eigen::Vector3d(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
      }
    }

    gzmsg << stream.str();
  }

  physics::ModelPtr model;
  physics::WorldPtr world;
  std::vector<physics::LinkPtr> links;
  std::map<std::pair<int, int>, Eigen::Vector3d> positions;
  event::ConnectionPtr updateConnection;
  common::Time lastPrintTime;
  double printRate = 2.0;
  double printPeriod = 0.5;
};

GZ_REGISTER_MODEL_PLUGIN(SkyweaveLinkTracker)
}  // namespace gazebo
