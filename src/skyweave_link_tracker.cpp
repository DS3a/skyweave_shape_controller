#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <Eigen/Core>

#include <pinocchio/parsers/sdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/contact-info.hpp>  // for RigidConstraintModel (depending on version)

#include <state_estimator.hpp>
#include <inverse_kinematics.hpp>

#include <filesystem>
#include <exception>
#include <algorithm>
#include <map>
#include <regex>
#include <sstream>
#include <string>
#include <vector>


// this is the path to the open tree model, not the closed loop which is being used by gazebo
// #define SDF_PATH "mass_mesh_open_tree.sdf"
#define SDF_PATH "/home/ds3a/dev/wadiyan_carpet/models/mesh/mass_mesh_open_tree.sdf"

namespace gazebo {
class SkyweaveLinkTracker : public ModelPlugin {
 public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
    if (!_model) {
      gzerr << "SkyweaveLinkTracker plugin requires a model.\n";
      return;
    } else {
      gzmsg << "Loading SkyweaveLinkTracker plugin for model: "
            << _model->GetName() << std::endl;
    }

    this->model = _model;
    this->world = _model->GetWorld();

    gzmsg << "Building Pinocchio model from SDF at: " << std::string(SDF_PATH) << std::endl;
    this->pin_model = std::make_shared<pinocchio::Model>();

    PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) constraint_models;
    std::string model_path = std::string(SDF_PATH);
    std::string base_link = std::string("mass_x0_y0");

    pinocchio::sdf::buildModel(model_path, *(this->pin_model), constraint_models, base_link);
    
    this->pin_data = std::make_shared<pinocchio::Data>(*(this->pin_model));
    gzmsg << "Built model succssfully with " << this->pin_model->nq << " DOF and "
          << size(this->pin_model->names) << " frames.\n";

    // for (auto name: this->pin_model->frames) {
    //   gzmsg << name.name << "\n";
    // }

    // this function adds everything to this->frame_ids
    this->CollectLinks(_sdf);

    this->state_estimator = std::make_shared<skyweave::StateEstimator>(this->pin_model, this->frame_ids);
    if (_sdf && _sdf->HasElement("print_rate")) {
      this->printRate = _sdf->Get<double>("print_rate");
    }
    if (this->printRate > 0.0) {
      this->printPeriod = 1.0 / this->printRate;
      this->state_estimator->setRate(this->printRate);
    }

    this->lastPrintTime = this->world->SimTime();
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&SkyweaveLinkTracker::OnUpdate, this, std::placeholders::_1));


   

    this->num_links = this->links.size();
    this->side_links = static_cast<int>(std::sqrt(this->num_links));
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
      const std::string& name = link->GetName();
      if (!prefix.empty()) {
        if (name.rfind(prefix, 0) != 0) {
          continue;
        }
      }
      this->links.push_back(link);
      int id = this->pin_model->getFrameId(name); // ensure frame exists in pinocchio model
      if (id < 0) {
        gzerr << "Link name " << name << " not found in Pinocchio model.\n";
      } else {
        skyweave::GridIndex index;
        if (ParseMassLinkName(name, index.first, index.second)) {
          this->frame_ids[index] = id;
        }
      }
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
    // rate of the state estimator
    if (this->printPeriod > 0.0) {
      const double elapsed = (info.simTime - this->lastPrintTime).Double();
      if (elapsed < this->printPeriod) {
        // return;
        // move to check if the controller is up next
      } else {
        this->lastPrintTime = info.simTime;
        this->positions.clear();

        std::ostringstream stream;
        // stream << "Skyweave link positions at " << info.simTime.Double() << "s ("
              //  << this->links.size() << " links)" << std::endl;

        Eigen::Vector3d centre;
        for (const auto& link : this->links) {
          if (!link) {
            continue;
          }
          const auto pose = link->WorldPose();
          // stream << " - " << link->GetName() << ": " << pose.Pos().X() << " "
          //        << pose.Pos().Y() << " " << pose.Pos().Z() << std::endl;

          int xIndex = 0;
          int yIndex = 0;
          if (ParseMassLinkName(link->GetName(), xIndex, yIndex)) {
            this->positions[{xIndex, yIndex}] =
                Eigen::Vector3d(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
            
            if (xIndex == 0 && yIndex == 0) {
              centre = Eigen::Vector3d(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
            }
          }
        }

        stream << "Skyweave link positions grid:" << std::endl;
        for (int y = -this->side_links/2; y <= this->side_links/2; ++y) {
          for (int x = -this->side_links/2; x <= this->side_links/2; ++x) {
            auto it = this->positions.find({x, y});
            if (it != this->positions.end()) {
              Eigen::Vector3d& pos = it->second;
              pos -= centre; // make it a "offset from centre"
            }
          }
        }
      
      
// TODO update the state estimator with these "measurements", and make it update the model state
        this->state_estimator->setPositionMeasurements(this->positions);
        this->state_estimator->updateEstimations(); 
      
      }
    }
  }

  physics::ModelPtr model;
  physics::WorldPtr world;

  std::vector<physics::LinkPtr> links;
  // std::map<skyweave::GridIndex, Eigen::Vector3d> positions;
  skyweave::PositionMap positions;

  std::shared_ptr<pinocchio::Model> pin_model;
  std::shared_ptr<pinocchio::Data> pin_data;

  // TODO create a state estimator instance 

  int num_links = 0; // the total number of links in the model
  int side_links = 0; // the number of links on one side of the model (square root of the total number of links)
  event::ConnectionPtr updateConnection;
  common::Time lastPrintTime;
  skyweave::FrameIndexMap frame_ids;
  double printRate = 2.0;
  double printPeriod = 0.5;

  std::shared_ptr<skyweave::StateEstimator> state_estimator;
};

GZ_REGISTER_MODEL_PLUGIN(SkyweaveLinkTracker)
}  // namespace gazebo
