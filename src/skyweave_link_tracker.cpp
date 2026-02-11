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
#include <gamma_surface.hpp>
#include <inverse_kinematics.hpp>
#include <thrusters.hpp>
#include <skyweave_sim.hpp>
#include <shape_controller.hpp>
#include <hover_controller.hpp>

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

double unwrap(double prev, double current)
{
    double diff = current - prev;
    while (diff > M_PI)  diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    return current;
    // return prev + diff;
}

// Ensure q is continuous over time: q(t) should not randomly flip sign.
Eigen::Quaterniond makeContinuous(const Eigen::Quaterniond& q,
                                 Eigen::Quaterniond& q_prev)
{
    Eigen::Quaterniond qc = q;
    if (q_prev.coeffs().dot(qc.coeffs()) < 0.0) {
        qc.coeffs() *= -1.0;
    }
    q_prev = qc;
    return qc;
}


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
    this->lastControlTime = this->world->SimTime();
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&SkyweaveLinkTracker::OnUpdate, this, std::placeholders::_1));


   

    this->num_links = this->links.size();
    this->side_links = static_cast<int>(std::sqrt(this->num_links));
    gzmsg << "SkyweaveLinkTracker loaded with " << this->links.size()
          << " links." << std::endl;

    this->springs = std::make_shared<skyweave_sim::Springs>(
        *(this->pin_model), this->frame_ids, this->gz_links_idx_map,
        this->links, /*k_rot=*/0.5);

    this->gamma_surface = std::make_shared<skyweave::controller::GammaSurface>();
    this->shape_controller = std::make_shared<skyweave::controller::ShapeController>(
        this->state_estimator, this->gamma_surface, this->pin_model);
    this->hover_controller = std::make_shared<skyweave::controller::HoverController>(
        this->state_estimator,
        this->shape_controller);
    this->shape_controller->acquire_spring_model(std::make_unique<skyweave_sim::Springs>(
        *(this->pin_model), this->frame_ids, this->gz_links_idx_map,
        this->links, /*k_rot=*/1.1));
  }

 private:
  void CollectLinks(const sdf::ElementPtr& _sdf) {
    std::string prefix;
    if (_sdf && _sdf->HasElement("link_name_prefix")) {
      prefix = _sdf->Get<std::string>("link_name_prefix");
    }

    this->links.clear();
    int link_count = 0;
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
          this->gz_links_idx_map[index] = link_count;
          this->thruster_map[index] = std::make_shared<skyweave::Thruster>(link);
          link_count++;
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
    static int update_count = 0;

    Eigen::VectorXd base_link_pose(7);

    Eigen::VectorXd base_link_twist(6);
    static double desired_roll = 0;
    static double desired_pitch = 0.00;
    // TODO calculate the spring forces and apply the torques on the links based on the model

    this->springs->ApplySpringTorques(
        this->springs->CalculateSpringTorques(
            this->state_estimator->CurrentJointPositions()
        )
    );


//    rate of the state estimator
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
        Eigen::Vector3d base_linear_vel = Eigen::Vector3d::Zero();
        Eigen::Vector3d base_angular_vel = Eigen::Vector3d::Zero();
 
        for (const auto& link : this->links) {
          if (!link) {
            continue;
          }
          const auto pose = link->WorldPose();
          const auto vel = link->WorldLinearVel();
          const auto ang_vel = link->WorldAngularVel();



          // stream << " - " << link->GetName() << ": " << pose.Pos().X() << " "
          //        << pose.Pos().Y() << " " << pose.Pos().Z() << std::endl;

          int xIndex = 0;
          int yIndex = 0;
          if (ParseMassLinkName(link->GetName(), xIndex, yIndex)) {
            this->positions[{xIndex, yIndex}] =
                Eigen::Vector3d(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
            
            if (xIndex == 0 && yIndex == 0) {
              // centre_orientation = Eigen::Quaterniond(
              //     pose.Rot().W(), pose.Rot().X(),
              //     pose.Rot().Y(), pose.Rot().Z());
              centre_orientation.w() = pose.Rot().W();
              centre_orientation.x() = pose.Rot().X();
              centre_orientation.y() = pose.Rot().Y();
              centre_orientation.z() = pose.Rot().Z();

              stream << "Skyweave centre link orientation (quaternion): "
                     << centre_orientation.w() << " "
                     << centre_orientation.x() << " "
                     << centre_orientation.y() << " "
                     << centre_orientation.z() << std::endl;
              centre = Eigen::Vector3d(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
              base_linear_vel = Eigen::Vector3d(vel.X(), vel.Y(), vel.Z());
              base_angular_vel = Eigen::Vector3d(ang_vel.X(), ang_vel.Y(), ang_vel.Z());
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


        base_link_pose.head<3>() = centre;
        Eigen::Quaterniond centre_orientation_q = Eigen::Quaterniond(
            centre_orientation.w(), centre_orientation.x(),
            centre_orientation.y(), centre_orientation.z());
        base_link_pose.tail<4>() = centre_orientation_q.coeffs(); // note that Eigen's quaternion coeffs are in the order (x, y, z, w)
        // the state also contains the centre position and orientation now
        
        base_link_twist.head<3>() = base_linear_vel;
        base_link_twist.tail<3>() = base_angular_vel;

        this->state_estimator->setPositionMeasurements(this->positions, this->centre_orientation);
        this->state_estimator->updateEstimations(); 
        this->hover_controller->setBaseLinkPose(base_link_pose);
        this->hover_controller->setBaseLinkTwist(base_link_twist);
        this->shape_controller->centre_orientation_ = this->centre_orientation;

        // gzmsg << stream.str();
      }
      double elapsed_control = (info.simTime - this->lastControlTime).Double();
      if (elapsed_control < this->controlPeriod) {
        // return;
      } else {
        this->control_ticks++;
        this->lastControlTime = info.simTime;

        // this->springs->CalculateSpringTorques(
        //     this->state_estimator->CurrentJointPositions()
        // );

        static Eigen::Quaterniond prev_centre_orientation_q(1, 0, 0, 0); // identity quaternion
        Eigen::Quaterniond zero_rotation_q(1, 0, 0, 0); // identity quaternion

        // find the euler angles of roll and pitch of the base link from the centre_orientation quaternion
        Eigen::Quaterniond centre_orientation_q = Eigen::Quaterniond(
            centre_orientation.w(), centre_orientation.x(),
            centre_orientation.y(), centre_orientation.z());
          
        centre_orientation_q = zero_rotation_q.conjugate() * centre_orientation_q; // rotate the orientation to make the initial orientation as the reference
        centre_orientation_q.normalize();
        centre_orientation_q = makeContinuous(centre_orientation_q, prev_centre_orientation_q);
        prev_centre_orientation_q = centre_orientation_q;


        Eigen::Matrix3d R = centre_orientation_q.toRotationMatrix();

        static double roll_prev = 0.0;
        static double pitch_prev = 0.0;
        static bool roll_and_not_pitch = true; // whether to correct roll or pitch in the current iteration

        // NOTE euler angles are acting weirdly, they are not continuous and they jump from -pi to pi randomly, so I need to unwrap them to make them continuous before feeding them into 
        // the controller. I think this is because of the way the quaternion is converted to euler angles, it is not guaranteed to be continuous. 
        // So I will implement an unwrap function that takes the previous angle and the current angle and unwraps it to make it continuous.

        double roll_raw = std::atan2(R(2,1), R(2,2));
        double pitch_raw = std::asin(std::clamp(-R(2,0), -1.0, 1.0));

        double roll = unwrap(roll_prev, roll_raw);
        roll = roll_raw;
        double pitch = unwrap(pitch_prev, pitch_raw);
        pitch = pitch_raw;
        
        roll_prev = roll;
        pitch_prev = pitch;

        double roll_error = roll - desired_roll; // desired roll is 0
        double pitch_error =  pitch - desired_pitch; // desired pitch is 0
        std::cout << "roll error is " << roll_error << " and pitch error is " << pitch_error << "\n";

        double angle_correction_kp =  11.009;
        double angle_correcttion_kd = 2.0000005;
        static bool phase_switch = false;



        // set the desired shape in gamma surface
        // TODO make amplitude oscillate between 0 and 0.05 with a parameterized frequency with time as control_ticks
          // --- params ---
          const double A = 0.01;                 // max amplitude
          const double dt = 1.0 / 50.0;          // if control_ticks increments at 50 Hz (change if not)
          const double f_env = 1.0 / 5;        // envelope frequency (Hz). Period = 4s. Change as you like.
          const double eps = 1e-4;               // "close to zero" threshold in meters

          // --- time ---
          double t = control_ticks * dt;

          // --- smooth ramp up/down envelope (0 -> A -> 0) ---
          double env = 0.5 * (1.0 - std::cos(2.0 * M_PI * f_env * t));  // in [0,1]
          double amp = A * env;

          // --- edge-detect "near zero" to flip once per cycle ---
          static bool was_near_zero = true; // start true so we don't flip immediately at t=0
          bool near_zero = (amp < eps);

          if (near_zero && !was_near_zero) {
            phase_switch = !phase_switch;   // flip only when we RETURN to zero
          }
          was_near_zero = near_zero;


         this->gamma_surface->update_amplitude(amp); // 0.05 meter amplitude
          // this->gamma_surface->update_phase(M_PI * (update_count % 2));
          if (update_count % 4 == 0) {
            // roll_and_not_pitch = !roll_and_not_pitch;
          }
          
          // check if amplitude is close to zero, every time it crosses zero, we flip the phase from PI to the PD controller, and back
          if (phase_switch) {
            this->gamma_surface->update_phase(M_PI);
            // do pitch correction in one iteration i.e. angle M_PI
            // and in the next iteration do roll correction i.e. angle M_PI/2
            if (!roll_and_not_pitch) {
              // correct pitch
              this->gamma_surface->update_angle(M_PI);
            } else {
              // correct roll
              this->gamma_surface->update_angle(M_PI/2);
            }
          } else {
            // this->gamma_surface->update_angle(M_PI/2); // 90 degrees
            if (roll_and_not_pitch) {
              // correct roll
              // this->gamma_surface->update_phase(angle_correction_kp * roll_error + angle_correcttion_kd * base_link_twist(3));
              this->gamma_surface->update_phase(0.00);
            } else {
              // this->gamma_surface->update_phase(angle_correction_kp * pitch_error + angle_correcttion_kd * base_link_twist(4));
              this->gamma_surface->update_phase(0.00);
              // correct pitch              
            }
          }
          
          update_count++;
          this->gamma_surface->update_frequency(M_PI/0.4);

        //  this->shape_controller->ComputeControlStep();
        std::map<skyweave::GridIndex, double> u_dict = this->shape_controller->ComputeControlStep();
        // std::map<skyweave::GridIndex, double> u_dict = this->hover_controller->ComputeControlStep();


        // make a PD controller to make the u_dict[{0, 0}] track a desired base z position of 1.0 meter, by adding a correction term to u_dict[{0, 0}]
        double kp_hover = 10.5;
        double kd_hover = 4.01;
        double desired_base_z_position = 2.0;
        double current_base_z_position = base_link_pose(2);
        double position_error = desired_base_z_position - current_base_z_position;
        double current_base_z_velocity = base_link_twist(2);
        double velocity_error = - current_base_z_velocity;
        double pd_correction = kp_hover * position_error + kd_hover * velocity_error;
        u_dict[{0, 0}] += 0.4 + pd_correction; // only z direction thrust

         std::cout << "Thruster commands:\n";
        for (const auto& [key, value] : u_dict) {
          std::cout << " - (" << key.first << ", " << key.second << "): " << value << "\n";
        }

        // std::map<std::pair<int, int>, double> u_dict2 = {
        //     {std::make_pair(-2, -2), 0.067391},
        //     {std::make_pair(-2, -1), 0.141025},
        //     {std::make_pair(-2, 0), -0.086811},
        //     {std::make_pair(-2, 1), 0.141025},
        //     {std::make_pair(-1, 1), 0.862275},
        //     {std::make_pair(-1, 2), 0.199421},
        //     {std::make_pair(0, -2), 0.415352},
        //     {std::make_pair(0, -1), 5.970946},
        //     {std::make_pair(0, 0), 0.000000},
        //     {std::make_pair(0, 1), 5.970946},
        //     {std::make_pair(0, 2), 0.415352},
        //     {std::make_pair(1, -2), 0.199421},
        //     {std::make_pair(1, -1), 0.862275},
        //     {std::make_pair(1, 0), 0.040547},
        //     {std::make_pair(1, 1), 0.862275},
        //     {std::make_pair(1, 2), 0.199421},
        //     {std::make_pair(2, -2), 0.067391},
        //     {std::make_pair(2, -1), 0.141025},
        //     {std::make_pair(2, 0), -0.086811},
        //     {std::make_pair(2, 1), 0.141025},
        //     {std::make_pair(2, 2), 0.067391},
        // };


        this->thruster_commands.clear();
        this->thruster_commands = u_dict; 
      }      
    }


    skyweave::apply_thrusts(this->thruster_commands, this->thruster_map);
  }

  physics::ModelPtr model;
  physics::WorldPtr world;

  std::vector<physics::LinkPtr> links;
  // std::map<skyweave::GridIndex, Eigen::Vector3d> positions;
  skyweave::PositionMap positions;
  Eigen::Quaterniond centre_orientation;

  std::shared_ptr<pinocchio::Model> pin_model;
  std::shared_ptr<pinocchio::Data> pin_data;
  std::shared_ptr<skyweave_sim::Springs> springs;

  // TODO create a state estimator instance 

  int num_links = 0; // the total number of links in the model
  int side_links = 0; // the number of links on one side of the model (square root of the total number of links)
  event::ConnectionPtr updateConnection;
  common::Time lastPrintTime;
  common::Time lastControlTime;
  skyweave::FrameIndexMap frame_ids;
  std::map<skyweave::GridIndex, int> gz_links_idx_map; // the map from the grid index to the gazebo link index
  std::map<skyweave::GridIndex, std::shared_ptr<skyweave::Thruster>> thruster_map;
  std::map<skyweave::GridIndex, double> thruster_commands;
  double printRate = 2.0;
  double printPeriod = 0.5;

  double controlRate = 50.0;
  double controlPeriod = 0.02;
  int control_ticks = 0;

  std::shared_ptr<skyweave::controller::GammaSurface> gamma_surface;
  std::shared_ptr<skyweave::controller::ShapeController> shape_controller;
  std::shared_ptr<skyweave::controller::HoverController> hover_controller;

  std::shared_ptr<skyweave::StateEstimator> state_estimator;
};

GZ_REGISTER_MODEL_PLUGIN(SkyweaveLinkTracker)
}  // namespace gazebo
