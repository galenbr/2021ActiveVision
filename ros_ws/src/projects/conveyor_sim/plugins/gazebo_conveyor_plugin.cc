#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo {
  class ConveyorPlugin : public ModelPlugin {
  public:
    ConveyorPlugin() : ModelPlugin() {}

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
      model = _model;
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ConveyorPlugin::OnUpdate, this));
    }

    // Callback on model startup
    void OnUpdate() {
      this->model->SetLinearVel(ignition::math::Vector3d(0, 0.3, 0)); // Hardcoded speed for now
      curPose = this->model->GetLink()->WorldPose();
      if(curPose.Pos().Y() >= 1) { // Hardcoded limit for now
	curPose.Pos().Y() = 0.0;	// Hardcoded reset position for now
	this->model->SetLinkWorldPose(curPose, "canonical");
      }
    }
  private:
    ignition::math::Pose3d curPose;
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
  };

  GZ_REGISTER_MODEL_PLUGIN(ConveyorPlugin);
}
