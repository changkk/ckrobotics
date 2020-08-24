#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{
  class StandFastener : public ModelPlugin
  {
    public:

      StandFastener() {
      }

      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
        // Store the pointer to the model
        this->mModel = _parent;

        // Listen to the update event. This event is broadcast every simulation iteration.
        this->mUpdateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&StandFastener::OnUpdate, this, _1));
      }

      // Called by the world update start event
      void OnUpdate(const common::UpdateInfo& _info) {
        if (!isFastened) {
          std::string targetName = "target" + this->mModel->GetName().substr(5);
          physics::WorldPtr world = this->mModel->GetWorld();
          physics::ModelPtr targetModel = world->GetModel(targetName);
          if (targetModel != NULL) {
            physics::LinkPtr standLink = this->mModel->GetLink("link_1");
            this->mModel->CreateJoint("fastener_joint", "fixed", standLink, targetModel->GetLink("link_0"));
            isFastened = true;
          }
        }
      }

    private:
      physics::ModelPtr mModel;
      event::ConnectionPtr mUpdateConnection;
      bool isFastened = false;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(StandFastener)
}
