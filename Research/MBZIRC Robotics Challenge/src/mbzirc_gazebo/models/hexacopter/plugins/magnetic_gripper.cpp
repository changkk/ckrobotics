#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <stdio.h>

namespace gazebo
{
  class MagneticGripper : public ModelPlugin
  {
    public:

      MagneticGripper() {
        this->mGripJoint = NULL;
        this->mRosNode = NULL;
        this->mStatus = false;
      }

      ~MagneticGripper() {
        if (this->mRosNode) {
          delete this->mRosNode;
        }
      }

      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
        // Store the pointer to the model
        this->mModel = _parent;

        // Listen to the update event. This event is broadcast every simulation iteration.
        this->mUpdateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&MagneticGripper::OnUpdate, this, _1));

        std::string robotNamespace = _sdf->GetElement("robotNamespace")->Get<std::string>() + NAMESPACE;
        this->mRosNode = new ros::NodeHandle(robotNamespace);
        this->mGripperLink = this->mModel->GetLink("magnetic_gripper_link");
        this->mTargetCapturedPublisher = this->mRosNode->advertise<std_msgs::Bool>(robotNamespace + "target_captured", 10);
        this->mOnService = this->mRosNode->advertiseService(robotNamespace + "on", &MagneticGripper::onServiceCallback, this);
        this->mOffService = this->mRosNode->advertiseService(robotNamespace + "off", &MagneticGripper::offServiceCallback, this);
      }

      // Called by the world update start event
      void OnUpdate(const common::UpdateInfo & _info) {
        if (this->mStatus && this->mGripJoint == NULL) {
          this->acquireTarget();
        }
        else if (!this->mStatus && this->mGripJoint != NULL) {
          this->mModel->RemoveJoint(this->mGripJoint->GetName());
          this->mGripJoint = NULL;
          std_msgs::Bool boolMsg;
          boolMsg.data = false;
          this->mTargetCapturedPublisher.publish(boolMsg);
        }
      }

    private:

      void acquireTarget() {
        physics::WorldPtr world = this->mModel->GetWorld();
        physics::Model_V models = world->GetModels();
        for (size_t i = 0; i < models.size(); ++i) {
          if (models[i]->GetName().find("target") != std::string::npos && models[i]->GetName() != "target_bin") {
            physics::ModelPtr targetModel = models[i];
            math::Pose gripperPose = this->mGripperLink->GetWorldPose();
            math::Pose targetPose = targetModel->GetWorldPose();
            if (fabs(gripperPose.pos.x - targetPose.pos.x) < CLOSE_ENOUGH_X_AND_Y &&
                fabs(gripperPose.pos.y - targetPose.pos.y) < CLOSE_ENOUGH_X_AND_Y &&
                fabs(gripperPose.pos.z - targetPose.pos.z) < CLOSE_ENOUGH_Z) {
              this->mGripJoint = this->mModel->CreateJoint("grip_joint", "fixed", this->mGripperLink, targetModel->GetLink("link_0"));
              this->removeFastenerJoint(targetModel, world);
              std_msgs::Bool boolMsg;
              boolMsg.data = true;
              this->mTargetCapturedPublisher.publish(boolMsg);
            }
          }
        }
      }

      void removeFastenerJoint(const physics::ModelPtr& targetModel, const physics::WorldPtr& world) {
        std::string standName = "stand" + targetModel->GetName().substr(6);
        physics::ModelPtr standModel = world->GetModel(standName);
        standModel->RemoveJoint("fastener_joint");
      }

      bool onServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        if (this->mStatus) {
          ROS_WARN("Magnetic Gripper: already status is 'on'");
        }
        else {
          this->mStatus = true;
          ROS_INFO("Magnetic Gripper: status: off -> on");
        }
        return true;
      }

      bool offServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        if (this->mStatus) {
          this->mStatus = false;
          ROS_INFO("Magnetic Gripper: status: on -> off");
        } else {
          ROS_WARN("Magnetic Gripper: already status is 'off'");
        }
        return true;
      }

      const std::string NAMESPACE = "magnetic_gripper/";
      const float CLOSE_ENOUGH_X_AND_Y = 0.1;
      const float CLOSE_ENOUGH_Z = 0.005;
      physics::ModelPtr mModel;
      physics::LinkPtr mGripperLink;
      physics::JointPtr mGripJoint;
      event::ConnectionPtr mUpdateConnection;
      ros::NodeHandle* mRosNode;
      ros::Publisher mTargetCapturedPublisher;
      ros::ServiceServer mOnService;
      ros::ServiceServer mOffService;
      bool mStatus;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MagneticGripper)
}
