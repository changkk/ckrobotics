#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <random>

namespace gazebo {
  class RandomWalk : public ModelPlugin {
    public:

      RandomWalk(
      ) : mRandomEngine(std::random_device()()), mSpeedChangeDistribution(-MAX_SPEED_CHANGE, MAX_SPEED_CHANGE), mTestDistribution(0.0, 1.0) {
        std::uniform_real_distribution<float> initialSpeedDistribution(-MAX_SPEED, MAX_SPEED);
        this->xVelocity = initialSpeedDistribution(this->mRandomEngine);
        this->yVelocity = initialSpeedDistribution(this->mRandomEngine);
      }

      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
        // Store the pointer to the model
        this->mModel = _parent;

        // Listen to the update event. This event is broadcast every simulation iteration.
        this->mUpdateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&RandomWalk::OnUpdate, this, _1));

        this->mInitialPose = this->mModel->GetWorldPose();
      }

      // Called by the world update start event
      void OnUpdate(const common::UpdateInfo& _info) {
        physics::JointPtr fastenerJoint = this->mModel->GetJoint("fastener_joint");
        if (fastenerJoint == NULL) {
          this->mModel->SetLinearVel(math::Vector3(0.0, 0.0, 0.0));
        }
        else {
          // Apply a small linear velocity to the model sometimes.
          if (this->mTestDistribution(mRandomEngine) > 0.9) {
            math::Pose currentPose = this->mModel->GetWorldPose();
            bool didChangeVelocity = false;
            if (this->mTestDistribution(mRandomEngine) > 0.5) {
              float distanceFromStart = currentPose.pos.x - this->mInitialPose.pos.x;
              float distanceFromBoundary = BOUNDARY_X - fabs(currentPose.pos.x);
              this->xVelocity = this->updateSpeed(this->xVelocity, distanceFromStart, distanceFromBoundary);
              didChangeVelocity = true;
            }
            if (this->mTestDistribution(mRandomEngine) > 0.5) {
              float distanceFromStart = currentPose.pos.y - this->mInitialPose.pos.y;
              float distanceFromBoundary = BOUNDARY_Y - fabs(currentPose.pos.y);
              this->yVelocity = this->updateSpeed(this->yVelocity, distanceFromStart, distanceFromBoundary);
              didChangeVelocity = true;
            }
            if (didChangeVelocity) {
              this->mModel->SetLinearVel(math::Vector3(this->xVelocity, this->yVelocity, 0.0));
            }
          }
        }
      }

    private:

      static constexpr const float MAX_SPEED = 1.38889;
      static constexpr const float MAX_SPEED_CHANGE = 0.1;
      static constexpr const float MOVEMENT_RADIUS = 3.0;
      static constexpr const float BOUNDARY_X = 30.0;
      static constexpr const float BOUNDARY_Y = 45.0;
      static constexpr const float MIN_BOUNDARY_DISTANCE = 0.2;
      physics::ModelPtr mModel;
      event::ConnectionPtr mUpdateConnection;
      math::Pose mInitialPose;
      std::default_random_engine mRandomEngine;
      std::uniform_real_distribution<float> mSpeedChangeDistribution;
      std::uniform_real_distribution<float> mTestDistribution;
      float xVelocity;
      float yVelocity;

      float updateSpeed(float speed, float distanceFromStart, float distanceFromBoundary) {
        float changeInSpeed = this->mSpeedChangeDistribution(this->mRandomEngine);
        speed += changeInSpeed;
        if (fabs(distanceFromBoundary) < MIN_BOUNDARY_DISTANCE) {
          speed = distanceFromBoundary > 0.0 ? -MAX_SPEED / 4.0 : MAX_SPEED / 4.0;
        }
        else if (fabs(distanceFromStart) > MOVEMENT_RADIUS) {
          speed = distanceFromStart > 0.0 ? -MAX_SPEED / 4.0 : MAX_SPEED / 4.0;
        }
        else if (fabs(speed) > MAX_SPEED) {
          speed -= 2.0 * changeInSpeed;
        }
        return speed;
      }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RandomWalk)
}
