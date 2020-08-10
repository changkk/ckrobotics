/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
 * Adapted from SITL Gazebo - Gimbal Controller Plugin
*/

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include "gimbal_controller.h"

using namespace gazebo;
using namespace std;

GZ_REGISTER_MODEL_PLUGIN(GimbalController)

/////////////////////////////////////////////////
GimbalController::GimbalController()
  :status("closed")
{
  /// TODO: make these gains part of sdf xml
  this->pitchPid.Init(5, 0, 0, 0, 0, 0.3, -0.3);
  this->rollPid.Init(5, 0, 0, 0, 0, 0.3, -0.3);
  this->yawPid.Init(1.0, 0, 0, 0, 0, 1.0, -1.0);
  this->pitchCommand = 0.5* M_PI;
  this->rollCommand = 0;
  this->yawCommand = 0;
}

/////////////////////////////////////////////////
void GimbalController::Load(physics::ModelPtr _model,
  sdf::ElementPtr _sdf)
{
  this->model = _model;

  this->sdf = _sdf;

  std::string yawLinkName = "cgo3_vertical_arm_link";
  this->yawLink = this->model->GetLink(yawLinkName);
  if (this->sdf->HasElement("link_yaw"))
  {
    // Add names to map
    yawLinkName = sdf->Get<std::string>("link_yaw");
    if (this->model->GetLink(yawLinkName))
    {
      this->yawLink = this->model->GetLink(yawLinkName);
    }
    else
    {
      gzwarn << "link_yaw [" << yawLinkName << "] does not exist?\n";
    }
  }
  if (!this->yawLink)
  {
    gzerr << "GimbalController::Load ERROR! Can't get yaw link '"
          << yawLinkName << "' " << endl;
  }

  std::string rollLinkName = "cgo3_horizontal_arm_link";
  this->rollLink = this->model->GetLink(rollLinkName);
  if (this->sdf->HasElement("link_roll"))
  {
    // Add names to map
    rollLinkName = sdf->Get<std::string>("link_roll");
    if (this->model->GetLink(rollLinkName))
    {
      this->rollLink = this->model->GetLink(rollLinkName);
    }
    else
    {
      gzwarn << "link_roll [" << rollLinkName << "] does not exist?\n";
    }
  }
  if (!this->rollLink)
  {
    gzerr << "GimbalController::Load ERROR! Can't get roll link '"
          << rollLinkName << "' " << endl;
  }


  std::string pitchLinkName = "cgo3_camera_link";
  this->pitchLink = this->model->GetLink(pitchLinkName);
  if (this->sdf->HasElement("link_pitch"))
  {
    // Add names to map
    pitchLinkName = sdf->Get<std::string>("link_pitch");
    if (this->model->GetLink(pitchLinkName))
    {
      this->pitchLink = this->model->GetLink(pitchLinkName);
    }
    else
    {
      gzwarn << "link_pitch [" << pitchLinkName << "] does not exist?\n";
    }
  }
  if (!this->pitchLink)
  {
    gzerr << "GimbalController::Load ERROR! Can't get pitch link '"
          << pitchLinkName << "' " << endl;
  }


  // get imu sensor
  std::string imuSensorName = "camera_imu";
  if (this->sdf->HasElement("imu"))
  {
    // Add names to map
    imuSensorName = sdf->Get<std::string>("imu");
  }
#if GAZEBO_MAJOR_VERSION >= 7
  this->imuSensor = std::static_pointer_cast<sensors::ImuSensor>(
    sensors::SensorManager::Instance()->GetSensor(imuSensorName));
#elif GAZEBO_MAJOR_VERSION >= 6
  this->imuSensor = boost::static_pointer_cast<sensors::ImuSensor>(
    sensors::SensorManager::Instance()->GetSensor(imuSensorName));
#endif
  if (!this->imuSensor)
  {
    gzerr << "GimbalController::Load ERROR! Can't get imu sensor '"
          << imuSensorName << "' " << endl;
  }
}

/////////////////////////////////////////////////
void GimbalController::Init()
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  this->lastUpdateTime = this->model->GetWorld()->GetSimTime();

  // receive pitch command via gz transport
  std::string pitchTopic = std::string("~/") +  this->model->GetName() +
    "/gimbal_pitch_cmd";
  this->pitchSub = this->node->Subscribe(pitchTopic,
     &GimbalController::OnPitchStringMsg, this);
  // receive roll command via gz transport
  std::string rollTopic = std::string("~/") +  this->model->GetName() +
    "/gimbal_roll_cmd";
  this->rollSub = this->node->Subscribe(rollTopic,
     &GimbalController::OnRollStringMsg, this);
  // receive yaw command via gz transport
  std::string yawTopic = std::string("~/") +  this->model->GetName() +
    "/gimbal_yaw_cmd";
  this->yawSub = this->node->Subscribe(yawTopic,
     &GimbalController::OnYawStringMsg, this);

  // plugin update
  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GimbalController::OnUpdate, this)));

  // publish pitch status via gz transport
  pitchTopic = std::string("~/") +  this->model->GetName()
    + "/gimbal_pitch_status";
#if GAZEBO_MAJOR_VERSION >= 7 && GAZEBO_MINOR_VERSION >= 4
  /// only gazebo 7.4 and above support Any
  this->pitchPub = node->Advertise<gazebo::msgs::Any>(pitchTopic);
#else
  this->pitchPub = node->Advertise<gazebo::msgs::GzString>(pitchTopic);
#endif

  // publish roll status via gz transport
  rollTopic = std::string("~/") +  this->model->GetName()
    + "/gimbal_roll_status";
#if GAZEBO_MAJOR_VERSION >= 7 && GAZEBO_MINOR_VERSION >= 4
  /// only gazebo 7.4 and above support Any
  this->rollPub = node->Advertise<gazebo::msgs::Any>(rollTopic);
#else
  this->rollPub = node->Advertise<gazebo::msgs::GzString>(rollTopic);
#endif

  // publish yaw status via gz transport
  yawTopic = std::string("~/") +  this->model->GetName()
    + "/gimbal_yaw_status";
#if GAZEBO_MAJOR_VERSION >= 7 && GAZEBO_MINOR_VERSION >= 4
  /// only gazebo 7.4 and above support Any
  this->yawPub = node->Advertise<gazebo::msgs::Any>(yawTopic);
#else
  this->yawPub = node->Advertise<gazebo::msgs::GzString>(yawTopic);
#endif

  gzmsg << "GimbalController::Init" << std::endl;
}

#if GAZEBO_MAJOR_VERSION >= 7 && GAZEBO_MINOR_VERSION >= 4
/// only gazebo 7.4 and above support Any
/////////////////////////////////////////////////
void GimbalController::OnPitchStringMsg(ConstAnyPtr &_msg)
{
//  gzdbg << "pitch command received " << _msg->double_value() << std::endl;
  this->pitchCommand = _msg->double_value();
}

/////////////////////////////////////////////////
void GimbalController::OnRollStringMsg(ConstAnyPtr &_msg)
{
//  gzdbg << "roll command received " << _msg->double_value() << std::endl;
  this->rollCommand = _msg->double_value();
}

/////////////////////////////////////////////////
void GimbalController::OnYawStringMsg(ConstAnyPtr &_msg)
{
//  gzdbg << "yaw command received " << _msg->double_value() << std::endl;
  this->yawCommand = _msg->double_value();
}
#else
/////////////////////////////////////////////////
void GimbalController::OnPitchStringMsg(ConstGzStringPtr &_msg)
{
//  gzdbg << "pitch command received " << _msg->data() << std::endl;
  this->pitchCommand = atof(_msg->data().c_str());
}

/////////////////////////////////////////////////
void GimbalController::OnRollStringMsg(ConstGzStringPtr &_msg)
{
//  gzdbg << "roll command received " << _msg->data() << std::endl;
  this->rollCommand = atof(_msg->data().c_str());
}

/////////////////////////////////////////////////
void GimbalController::OnYawStringMsg(ConstGzStringPtr &_msg)
{
//  gzdbg << "yaw command received " << _msg->data() << std::endl;
  this->yawCommand = atof(_msg->data().c_str());
}
#endif

/////////////////////////////////////////////////
ignition::math::Vector3d GimbalController::ThreeAxisRot(
  double r11, double r12, double r21, double r31, double r32)
{
  return ignition::math::Vector3d(
    atan2( r31, r32 ),
    asin ( r21 ),
    atan2( r11, r12 ));
}

/////////////////////////////////////////////////
ignition::math::Vector3d GimbalController::QtoZXY(
  const ignition::math::Quaterniond &_q)
{
  // taken from
  // http://bediyap.com/programming/convert-quaternion-to-euler-rotations/
  // case zxy:
  ignition::math::Vector3d result = this->ThreeAxisRot(
    -2*(_q.X()*_q.Y() - _q.W()*_q.Z()),
    _q.W()*_q.W() - _q.X()*_q.X() + _q.Y()*_q.Y() - _q.Z()*_q.Z(),
    2*(_q.Y()*_q.Z() + _q.W()*_q.X()),
    -2*(_q.X()*_q.Z() - _q.W()*_q.Y()),
    _q.W()*_q.W() - _q.X()*_q.X() - _q.Y()*_q.Y() + _q.Z()*_q.Z());
  return result;
}

/////////////////////////////////////////////////
void GimbalController::OnUpdate()
{
  if (!this->pitchLink || !this->rollLink || !this->yawLink)
    return;

  common::Time time = this->model->GetWorld()->GetSimTime();
  if (time < this->lastUpdateTime)
  {
    gzerr << "time reset event\n";
    this->lastUpdateTime = time;
    return;
  }
  else if (time > this->lastUpdateTime)
  {
    double dt = (this->lastUpdateTime - time).Double();

    math::Pose rollLinkPose = this->rollLink->GetWorldPose();
    math::Pose pitchLinkPose = this->pitchLink->GetWorldPose();
    math::Pose yawLinkPose = this->yawLink->GetWorldPose();

    rollLinkPose.rot.x = 0.0;
    rollLinkPose.rot.y = 0.0;
    rollLinkPose.rot.z = 0.0;
    rollLinkPose.rot.w = 1.0;

    pitchLinkPose.rot.x = 0.0;
    pitchLinkPose.rot.y = 0.0;
    pitchLinkPose.rot.z = 0.0;
    pitchLinkPose.rot.w = 1.0;

    yawLinkPose.rot.x = 0.0;
    yawLinkPose.rot.y = 0.0;
    yawLinkPose.rot.z = 0.0;
    yawLinkPose.rot.w = 1.0;

    this->rollLink->SetWorldPose(rollLinkPose);
    this->pitchLink->SetWorldPose(pitchLinkPose);
    this->yawLink->SetWorldPose(yawLinkPose);

    this->lastUpdateTime = time;
  }
}

/////////////////////////////////////////////////
double GimbalController::NormalizeAbout(double _angle, double reference)
{
  double diff = _angle - reference;
  // normalize diff about (-pi, pi], then add reference
  while (diff <= -M_PI)
  {
    diff += 2.0*M_PI;
  }
  while (diff > M_PI)
  {
    diff -= 2.0*M_PI;
  }
  return diff + reference;
}

/////////////////////////////////////////////////
double GimbalController::ShortestAngularDistance(double _from, double _to)
{
  return this->NormalizeAbout(_to, _from) - _from;
}
