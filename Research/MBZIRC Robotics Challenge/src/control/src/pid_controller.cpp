#include <control/pid_controller.h>

#include <sstream>

static const int QUEUE_SIZE = 1;
static const std::string TOPIC_CONTROL_EFFORT_X = "pid/x_position/control_effort";
static const std::string TOPIC_CONTROL_EFFORT_Y = "pid/y_position/control_effort";
static const std::string TOPIC_CONTROL_EFFORT_Z = "pid/z_position/control_effort";
static const std::string TOPIC_CONTROL_EFFORT_YAW = "pid/yaw/control_effort";
static const std::string TOPIC_STATE_X = "pid/x_position/state";
static const std::string TOPIC_STATE_Y = "pid/y_position/state";
static const std::string TOPIC_STATE_Z = "pid/z_position/state";
static const std::string TOPIC_STATE_YAW = "pid/yaw/state";
static const std::string TOPIC_SETPOINT_X = "pid/x_position/setpoint";
static const std::string TOPIC_SETPOINT_Y = "pid/y_position/setpoint";
static const std::string TOPIC_SETPOINT_Z = "pid/z_position/setpoint";
static const std::string TOPIC_SETPOINT_YAW = "pid/yaw/setpoint";

PidController::PidController(
  ros::NodeHandle &rosNode,
  UavControl* uavControl,
  void (UavControl::*controlEffortCallback)(geometry_msgs::Twist twist)
) {
  this->mNodeHandle = &rosNode;
  this->mXControlEffortSubscriber = this->mNodeHandle->subscribe<std_msgs::Float64>(TOPIC_CONTROL_EFFORT_X, QUEUE_SIZE, &PidController::xControlEffortCallback, this);
  this->mYControlEffortSubscriber = this->mNodeHandle->subscribe<std_msgs::Float64>(TOPIC_CONTROL_EFFORT_Y, QUEUE_SIZE, &PidController::yControlEffortCallback, this);
  this->mZControlEffortSubscriber = this->mNodeHandle->subscribe<std_msgs::Float64>(TOPIC_CONTROL_EFFORT_Z, QUEUE_SIZE, &PidController::zControlEffortCallback, this);
  this->mYawControlEffortSubscriber = this->mNodeHandle->subscribe<std_msgs::Float64>(TOPIC_CONTROL_EFFORT_YAW, QUEUE_SIZE, &PidController::yawControlEffortCallback, this);
  this->mXStatePublisher = this->mNodeHandle->advertise<std_msgs::Float64>(TOPIC_STATE_X, QUEUE_SIZE);
  this->mYStatePublisher = this->mNodeHandle->advertise<std_msgs::Float64>(TOPIC_STATE_Y, QUEUE_SIZE);
  this->mZStatePublisher = this->mNodeHandle->advertise<std_msgs::Float64>(TOPIC_STATE_Z, QUEUE_SIZE);
  this->mYawStatePublisher = this->mNodeHandle->advertise<std_msgs::Float64>(TOPIC_STATE_YAW, QUEUE_SIZE);
  this->mXSetpointPublisher = this->mNodeHandle->advertise<std_msgs::Float64>(TOPIC_SETPOINT_X, QUEUE_SIZE);
  this->mYSetpointPublisher = this->mNodeHandle->advertise<std_msgs::Float64>(TOPIC_SETPOINT_Y, QUEUE_SIZE);
  this->mZSetpointPublisher = this->mNodeHandle->advertise<std_msgs::Float64>(TOPIC_SETPOINT_Z, QUEUE_SIZE);
  this->mYawSetpointPublisher = this->mNodeHandle->advertise<std_msgs::Float64>(TOPIC_SETPOINT_YAW, QUEUE_SIZE);
  this->mUavControl = uavControl;
  this->mControlEffortCallback = controlEffortCallback;
}

void PidController::setState(const geometry_msgs::Pose& pose) const {
  std_msgs::Float64 msg;
  msg.data = pose.position.x;
  this->mXStatePublisher.publish(msg);
  msg.data = pose.position.y;
  this->mYStatePublisher.publish(msg);
  msg.data = pose.position.z;
  this->mZStatePublisher.publish(msg);
  msg.data = pose.orientation.z;
  this->mYawStatePublisher.publish(msg);
}

void PidController::setSetpoint(const geometry_msgs::Pose& pose) const {
  std_msgs::Float64 msg;
  msg.data = pose.position.x;
  this->mXSetpointPublisher.publish(msg);
  msg.data = pose.position.y;
  this->mYSetpointPublisher.publish(msg);
  msg.data = pose.position.z;
  this->mZSetpointPublisher.publish(msg);
  msg.data = pose.orientation.z;
  this->mYawSetpointPublisher.publish(msg);
}

void PidController::xControlEffortCallback(const std_msgs::Float64::ConstPtr& msg) {
  this->mVelocity.linear.x = msg->data;
  (*this->mUavControl.*this->mControlEffortCallback)(this->mVelocity);
}

void PidController::yControlEffortCallback(const std_msgs::Float64::ConstPtr& msg) {
  this->mVelocity.linear.y = msg->data;
}

void PidController::zControlEffortCallback(const std_msgs::Float64::ConstPtr& msg) {
  this->mVelocity.linear.z = msg->data;
}

void PidController::yawControlEffortCallback(const std_msgs::Float64::ConstPtr& msg) {
  // this->mVelocity.angular.z = msg->data;
}
