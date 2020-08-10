#include <control/feedback_control.h>

#include <std_srvs/Empty.h>

static const int QUEUE_SIZE = 1;
static const std::string TOPIC_PERCEPTION_TRACK_TARGET_FEEDBACK = "perception/track_target/feedback";
static const std::string TOPIC_PERCEPTION_TRACK_TARGET_ON = "perception/track_target/on";
static const std::string TOPIC_PERCEPTION_TRACK_TARGET_OFF = "perception/track_target/off";
static const std::string TOPIC_CONTROL_WAYPOINT = "uav_control/waypoint";
static const std::string TOPIC_CONTROL_VELOCITY = "uav_control/velocity";
static const std::string TOPIC_CONTROL_VELOCITY_ALERT = "uav_control/velocity/alert";
static const std::string TOPIC_LOCALIZER_ODOMETRY = "localizer/global_odom";
static const std::string TOPIC_GRIPPER_CAPTURED = "magnetic_gripper/target_captured";

FeedbackControl::FeedbackControl(
  ros::NodeHandle &rosNode,
  float targetHeight,
  TargetTaskControl* targetTaskControl,
  void (TargetTaskControl::*targetAcquiredCallback)(const geometry_msgs::Pose& waypoint)
) : TARGET_HEIGHT(targetHeight) {
  this->mTargetTaskControl = targetTaskControl;
  this->mTargetAcquiredCallback = targetAcquiredCallback;
  this->mFeedbackSubscriber = rosNode.subscribe<geometry_msgs::Pose>(TOPIC_PERCEPTION_TRACK_TARGET_FEEDBACK, QUEUE_SIZE, &FeedbackControl::feedbackCallback, this);
  this->mVelocityAlertSubscriber = rosNode.subscribe<geometry_msgs::Pose>(TOPIC_CONTROL_VELOCITY_ALERT, QUEUE_SIZE, &FeedbackControl::velocityAlertCallback, this);
  this->mTargetCapturedSubscriber = rosNode.subscribe<std_msgs::Bool>(TOPIC_GRIPPER_CAPTURED, QUEUE_SIZE, &FeedbackControl::targetCapturedCallback, this);
  this->mOdometrySubscriber = rosNode.subscribe<nav_msgs::Odometry>(TOPIC_LOCALIZER_ODOMETRY, QUEUE_SIZE, &FeedbackControl::odometryCallback, this);
  this->mWaypointPublisher = rosNode.advertise<geometry_msgs::Pose>(TOPIC_CONTROL_WAYPOINT, QUEUE_SIZE);
  this->mVelocityPublisher = rosNode.advertise<geometry_msgs::Twist>(TOPIC_CONTROL_VELOCITY, QUEUE_SIZE);
  this->mFeedbackOnService = rosNode.serviceClient<std_srvs::Empty>(TOPIC_PERCEPTION_TRACK_TARGET_ON);
  this->mFeedbackOffService = rosNode.serviceClient<std_srvs::Empty>(TOPIC_PERCEPTION_TRACK_TARGET_OFF);
  this->mIsEnabled = false;
}

void FeedbackControl::enable(const geometry_msgs::Pose& position) {
  std_srvs::Empty emptyRequest;
  this->mFeedbackOnService.call(emptyRequest);
  this->mPosition = position;
  this->mIsEnabled = true;
}

void FeedbackControl::disable() {
  std_srvs::Empty emptyRequest;
  this->mFeedbackOffService.call(emptyRequest);
  this->mIsEnabled = false;
}

bool FeedbackControl::isEnabled() const {
  return this->mIsEnabled;
}

void FeedbackControl::feedbackCallback(const geometry_msgs::Pose::ConstPtr& msg) {
  if (this->mIsEnabled) {
    geometry_msgs::Pose waypoint = *msg;
    waypoint.position.z = this->calculateTargetWaypointHeight(waypoint);
    this->mWaypointPublisher.publish(waypoint);
  }
}

void FeedbackControl::velocityAlertCallback(const geometry_msgs::Pose::ConstPtr& msg) {
  this->mPosition = *msg;
}

void FeedbackControl::targetCapturedCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data) {
    (*this->mTargetTaskControl.*this->mTargetAcquiredCallback)(this->mPosition);
  }
}

void FeedbackControl::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  this->mOdometry = *msg;
}

geometry_msgs::Pose FeedbackControl::getPosition() const {
  return this->mPosition;
}

float FeedbackControl::calculateTargetWaypointHeight(const geometry_msgs::Pose& targetPosition) const {
  geometry_msgs::Pose currentPosition = this->mOdometry.pose.pose;
  float height = TARGET_HEIGHT;
  float bufferHeight = 0.0;
  if (currentPosition.position.z > 7.0) {
    bufferHeight = 2.0;
  }
  else if (currentPosition.position.z > 5.0) {
    bufferHeight = 1.0;
  }
  else if (currentPosition.position.z > 3.0) {
    bufferHeight = 0.5;
  }
  float horizontalDistance = sqrt(pow(targetPosition.position.x - currentPosition.position.x, 2) + pow(targetPosition.position.y - currentPosition.position.y, 2));
  height += bufferHeight;
  height += horizontalDistance > 0.5 ? horizontalDistance : 0.0;
  return height;
}
