#include <perception/target_tracker.h>

#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>

static const int QUEUE_SIZE = 1;
static const float POSITION_GAIN = 2.0;
static const float CAMERA_OFFSET = 0.25;
static const std::string TOPIC_PERCEPTION_TRACK_TARGET_FEEDBACK = "perception/track_target/feedback";
static const std::string TOPIC_PERCEPTION_TRACK_TARGET_ON = "perception/track_target/on";
static const std::string TOPIC_PERCEPTION_TRACK_TARGET_OFF = "perception/track_target/off";
static const std::string TOPIC_LOGGER = "/ocs/log_message";

TargetTracker::TargetTracker(
  ros::NodeHandle& rosNode,
  const unsigned int horizontalPixels,
  const unsigned int verticalPixels
) : HORIZONTAL_PIXELS(horizontalPixels), VERTICAL_PIXELS(verticalPixels) {
  this->mNodeHandle = &rosNode;
  this->mFeedbackPublisher = this->mNodeHandle->advertise<geometry_msgs::Pose>(TOPIC_PERCEPTION_TRACK_TARGET_FEEDBACK, QUEUE_SIZE);
  this->mLoggerPublisher = this->mNodeHandle->advertise<std_msgs::String>(TOPIC_LOGGER, QUEUE_SIZE);
  this->mTrackerOnService = this->mNodeHandle->advertiseService(TOPIC_PERCEPTION_TRACK_TARGET_ON, &TargetTracker::onServiceCallback, this);
  this->mTrackerOffService = this->mNodeHandle->advertiseService(TOPIC_PERCEPTION_TRACK_TARGET_OFF, &TargetTracker::offServiceCallback, this);
  this->mStatus = false;
}

bool TargetTracker::getStatus() const {
  return this->mStatus;
}

void TargetTracker::identifyTrackedTarget(Target target, const PositionHandler& positionHandler) const {
  const std::tuple<float, float, float>& worldPosition = positionHandler.getWorldPosition();
  float currentYaw = positionHandler.getCurrentYaw();
  float positionGain = std::get<2>(worldPosition) > 2.5 ? 1.0 : POSITION_GAIN;

  std::tuple<float, float> adjustment;
  std::tuple<float, float> adjustedPosition;

  std::get<0>(adjustment) = std::get<0>(target.getPosition()) - std::get<0>(worldPosition);
  std::get<1>(adjustment) = std::get<1>(target.getPosition()) - std::get<1>(worldPosition);

  float cameraOffsetX = CAMERA_OFFSET * sin(currentYaw);
  float cameraOffsetY = CAMERA_OFFSET * cos(currentYaw);

  std::get<0>(adjustedPosition) = std::get<0>(target.getPosition()) + positionGain * std::get<0>(adjustment) - cameraOffsetX;
  std::get<1>(adjustedPosition) = std::get<1>(target.getPosition()) + positionGain * std::get<1>(adjustment) + cameraOffsetY;

  target.setPosition(adjustedPosition);

  this->publishTargetFeedback(target);
}

bool TargetTracker::onServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  if (this->mStatus) {
    ROS_WARN("Target Tracker status already 'on'");
  }
  else {
    this->mStatus = true;
    ROS_INFO("Target Tracker status: off -> on");
  }
  return true;
}

bool TargetTracker::offServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  if (this->mStatus) {
    this->mStatus = false;
    ROS_INFO("Target Tracker status: on -> off");
  }
  else {
    ROS_WARN("Target Tracker status already 'off'");
  }
  return true;
}

void TargetTracker::publishTargetFeedback(const Target& target) const {
  geometry_msgs::Pose pose;
  pose.position.x = std::get<0>(target.getPosition());
  pose.position.y = std::get<1>(target.getPosition());
  this->mFeedbackPublisher.publish(pose);
}

float TargetTracker::normalizeDistanceFromCenter(unsigned int position, unsigned int length) const {
  double halfLength = (double)length / 2.0;
  return ((double)position - halfLength) / halfLength;
}

void TargetTracker::logMessage(std::string message) const {
  std::string ns = this->mNodeHandle->getNamespace();
  ns.erase(0, 2);
  std_msgs::String logMessage;
  logMessage.data = "[" + ns + "] [TargetTracker] " + message;
  this->mLoggerPublisher.publish(logMessage);
}
