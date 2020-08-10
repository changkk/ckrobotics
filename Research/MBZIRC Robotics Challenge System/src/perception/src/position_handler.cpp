#include <perception/position_handler.h>

#include <tf/transform_datatypes.h>

static const int QUEUE_SIZE = 1;
static const float HALF_PI = 1.57079632679;

PositionHandler::PositionHandler(
  ros::NodeHandle &rosNode,
  std::string odometryTopic,
  unsigned int horizontalPixels,
  unsigned int verticalPixels,
  float horizontalFov,
  float verticalFov
) : HORIZONTAL_PIXELS(horizontalPixels),
    VERTICAL_PIXELS(verticalPixels),
    TANGENT_OF_HALF_HORIZONTAL_FOV(tan(horizontalFov / 2.0)),
    TANGENT_OF_HALF_VERTICAL_FOV(tan(verticalFov / 2.0)) {
  this->mOdometrySubscriber = rosNode.subscribe<nav_msgs::Odometry>(odometryTopic, QUEUE_SIZE, &PositionHandler::odometryCallback, this);
  this->mCurrentYaw = 0.0;
  if (!rosNode.getParam("/hexacopter/yaw_offset", this->mOffsetYaw)) {
    this->mOffsetYaw = 0.0;
  }
}

std::tuple<float, float, float> PositionHandler::getWorldPosition() const {
  return this->mPosition;
}

std::tuple<float, float> PositionHandler::getWorldPositionFromImagePosition(const std::tuple<unsigned int, unsigned int>& imagePosition) const {
  std::tuple<float, float, float> selfWorldPosition = this->mPosition;
  std::tuple<float, float> targetWorldPosition;
  float horizontalLengthInMeters = this->getLengthInMeters(this->TANGENT_OF_HALF_HORIZONTAL_FOV, std::get<2>(selfWorldPosition));
  float verticalLengthInMeters = this->getLengthInMeters(this->TANGENT_OF_HALF_VERTICAL_FOV, std::get<2>(selfWorldPosition));
  float xPrime = this->getWorldPositionDimension(std::get<0>(imagePosition), HORIZONTAL_PIXELS, horizontalLengthInMeters);
  float yPrime = -this->getWorldPositionDimension(std::get<1>(imagePosition), VERTICAL_PIXELS, verticalLengthInMeters);
  float sinTheta = sin(this->mCurrentYaw);
  float cosTheta = cos(this->mCurrentYaw);
  float xTransformed = xPrime * cosTheta - yPrime * sinTheta;
  float yTransformed = yPrime * cosTheta + xPrime * sinTheta;
  std::get<0>(targetWorldPosition) = xTransformed + std::get<0>(selfWorldPosition);
  std::get<1>(targetWorldPosition) = yTransformed + std::get<1>(selfWorldPosition);
  return targetWorldPosition;
}

std::tuple<int, int> PositionHandler::getImagePositionFromWorldPosition(const std::tuple<float, float>& worldPosition, const std::tuple<float, float>& pixelsPerMeter) const {
  std::tuple<float, float, float> selfWorldPosition = this->mPosition;
  std::tuple<int, int> imagePosition;
  float relativeX = std::get<0>(worldPosition) - std::get<0>(selfWorldPosition);
  float relativeY = -(std::get<1>(worldPosition) - std::get<1>(selfWorldPosition));

  // TODO: Verify direction
  float sinTheta = sin(this->mCurrentYaw);
  float cosTheta = cos(this->mCurrentYaw);
  float transformedX = relativeX * cosTheta - relativeY * sinTheta;
  float transformedY = relativeY * cosTheta + relativeX * sinTheta;
  std::get<0>(imagePosition) = transformedX * std::get<0>(pixelsPerMeter) + 0.5 * HORIZONTAL_PIXELS;
  std::get<1>(imagePosition) = transformedY * std::get<1>(pixelsPerMeter) + 0.5 * VERTICAL_PIXELS;
  return imagePosition;
}

float PositionHandler::getCurrentHeight() const {
  return std::get<2>(this->mPosition);
}

double PositionHandler::getCurrentYaw() const {
  return this->mCurrentYaw;
}

std::tuple<float, float> PositionHandler::getImageRange() const {
  std::tuple<float, float> imageRange;
  float height = std::get<2>(this->mPosition);
  std::get<0>(imageRange) = this->getLengthInMeters(this->TANGENT_OF_HALF_HORIZONTAL_FOV, height);
  std::get<1>(imageRange) = this->getLengthInMeters(this->TANGENT_OF_HALF_VERTICAL_FOV, height);
  return imageRange;
}

void PositionHandler::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  this->mPosition = std::make_tuple(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  tf::Quaternion quaternion;
  double roll, pitch, yaw;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quaternion);
  tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  this->mCurrentYaw = yaw - HALF_PI + mOffsetYaw;
  // std::cout << "Yaw offset " << this->mCurrentYaw << std::endl;
}

float PositionHandler::getLengthInMeters(float tangentOfHalfFov, float height) const {
  return 2.0 * height * tangentOfHalfFov;
}

float PositionHandler::getPixelsPerMeter(unsigned int pixels, float meters) const {
  return (double)pixels / meters;
}

float PositionHandler::getWorldPositionDimension(unsigned int pixels, unsigned int maxPixels, float maxMeters) const {
  return (maxMeters * (double)pixels / (double)maxPixels) - (0.5 * maxMeters);
}
