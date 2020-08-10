#include <perception/target_display.h>

#include <opencv2/highgui/highgui.hpp>

static const int QUEUE_SIZE = 1;
static const unsigned int MARKER_SIZE = 5;
static const std::string ARENA_WINDOW = "Image window";
static const unsigned int ARENA_PIXEL_WIDTH = 600;
static const unsigned int ARENA_PIXEL_HEIGHT = 900;
static const float ARENA_METER_WIDTH = 60.0;
static const float ARENA_METER_HEIGHT = 90.0;
static const float ARENA_PIXEL_WIDTH_PER_METER = (float)ARENA_PIXEL_WIDTH / ARENA_METER_WIDTH;
static const float ARENA_PIXEL_HEIGHT_PER_METER = (float)ARENA_PIXEL_HEIGHT / ARENA_METER_HEIGHT;
static const float PI = 3.1415926535;
const unsigned int HORIZONTAL_PIXELS = 640;
const unsigned int VERTICAL_PIXELS = 480;
const float CAMERA_HORIZONTAL_FOV = 1.0472;
const float CAMERA_VERTICAL_FOV = 0.81727571;

TargetDisplay::TargetDisplay(
  ros::NodeHandle &rosNode,
  std::string targetTopic,
  std::string odometryTopic,
  std::string arenaImage
) : mPositionHandler(rosNode, odometryTopic, HORIZONTAL_PIXELS, VERTICAL_PIXELS, CAMERA_HORIZONTAL_FOV, CAMERA_VERTICAL_FOV),
    ARENA_IMAGE(cv::imread(arenaImage, CV_LOAD_IMAGE_COLOR)) {
  this->mTargetSubscriber = rosNode.subscribe<perception::TargetArray>(targetTopic.c_str(), QUEUE_SIZE, &TargetDisplay::targetCallback, this);
}

void TargetDisplay::targetCallback(const perception::TargetArray::ConstPtr& msg) {
  this->mTargets.clear();
  for (std::vector<perception::TargetMsg>::const_iterator iter = msg->targets.begin(); iter != msg->targets.end(); ++iter) {
    this->mTargets.push_back(Target::deserialize(*iter));
  }
  this->updateWindow();
}

void TargetDisplay::updateWindow() const {
  cv::Mat rgbFrame = this->ARENA_IMAGE.clone();
  for (std::vector<Target>::const_iterator iter = this->mTargets.begin(); iter != this->mTargets.end(); ++iter) {
    cv::circle(rgbFrame, this->getImagePosition(*iter), MARKER_SIZE, this->getColorScalar(iter->getColor()), 3);
  }
  this->drawFieldOfView(rgbFrame);
  cv::imshow(ARENA_WINDOW, rgbFrame);
  cv::waitKey(3);
}

void TargetDisplay::drawFieldOfView(cv::Mat& rgbFrame) const {
  std::tuple<float, float, float> worldPosition = this->mPositionHandler.getWorldPosition();
  std::tuple<float, float> imageRange = this->mPositionHandler.getImageRange();
  float currentYaw = this->mPositionHandler.getCurrentYaw();
  std::tuple<float, float> vertex0 = this->computeVertex(std::tuple<float, float>(-std::get<0>(imageRange) / 2.0, -std::get<1>(imageRange) / 2.0), worldPosition, currentYaw);
  std::tuple<float, float> vertex1 = this->computeVertex(std::tuple<float, float>(-std::get<0>(imageRange) / 2.0, std::get<1>(imageRange) / 2.0), worldPosition, currentYaw);
  std::tuple<float, float> vertex2 = this->computeVertex(std::tuple<float, float>(std::get<0>(imageRange) / 2.0, std::get<1>(imageRange) / 2.0), worldPosition, currentYaw);
  std::tuple<float, float> vertex3 = this->computeVertex(std::tuple<float, float>(std::get<0>(imageRange) / 2.0, -std::get<1>(imageRange) / 2.0), worldPosition, currentYaw);
  this->drawFovLine(rgbFrame, vertex0, vertex1);
  this->drawFovLine(rgbFrame, vertex1, vertex2);
  this->drawFovLine(rgbFrame, vertex2, vertex3);
  this->drawFovLine(rgbFrame, vertex3, vertex0);
}

cv::Point2i TargetDisplay::getImagePosition(const Target& target) const {
  float targetX = std::get<0>(target.getPosition());
  float targetY = std::get<1>(target.getPosition());
  unsigned int x = (unsigned int)(ARENA_PIXEL_WIDTH_PER_METER * (targetX + (ARENA_METER_WIDTH / 2.0)));
  unsigned int y = (unsigned int)(ARENA_PIXEL_HEIGHT - (ARENA_PIXEL_HEIGHT_PER_METER * (targetY + (ARENA_METER_HEIGHT / 2.0))));
  return cv::Point2i(x, y);
}

cv::Scalar TargetDisplay::getColorScalar(Target::Color color) const {
  switch (color) {
  case Target::Color::RED:
    return cv::Scalar(27, 13, 252);
    break;
  case Target::Color::GREEN:
    return cv::Scalar(47, 253, 41);
    break;
  case Target::Color::BLUE:
    return cv::Scalar(251, 36, 11);
    break;
  case Target::Color::YELLOW:
    return cv::Scalar(56, 253, 255);
    break;
  case Target::Color::ORANGE:
    return cv::Scalar(35, 126, 253);
    break;
  default:
    return cv::Scalar(255, 255, 255);
    break;
  }
}

std::tuple<float, float> TargetDisplay::computeVertex(const std::tuple<float, float>& vertex, const std::tuple<float, float, float>& center, float yaw) const {
  std::tuple<float, float> transformedVertex;
  float sinTheta = sin(-yaw);
  float cosTheta = cos(-yaw);
  std::get<0>(transformedVertex) = std::get<0>(vertex) * cosTheta - std::get<1>(vertex) * sinTheta + std::get<0>(center);
  std::get<1>(transformedVertex) = std::get<1>(vertex) * cosTheta + std::get<0>(vertex) * sinTheta + std::get<1>(center);
  return transformedVertex;
}

void TargetDisplay::drawFovLine(cv::Mat& rgbFrame, const std::tuple<float, float>& vertex0, const std::tuple<float, float>& vertex1) const {
  std::tuple<int, int> imageVertex0;
  std::tuple<int, int> imageVertex1;
  std::get<0>(imageVertex0) = ARENA_PIXEL_WIDTH_PER_METER * (std::get<0>(vertex0) + (ARENA_METER_WIDTH / 2.0));
  std::get<1>(imageVertex0) = ARENA_PIXEL_HEIGHT - (ARENA_PIXEL_HEIGHT_PER_METER * (std::get<1>(vertex0) + (ARENA_METER_HEIGHT / 2.0)));
  std::get<0>(imageVertex1) = ARENA_PIXEL_WIDTH_PER_METER * (std::get<0>(vertex1) + (ARENA_METER_WIDTH / 2.0));
  std::get<1>(imageVertex1) = ARENA_PIXEL_HEIGHT - (ARENA_PIXEL_HEIGHT_PER_METER * (std::get<1>(vertex1) + (ARENA_METER_HEIGHT / 2.0)));
  cv::Point point0(std::get<0>(imageVertex0), std::get<1>(imageVertex0));
  cv::Point point1(std::get<0>(imageVertex1), std::get<1>(imageVertex1));
  cv::line(rgbFrame, point0, point1, cv::Scalar(255, 0, 255), 1);
}
