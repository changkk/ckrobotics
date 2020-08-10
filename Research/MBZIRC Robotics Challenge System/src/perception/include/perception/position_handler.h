#ifndef POSITION_HANDLER_H
#define POSITION_HANDLER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tuple>

class PositionHandler {
public:
  PositionHandler(
    ros::NodeHandle &rosNode,
    std::string odometryTopic,
    unsigned int horizontalPixels,
    unsigned int verticalPixels,
    float horizontalFov,
    float verticalFov
  );
  std::tuple<float, float, float> getWorldPosition() const;
  std::tuple<float, float> getWorldPositionFromImagePosition(const std::tuple<unsigned int, unsigned int>& imagePositio) const;
  std::tuple<int, int> getImagePositionFromWorldPosition(const std::tuple<float, float>& worldPosition, const std::tuple<float, float>& pixelsPerMeter) const;
  float getCurrentHeight() const;
  double getCurrentYaw() const;
  std::tuple<float, float> getImageRange() const;

private:
  PositionHandler();
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
  float getLengthInMeters(float fov, float height) const;
  float getPixelsPerMeter(unsigned int pixels, float meters) const;
  float getWorldPositionDimension(unsigned int pixels, unsigned int maxPixels, float maxMeters) const;

  ros::Subscriber mOdometrySubscriber;
  std::tuple<float, float, float> mPosition;
  double mCurrentYaw;
  double mOffsetYaw;
  const unsigned int HORIZONTAL_PIXELS;
  const unsigned int VERTICAL_PIXELS;
  const float TANGENT_OF_HALF_HORIZONTAL_FOV;
  const float TANGENT_OF_HALF_VERTICAL_FOV;
};

#endif
