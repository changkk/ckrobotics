#ifndef TARGET_DISPLAY_H
#define TARGET_DISPLAY_H

#include <perception/position_handler.h>
#include <perception/target.h>
#include <perception/TargetArray.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <vector>

class TargetDisplay {
public:
  TargetDisplay(
    ros::NodeHandle &rosNode,
    std::string targetTopic,
    std::string odometryTopic,
    std::string arenaImage
  );

private:
  TargetDisplay();
  void targetCallback(const perception::TargetArray::ConstPtr& msg);
  void updateWindow() const;
  void drawFieldOfView(cv::Mat& rgbFrame) const;
  cv::Point2i getImagePosition(const Target& target) const;
  cv::Scalar getColorScalar(Target::Color color) const;
  std::tuple<float, float> computeVertex(const std::tuple<float, float>& vertex, const std::tuple<float, float, float>& center, float yaw) const;
  void drawFovLine(cv::Mat& rgbFrame, const std::tuple<float, float>& vertex0, const std::tuple<float, float>& vertex1) const;

  std::vector<Target> mTargets;
  ros::Subscriber mTargetSubscriber;
  PositionHandler mPositionHandler;
  const cv::Mat ARENA_IMAGE;
};

#endif
