#ifndef TARGET_TRACKER_H
#define TARGET_TRACKER_H

#include <perception/image_object.h>
#include <perception/position_handler.h>
#include <perception/target.h>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

class TargetTracker
{
public:
  TargetTracker(
    ros::NodeHandle& rosNode,
    const unsigned int horizontalPixels,
    const unsigned int verticalPixels
  );
  bool getStatus() const;
  void identifyTrackedTarget(Target target, const PositionHandler& positionHandler) const;

private:
  TargetTracker();

  bool onServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool offServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  void publishTargetFeedback(const Target& target) const;
  float normalizeDistanceFromCenter(unsigned int position, unsigned int length) const;
  void logMessage(std::string message) const;

  ros::NodeHandle* mNodeHandle;
  ros::Publisher mFeedbackPublisher;
  ros::Publisher mLoggerPublisher;
  ros::ServiceServer mTrackerOnService;
  ros::ServiceServer mTrackerOffService;
  bool mStatus;
  const unsigned int HORIZONTAL_PIXELS;
  const unsigned int VERTICAL_PIXELS;
};

#endif
