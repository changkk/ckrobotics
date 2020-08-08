#ifndef LANDING_GEAR_H
#define LANDING_GEAR_H

#include <ros/ros.h>

class LandingGear {
public:
  LandingGear();
  void initialize(ros::NodeHandle& rosNode, ros::Publisher& rcOverridePublisher, ros::Publisher& loggerPublisher);
  bool retract(float height);
  bool extend(bool attemptingToRetract);
  bool getIsRetracted() const;
  int getRetractAttempts() const;

private:
  void publishMessage(bool retract) const;
  void logMessage(std::string message) const;

  bool mIsRetracted;
  int mRetractAttempts;
  ros::Time mLastRequest;
  ros::NodeHandle mNodeHandle;
  ros::Publisher mRcOverridePublisher;
  ros::Publisher mLoggerPublisher;
};

#endif
