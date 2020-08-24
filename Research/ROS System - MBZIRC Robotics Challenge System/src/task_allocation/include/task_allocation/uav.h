#ifndef UAV_H
#define UAV_H

#include <ros/ros.h>

class Uav {
public:
  Uav(ros::NodeHandle &rosNode, std::string name);

  void takeOff();
  void rtl();
  void setWaypoint(float xPos, float yPos, float zPos);
  void setVelocity(float xVel, float yVel, float zVel);

private:
  Uav();

  ros::Publisher mTakeoffPublisher;
  ros::Publisher mRtlPublisher;
  ros::Publisher mWaypointPublisher;
  ros::Publisher mVelocityPublisher;

};

#endif
