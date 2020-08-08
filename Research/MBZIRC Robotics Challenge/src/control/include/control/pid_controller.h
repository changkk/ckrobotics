#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

class UavControl;

class PidController
{
public:
  PidController(
    ros::NodeHandle &rosNode,
    UavControl* uavControl,
    void (UavControl::*controlEffortCallback)(geometry_msgs::Twist twist)
  );
  void setState(const geometry_msgs::Pose& pose) const;
  void setSetpoint(const geometry_msgs::Pose& pose) const;

private:
  PidController();
  void xControlEffortCallback(const std_msgs::Float64::ConstPtr& msg);
  void yControlEffortCallback(const std_msgs::Float64::ConstPtr& msg);
  void zControlEffortCallback(const std_msgs::Float64::ConstPtr& msg);
  void yawControlEffortCallback(const std_msgs::Float64::ConstPtr& msg);

  ros::NodeHandle* mNodeHandle;
  ros::Subscriber mXControlEffortSubscriber;
  ros::Subscriber mYControlEffortSubscriber;
  ros::Subscriber mZControlEffortSubscriber;
  ros::Subscriber mYawControlEffortSubscriber;
  ros::Publisher mXStatePublisher;
  ros::Publisher mYStatePublisher;
  ros::Publisher mZStatePublisher;
  ros::Publisher mYawStatePublisher;
  ros::Publisher mXSetpointPublisher;
  ros::Publisher mYSetpointPublisher;
  ros::Publisher mZSetpointPublisher;
  ros::Publisher mYawSetpointPublisher;
  geometry_msgs::Twist mVelocity;
  UavControl* mUavControl;
  void (UavControl::*mControlEffortCallback)(geometry_msgs::Twist twist);
};

#endif
