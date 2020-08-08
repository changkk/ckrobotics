#ifndef FEEDBACK_CONTROL_H
#define FEEDBACK_CONTROL_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

class TargetTaskControl;

class FeedbackControl
{
public:
  FeedbackControl(
    ros::NodeHandle &rosNode,
    float targetHeight,
    TargetTaskControl* targetTaskControl,
    void (TargetTaskControl::*targetAcquiredCallback)(const geometry_msgs::Pose& waypoint)
  );
  void enable(const geometry_msgs::Pose& position);
  void disable();
  bool isEnabled() const;
  geometry_msgs::Pose getPosition() const;

private:
  FeedbackControl();
  void feedbackCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void velocityAlertCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void targetCapturedCallback(const std_msgs::Bool::ConstPtr& msg);
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
  float calculateTargetWaypointHeight(const geometry_msgs::Pose& targetPosition) const;

  ros::Subscriber mFeedbackSubscriber;
  ros::Subscriber mVelocityAlertSubscriber;
  ros::Subscriber mTargetCapturedSubscriber;
  ros::Subscriber mOdometrySubscriber;
  ros::Publisher mWaypointPublisher;
  ros::Publisher mVelocityPublisher;
  ros::ServiceClient mFeedbackOnService;
  ros::ServiceClient mFeedbackOffService;
  TargetTaskControl* mTargetTaskControl;
  void (TargetTaskControl::*mTargetAcquiredCallback)(const geometry_msgs::Pose& waypoint);
  geometry_msgs::Pose mPosition;
  nav_msgs::Odometry mOdometry;
  bool mIsEnabled;
  const float TARGET_HEIGHT;
};

#endif
