#ifndef UAV_CONTROL_H
#define UAV_CONTROL_H

#include <control/mavros_adapter.h>
#include <control/pid_controller.h>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

class UavControl
{
public:
  UavControl(
    ros::NodeHandle &rosNode,
    ros::Rate rosRate,
    std::string odometryTopic,
    MavrosAdapter::Autopilot autopilot,
    bool isSimulation
  );
  void initializeMavros();
  void armingEvent();
  void localPoseEvent(geometry_msgs::Pose pose);
  void controlEffortEvent(geometry_msgs::Twist twist);

private:
  UavControl();

  void initializeParameters();
  void takeoffCallback(const std_msgs::Empty::ConstPtr& msg);
  void landCallback(const std_msgs::Empty::ConstPtr& msg);
  void rtlCallback(const std_msgs::Empty::ConstPtr& msg);
  void waypointCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void waypointNoPublishCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void resumeMissionCallback(const std_msgs::Empty::ConstPtr& msg);
  void stopCallback(const std_msgs::Empty::ConstPtr& msg);
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void gripperCallback(const std_msgs::Bool::ConstPtr& msg);
  void setWaypoint(const geometry_msgs::Pose pose);
  void logMessage(std::string message, bool isError = false) const;
  geometry_msgs::Pose transformGlobalPoseToLocalPose(const geometry_msgs::Pose& pose) const;
  geometry_msgs::Pose rotateAboutZ(const geometry_msgs::Pose& pose) const;
  std::string getNamespace() const;
  void publishState() const;

  MavrosAdapter mMavrosAdapter;
  PidController mPidController;
  ros::Subscriber mTakeoffSubscriber;
  ros::Subscriber mLandSubscriber;
  ros::Subscriber mRtlSubscriber;
  ros::Subscriber mWaypointSubscriber;
  ros::Subscriber mWaypointNoPublishSubscriber;
  ros::Subscriber mVelocitySubscriber;
  ros::Subscriber mResumeMissionSubscriber;
  ros::Subscriber mStopSubscriber;
  ros::Subscriber mOdometrySubscriber;
  ros::Subscriber mGripperSubscriber;
  ros::Publisher mWaypointArrivedPublisher;
  ros::Publisher mVelocityAlertPublisher;
  ros::Publisher mStatePublisher;
  ros::Publisher mLoggerPublisher;

  ros::NodeHandle* mNodeHandle;
  geometry_msgs::Pose mGlobalWaypoint;
  geometry_msgs::Pose mLocalWaypoint;
  nav_msgs::Odometry mCurrentOdometry;
  float mAngleOffset;
  bool mEnRouteToWaypoint;
  bool mReceivedOdometry;
  bool mInPidControl;
  bool mIsSimulation;

  tf::TransformListener tfListener;
  std::string tfBaselink;
  std::string tfArmOrigin;
};

#endif
