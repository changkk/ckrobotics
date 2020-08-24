#ifndef TARGET_TASK_CONTROL_H
#define TARGET_TASK_CONTROL_H

#include <control/search_pattern.h>
#include <control/collection_task.h>
#include <control/feedback_control.h>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <vector>
#include <list>

class TargetTaskControl
{
public:
  TargetTaskControl(ros::NodeHandle &rosNode, float targetHeight);
  void loadSearchPatterns(std::string dataDirectory);

private:
  enum State {
    IDLE,
    SEARCH,
    COLLECT
  };

  TargetTaskControl();
  void targetAcquiredEvent(const geometry_msgs::Pose& position);

  void targetSearchCallback(const std_msgs::Empty::ConstPtr& msg);
  void targetAcquireCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void targetDropCallback(const std_msgs::Empty::ConstPtr& msg);
  void waypointArrivedCallback(const std_msgs::Empty::ConstPtr& msg);
  void cancelCallback(const std_msgs::Bool::ConstPtr& msg);
  void contactSensorCallback(const std_msgs::Bool::ConstPtr& msg);

  void executeControlCommand();
  void executeCollectionTask(const CollectionTask& collectionTask);
  void publishState() const;
  void activateGripper(bool doActivate);
  std::string getActionString() const;

  ros::Subscriber mTargetSearchSubscriber;
  ros::Subscriber mTargetAcquireSubscriber;
  ros::Subscriber mTargetDropSubscriber;
  ros::Subscriber mWaypointArrivedSubscriber;
  ros::Subscriber mCancelSubscriber;
  ros::Subscriber mContactSensorSubscriber;
  ros::Publisher mSearchCompletePublisher;
  ros::Publisher mTakeoffPublisher;
  ros::Publisher mWaypointPublisher;
  ros::Publisher mStopPublisher;
  ros::Publisher mStatePublisher;
  ros::Publisher mPausePerceptionPublisher;
  ros::Publisher mGripperPublisher;
  ros::ServiceClient mGripperOnService;
  ros::ServiceClient mGripperOffService;
  State mState;
  std::vector<SearchPattern>::iterator mCurrentPattern;
  std::vector<SearchPattern> mSearchPatterns;
  std::list<CollectionTask> mCollectionTaskList;
  std::tuple<float, float> mDropPosition;
  FeedbackControl mFeedbackControl;
  bool mIsContactSensorEnabled;
};

#endif
