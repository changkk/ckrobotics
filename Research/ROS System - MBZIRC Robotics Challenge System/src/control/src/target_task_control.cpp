#include <control/target_task_control.h>
#include <control/HighLevelStateMsg.h>

#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <boost/filesystem.hpp>

static const int QUEUE_SIZE = 1;
static const std::string TOPIC_CONTROL_TARGET_SEARCH = "uav_control/target/search";
static const std::string TOPIC_CONTROL_TARGET_SEARCH_COMPLETE = "uav_control/target/search/complete";
static const std::string TOPIC_CONTROL_TARGET_ACQUIRE = "uav_control/target/acquire";
static const std::string TOPIC_CONTROL_TARGET_DROP = "uav_control/target/drop";
static const std::string TOPIC_CONTROL_TARGET_CANCEL = "uav_control/target/cancel";
static const std::string TOPIC_CONTROL_TARGET_STATE = "uav_control/target/state";
static const std::string TOPIC_CONTROL_TAKEOFF = "uav_control/takeoff";
static const std::string TOPIC_CONTROL_WAYPOINT = "uav_control/waypoint";
static const std::string TOPIC_CONTROL_WAYPOINT_ARRIVED = "uav_control/waypoint/arrived";
static const std::string TOPIC_CONTROL_STOP = "uav_control/stop";
static const std::string TOPIC_MAGNETIC_GRIPPER = "uav_control/gripper";
static const std::string TOPIC_MAGNETIC_GRIPPER_ON = "magnetic_gripper/on";
static const std::string TOPIC_MAGNETIC_GRIPPER_OFF = "magnetic_gripper/off";
static const std::string TOPIC_CONTACT_SENSOR = "contact_sensors";
static const std::string TOPIC_PERCEPTION_PAUSE = "perception/pause";

TargetTaskControl::TargetTaskControl(ros::NodeHandle &rosNode, float targetHeight) :
    mFeedbackControl(rosNode, targetHeight, this, &TargetTaskControl::targetAcquiredEvent) {
  this->mTargetSearchSubscriber = rosNode.subscribe<std_msgs::Empty>(TOPIC_CONTROL_TARGET_SEARCH, QUEUE_SIZE, &TargetTaskControl::targetSearchCallback, this);
  this->mTargetAcquireSubscriber = rosNode.subscribe<geometry_msgs::Pose>(TOPIC_CONTROL_TARGET_ACQUIRE, QUEUE_SIZE, &TargetTaskControl::targetAcquireCallback, this);
  this->mTargetDropSubscriber = rosNode.subscribe<std_msgs::Empty>(TOPIC_CONTROL_TARGET_DROP, QUEUE_SIZE, &TargetTaskControl::targetDropCallback, this);
  this->mWaypointArrivedSubscriber = rosNode.subscribe<std_msgs::Empty>(TOPIC_CONTROL_WAYPOINT_ARRIVED, QUEUE_SIZE, &TargetTaskControl::waypointArrivedCallback, this);
  this->mCancelSubscriber = rosNode.subscribe<std_msgs::Bool>(TOPIC_CONTROL_TARGET_CANCEL, QUEUE_SIZE, &TargetTaskControl::cancelCallback, this);
  this->mContactSensorSubscriber = rosNode.subscribe<std_msgs::Bool>(TOPIC_CONTACT_SENSOR, QUEUE_SIZE, &TargetTaskControl::contactSensorCallback, this);
  this->mSearchCompletePublisher = rosNode.advertise<std_msgs::Empty>(TOPIC_CONTROL_TARGET_SEARCH_COMPLETE, QUEUE_SIZE);
  this->mTakeoffPublisher = rosNode.advertise<std_msgs::Empty>(TOPIC_CONTROL_TAKEOFF, QUEUE_SIZE);
  this->mWaypointPublisher = rosNode.advertise<geometry_msgs::Pose>(TOPIC_CONTROL_WAYPOINT, QUEUE_SIZE);
  this->mStopPublisher = rosNode.advertise<std_msgs::Empty>(TOPIC_CONTROL_STOP, QUEUE_SIZE);
  this->mStatePublisher = rosNode.advertise<control::HighLevelStateMsg>(TOPIC_CONTROL_TARGET_STATE, QUEUE_SIZE);
  this->mPausePerceptionPublisher = rosNode.advertise<std_msgs::Bool>(TOPIC_PERCEPTION_PAUSE, QUEUE_SIZE);
  this->mGripperPublisher = rosNode.advertise<std_msgs::Bool>(TOPIC_MAGNETIC_GRIPPER, QUEUE_SIZE);
  this->mGripperOnService = rosNode.serviceClient<std_srvs::Empty>(TOPIC_MAGNETIC_GRIPPER_ON);
  this->mGripperOffService = rosNode.serviceClient<std_srvs::Empty>(TOPIC_MAGNETIC_GRIPPER_OFF);
  this->mState = State::IDLE;
  this->mIsContactSensorEnabled = false;
  this->mCurrentPattern = this->mSearchPatterns.begin();
  if (!rosNode.getParam("/drop_zone/x_offset", std::get<0>(this->mDropPosition))) {
    std::get<0>(this->mDropPosition) = 0.0;
  }
  if (!rosNode.getParam("/drop_zone/y_offset", std::get<1>(this->mDropPosition))) {
    std::get<1>(this->mDropPosition) = 0.0;
  }
}

void TargetTaskControl::loadSearchPatterns(std::string dataDirectory) {
  boost::filesystem::path path(dataDirectory);
  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator iter(path); iter != end_itr; ++iter) {
    if (boost::filesystem::is_regular_file(iter->path())) {
      this->mSearchPatterns.push_back(SearchPattern(iter->path().string()));
    }
  }
  this->mCurrentPattern = this->mSearchPatterns.begin();
}

void TargetTaskControl::targetAcquiredEvent(const geometry_msgs::Pose& position) {
  std::cout << "TARGET ACQUIRED EVENT" << std::endl;
  this->mFeedbackControl.disable();
  geometry_msgs::Pose waypoint = position;
  waypoint.position.z = 4.0;
  this->mCollectionTaskList.clear();
  this->mWaypointPublisher.publish(waypoint);
}

void TargetTaskControl::targetSearchCallback(const std_msgs::Empty::ConstPtr& msg) {
  this->mState = State::SEARCH;
  this->mCurrentPattern->startPattern();
  this->executeControlCommand();
}

void TargetTaskControl::targetAcquireCallback(const geometry_msgs::Pose::ConstPtr& msg) {
  this->mState = State::COLLECT;
  this->mCollectionTaskList.clear();
  geometry_msgs::Pose waypoint = *msg;
  waypoint.position.z = 3.0;
  this->mCollectionTaskList.push_back(CollectionTask(CollectionTask::ACQUIRE, waypoint));
  waypoint.position.z = 5.0;
  this->mCollectionTaskList.push_back(CollectionTask(CollectionTask::WAYPOINT, waypoint));
  this->executeControlCommand();
}

void TargetTaskControl::targetDropCallback(const std_msgs::Empty::ConstPtr& msg) {
  this->mState = State::COLLECT;
  geometry_msgs::Pose waypoint;
  waypoint.position.x = std::get<0>(this->mDropPosition);
  waypoint.position.y = std::get<1>(this->mDropPosition);
  waypoint.position.z = 2.0;
  this->mCollectionTaskList.push_back(CollectionTask(CollectionTask::WAYPOINT, waypoint));
  this->mCollectionTaskList.push_back(CollectionTask(CollectionTask::DROP, waypoint));
  this->executeControlCommand();
}

void TargetTaskControl::waypointArrivedCallback(const std_msgs::Empty::ConstPtr& msg) {
  this->executeControlCommand();
}

void TargetTaskControl::cancelCallback(const std_msgs::Bool::ConstPtr& msg) {
  this->mCollectionTaskList.clear();
  this->mCurrentPattern->cancel();
  if (msg->data) {
    this->mStopPublisher.publish(std_msgs::Empty());
  }
}

void TargetTaskControl::contactSensorCallback(const std_msgs::Bool::ConstPtr& msg) {
  // Check for different value.
  if (this->mIsContactSensorEnabled != msg->data) {
    std_msgs::Bool boolMsg;
    boolMsg.data = msg->data;
    this->mPausePerceptionPublisher.publish(boolMsg);
  }
  this->mIsContactSensorEnabled = msg->data;
}

void TargetTaskControl::executeControlCommand() {
  switch (this->mState) {
  case State::SEARCH:
    if (this->mCurrentPattern->isComplete()) {
      this->mSearchCompletePublisher.publish(std_msgs::Empty());
      this->mState = State::IDLE;
    }
    else {
      this->mWaypointPublisher.publish(this->mCurrentPattern->getWaypoint());
      this->mCurrentPattern->moveNext();
    }
    break;
  case State::COLLECT:
    if (!this->mCollectionTaskList.empty()) {
      this->executeCollectionTask(this->mCollectionTaskList.front());
      this->mCollectionTaskList.pop_front();
    }
    break;
  default:
    break;
  }
  this->publishState();
}

void TargetTaskControl::executeCollectionTask(const CollectionTask& collectionTask) {
  switch (collectionTask.getAction()) {
  case CollectionTask::Action::WAYPOINT:
    this->mWaypointPublisher.publish(collectionTask.getWaypoint());
    break;
  case CollectionTask::Action::ACQUIRE:
    this->mFeedbackControl.enable(collectionTask.getWaypoint());
    this->activateGripper(true);
    break;
  case CollectionTask::Action::DROP:
    sleep(1);
    this->activateGripper(false);
    sleep(5);
    this->mWaypointPublisher.publish(collectionTask.getWaypoint());
    break;
  default:
    break;
  }
  this->publishState();
}

void TargetTaskControl::activateGripper(bool doActivate) {
  std_srvs::Empty emptyRequest;
  std_msgs::Bool boolMsg;
  boolMsg.data = doActivate;
  this->mGripperPublisher.publish(boolMsg);
  if (doActivate) {
    this->mGripperOnService.call(emptyRequest);
  }
  else {
    this->mGripperOffService.call(emptyRequest);
  }
}

void TargetTaskControl::publishState() const {
  control::HighLevelStateMsg stateMsg;
  stateMsg.mode = this->getActionString();
  stateMsg.tracking = this->mFeedbackControl.isEnabled();
  stateMsg.target_pose = this->mFeedbackControl.getPosition();
  this->mStatePublisher.publish(stateMsg);
}

std::string TargetTaskControl::getActionString() const {
  switch (this->mState) {
  case State::SEARCH:
    return "SEARCH";
    break;
  case State::COLLECT:
    return "COLLECT";
    break;
  default:
    return "IDLE";
    break;
  }
}
