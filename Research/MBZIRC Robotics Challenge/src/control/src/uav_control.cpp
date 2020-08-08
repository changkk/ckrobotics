#include <control/uav_control.h>
#include <control/StateMsg.h>

#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <sstream>

static const int QUEUE_SIZE = 1;
static const std::string TF_BASELINK_SUFFIX = "_base_link";
static const std::string TF_ARM_ORIGIN_SUFFIX = "_arm_origin";
static const std::string TF_GLOBAL_ORIGIN = "global_map";
static const std::string TOPIC_CONTROL_TAKEOFF = "uav_control/takeoff";
static const std::string TOPIC_CONTROL_LAND = "uav_control/land";
static const std::string TOPIC_CONTROL_RTL = "uav_control/rtl";
static const std::string TOPIC_CONTROL_WAYPOINT = "uav_control/waypoint";
static const std::string TOPIC_CONTROL_WAYPOINT_NO_PUBLISH = "uav_control/pseudo_waypoint";
static const std::string TOPIC_CONTROL_WAYPOINT_ARRIVED = "uav_control/waypoint/arrived";
static const std::string TOPIC_CONTROL_VELOCITY = "uav_control/velocity";
static const std::string TOPIC_CONTROL_VELOCITY_ALERT = "uav_control/velocity/alert";
static const std::string TOPIC_CONTROL_RESUME_MISSION = "uav_control/resume_mission";
static const std::string TOPIC_CONTROL_STOP = "uav_control/stop";
static const std::string TOPIC_CONTROL_STATE = "uav_control/state";
static const std::string TOPIC_CONTROL_GRIPPER = "uav_control/gripper";
static const std::string TOPIC_LOGGER = "/ocs/log_message";
static const float TAKEOFF_HEIGHT = 10.0;
static const float CLOSE_ENOUGH = 2.0;
static const float VELOCITY_ALERT_HEIGHT = 3.0;
static const float PI = 3.14159265359;

UavControl::UavControl(
  ros::NodeHandle &rosNode,
  ros::Rate rosRate,
  std::string odometryTopic,
  MavrosAdapter::Autopilot autopilot,
  bool isSimulation
) : mMavrosAdapter(rosNode, rosRate, autopilot, TAKEOFF_HEIGHT, this, &UavControl::armingEvent, &UavControl::localPoseEvent),
    mPidController(rosNode, this, &UavControl::controlEffortEvent) {
  this->mNodeHandle = &rosNode;
  this->mTakeoffSubscriber = this->mNodeHandle->subscribe<std_msgs::Empty>(TOPIC_CONTROL_TAKEOFF, QUEUE_SIZE, &UavControl::takeoffCallback, this);
  this->mLandSubscriber = this->mNodeHandle->subscribe<std_msgs::Empty>(TOPIC_CONTROL_LAND, QUEUE_SIZE, &UavControl::landCallback, this);
  this->mRtlSubscriber = this->mNodeHandle->subscribe<std_msgs::Empty>(TOPIC_CONTROL_RTL, QUEUE_SIZE, &UavControl::rtlCallback, this);
  this->mWaypointSubscriber = this->mNodeHandle->subscribe<geometry_msgs::Pose>(TOPIC_CONTROL_WAYPOINT, QUEUE_SIZE, &UavControl::waypointCallback, this);
  this->mWaypointNoPublishSubscriber = this->mNodeHandle->subscribe<geometry_msgs::Pose>(TOPIC_CONTROL_WAYPOINT_NO_PUBLISH, QUEUE_SIZE, &UavControl::waypointNoPublishCallback, this);
  this->mVelocitySubscriber = this->mNodeHandle->subscribe<geometry_msgs::Twist>(TOPIC_CONTROL_VELOCITY, QUEUE_SIZE, &UavControl::velocityCallback, this);
  this->mResumeMissionSubscriber = this->mNodeHandle->subscribe<std_msgs::Empty>(TOPIC_CONTROL_RESUME_MISSION, QUEUE_SIZE, &UavControl::resumeMissionCallback, this);
  this->mStopSubscriber = this->mNodeHandle->subscribe<std_msgs::Empty>(TOPIC_CONTROL_STOP, QUEUE_SIZE, &UavControl::stopCallback, this);
  this->mOdometrySubscriber = this->mNodeHandle->subscribe<nav_msgs::Odometry>(odometryTopic, QUEUE_SIZE, &UavControl::odometryCallback, this);
  this->mGripperSubscriber = this->mNodeHandle->subscribe<std_msgs::Bool>(TOPIC_CONTROL_GRIPPER, QUEUE_SIZE, &UavControl::gripperCallback, this);
  this->mWaypointArrivedPublisher = this->mNodeHandle->advertise<std_msgs::Empty>(TOPIC_CONTROL_WAYPOINT_ARRIVED, QUEUE_SIZE);
  this->mVelocityAlertPublisher = this->mNodeHandle->advertise<geometry_msgs::Pose>(TOPIC_CONTROL_VELOCITY_ALERT, QUEUE_SIZE);
  this->mStatePublisher = this->mNodeHandle->advertise<control::StateMsg>(TOPIC_CONTROL_STATE, QUEUE_SIZE);
  this->mLoggerPublisher = this->mNodeHandle->advertise<std_msgs::String>(TOPIC_LOGGER, QUEUE_SIZE);
  this->mEnRouteToWaypoint = false;
  this->mReceivedOdometry = false;
  this->mInPidControl = false;
  this->mIsSimulation = isSimulation;
  this->tfBaselink = this->getNamespace() + TF_BASELINK_SUFFIX;
  this->tfArmOrigin = this->getNamespace() + TF_ARM_ORIGIN_SUFFIX;
  this->initializeParameters();
}

void UavControl::initializeMavros() {
  this->mMavrosAdapter.initialize();
  this->logMessage("Initialized");
}

void UavControl::armingEvent() {
  this->mInPidControl = true;
  this->mGlobalWaypoint = this->mCurrentOdometry.pose.pose;
  this->mGlobalWaypoint.position.z += TAKEOFF_HEIGHT;
  this->setWaypoint(this->mGlobalWaypoint);
  std::stringstream ss;
  ss << "Armed at (" << this->mCurrentOdometry.pose.pose.position.x << ", " << this->mCurrentOdometry.pose.pose.position.y << ", " << this->mCurrentOdometry.pose.pose.position.z << ")";
  this->logMessage(ss.str());
}

void UavControl::localPoseEvent(geometry_msgs::Pose pose) {
  if (this->mMavrosAdapter.isArmed()) {
    this->mPidController.setState(pose);
    this->mPidController.setSetpoint(this->mLocalWaypoint);
  }
  this->publishState();
}

void UavControl::controlEffortEvent(geometry_msgs::Twist twist) {
  if (this->mInPidControl && this->mMavrosAdapter.isInVelocityMode()) {
    this->mMavrosAdapter.executeMoveWithVelocity(twist);
  }
}

void UavControl::initializeParameters() {
  if (this->mNodeHandle->getParam("/arena/angle", this->mAngleOffset)) {
    std::stringstream ss;
    ss << "Angular offset set to \"" << this->mAngleOffset << "\".";
    this->logMessage(ss.str());
  }
  else {
    this->mAngleOffset = 0.0;
    this->logMessage("Angular offset parameter for arena not configured.", true);
  }
}

void UavControl::takeoffCallback(const std_msgs::Empty::ConstPtr& msg) {
  ros::Rate rosRate = 1;
  while (!this->mReceivedOdometry) {
    ROS_WARN("Delaying takeoff until odometry is received.");
    ros::spinOnce();
    rosRate.sleep();
  }
  this->mMavrosAdapter.executeTakeoffSequence(this->mAngleOffset);
}

void UavControl::landCallback(const std_msgs::Empty::ConstPtr& msg) {
  this->mInPidControl = false;
  this->mMavrosAdapter.executeLandingSequence();
}

void UavControl::rtlCallback(const std_msgs::Empty::ConstPtr& msg) {
  this->mInPidControl = false;
  this->mMavrosAdapter.executeRtlSequence();
}

void UavControl::waypointCallback(const geometry_msgs::Pose::ConstPtr& msg) {
  this->setWaypoint(*msg);
  this->mMavrosAdapter.enableVelocityMode(true);
}

void UavControl::waypointNoPublishCallback(const geometry_msgs::Pose::ConstPtr& msg) {
  this->setWaypoint(*msg);
  this->mMavrosAdapter.enableVelocityMode(false);
}

void UavControl::velocityCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  this->mInPidControl = false;
  geometry_msgs::Twist twist;
  float sinTheta = sin(this->mAngleOffset);
  float cosTheta = cos(this->mAngleOffset);
  twist.linear.x = (msg->linear.x * cosTheta) - (msg->linear.y * sinTheta);
  twist.linear.y = (msg->linear.y * cosTheta) + (msg->linear.x * sinTheta);
  twist.linear.z = msg->linear.z;
  this->mMavrosAdapter.executeMoveWithVelocity(twist);
}

void UavControl::resumeMissionCallback(const std_msgs::Empty::ConstPtr& msg) {
  this->mInPidControl = true;
  this->mEnRouteToWaypoint = false;
  // Set waypoint to current position when returning to mission.
  this->mGlobalWaypoint = this->mCurrentOdometry.pose.pose;
  this->setWaypoint(this->mGlobalWaypoint);
  this->mMavrosAdapter.executeMissionResume();
}

void UavControl::stopCallback(const std_msgs::Empty::ConstPtr& msg) {
  this->mGlobalWaypoint = this->mCurrentOdometry.pose.pose;
  this->setWaypoint(this->mGlobalWaypoint);
}

void UavControl::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  this->mCurrentOdometry = *msg;
  this->mReceivedOdometry = true;
  if (this->mEnRouteToWaypoint &&
      fabs(msg->pose.pose.position.x - this->mGlobalWaypoint.position.x) < CLOSE_ENOUGH &&
      fabs(msg->pose.pose.position.y - this->mGlobalWaypoint.position.y) < CLOSE_ENOUGH &&
      fabs(msg->pose.pose.position.z - this->mGlobalWaypoint.position.z) < CLOSE_ENOUGH) {
    this->mWaypointArrivedPublisher.publish(std_msgs::Empty());
    this->mEnRouteToWaypoint = false;
  }
  else if (this->mMavrosAdapter.isInVelocityMode() && msg->pose.pose.position.z < VELOCITY_ALERT_HEIGHT) {
    this->mVelocityAlertPublisher.publish(msg->pose.pose);
  }
}

void UavControl::gripperCallback(const std_msgs::Bool::ConstPtr& msg) {
  this->mMavrosAdapter.enableGripper(msg->data);
  this->logMessage("Magnetic gripper: " + msg->data);
}

void UavControl::setWaypoint(const geometry_msgs::Pose pose) {
  /*
     1. try to transform the waypoint into arm_origin frame
     2. on success : execute waypoint
        on failure : ignore **for now!!!**
  */
  ros::Time starttime = ros::Time::now();
  ros::Duration tensecs(10.0);
  ros::Time now = starttime;
  bool success = false;
  std_msgs::String errormsg, succesMsg;

  geometry_msgs::PoseStamped posestampedglobal;
  geometry_msgs::PoseStamped posestampedarm;
  posestampedglobal.header.frame_id = TF_GLOBAL_ORIGIN;
  posestampedglobal.pose = pose;
  posestampedglobal.pose.orientation = tf::createQuaternionMsgFromYaw(this->mAngleOffset);

  while (now - starttime < tensecs) {
    try {
      // 1. transform the waypoint into arm_origin frame
      if (this->tfListener.waitForTransform(TF_GLOBAL_ORIGIN, this->tfBaselink, ros::Time(0), ros::Duration(1.0))) {
        tfListener.transformPose(this->tfArmOrigin, posestampedglobal, posestampedarm);

        // 2. execute waypoint
        this->mInPidControl = true;
        this->mEnRouteToWaypoint = true;
        this->mGlobalWaypoint = pose;
        this->mLocalWaypoint = posestampedarm.pose;
        success = true;
        break;
      }
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("tfexception - %s", ex.what());
      success = false;
      errormsg.data = "error " + std::string(ex.what()) + " transforming from " + TF_GLOBAL_ORIGIN + " -> " + this->tfArmOrigin + " at uav_control " + this->getNamespace();
      this->mLoggerPublisher.publish(errormsg);
      break;
    }
    now = ros::Time::now();
  }

  if (success) {
    this->logMessage("waypoint published");
  }
  else {
    this->mLoggerPublisher.publish(errormsg);
  }
}

void UavControl::logMessage(std::string message, bool isError) const {
  std_msgs::String logMessage;
  if (isError) {
    logMessage.data = "[ERROR] ";
  }
  logMessage.data += "[" + this->getNamespace() + "] [UavControl] " + message;
  this->mLoggerPublisher.publish(logMessage);
}

std::string UavControl::getNamespace() const {
  std::string ns = this->mNodeHandle->getNamespace();
  ns.erase(0, 2);
  return ns;
}

void UavControl::publishState() const {
  control::StateMsg stateMsg;
  stateMsg.mode = this->mMavrosAdapter.getActionString();
  stateMsg.en_route_to_waypoint = this->mEnRouteToWaypoint;
  stateMsg.have_received_odometry = this->mReceivedOdometry;
  stateMsg.in_pid_control = this->mInPidControl;
  stateMsg.is_simulation = this->mIsSimulation;
  stateMsg.global_waypoint = this->mGlobalWaypoint;
  stateMsg.local_waypoint = this->mLocalWaypoint;
  this->mStatePublisher.publish(stateMsg);
}
