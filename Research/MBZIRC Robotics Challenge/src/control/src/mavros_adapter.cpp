#include <control/mavros_adapter.h>

#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/StreamRate.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <tf/transform_datatypes.h>

static const int QUEUE_SIZE = 1;
static const std::string TOPIC_MAVROS_STATE = "mavros/state";
static const std::string TOPIC_MAVROS_POSITION_GLOBAL = "mavros/global_position/global";
static const std::string TOPIC_MAVROS_POSITION_LOCAL = "mavros/local_position/pose";
static const std::string TOPIC_MAVROS_SETPOINT_POSITION_LOCAL = "mavros/setpoint_position/local";
static const std::string TOPIC_MAVROS_SETPOINT_VELOCITY = "mavros/setpoint_velocity/cmd_vel";
static const std::string TOPIC_MAVROS_COMMAND_ARM = "mavros/cmd/arming";
static const std::string TOPIC_MAVROS_COMMAND_TAKEOFF = "mavros/cmd/takeoff";
static const std::string TOPIC_MAVROS_SET_MODE = "mavros/set_mode";
static const std::string TOPIC_MAVROS_SET_STREAM_RATE = "mavros/set_stream_rate";
static const std::string TOPIC_MAVROS_RC_IN = "mavros/rc/in";
static const std::string TOPIC_MAVROS_RC_OVERRIDE = "mavros/rc/override";
static const std::string TOPIC_MAVROS_ACTUATOR_CONTROL = "mavros/actuator_control";
static const std::string TOPIC_LOGGER = "/ocs/log_message";
static const std::string PX4_MODE_OFFBOARD = "OFFBOARD";
static const std::string PX4_MODE_LAND = "AUTO.LAND";
static const std::string PX4_MODE_RTL = "AUTO.RTL";
static const std::string APM_MODE_OFFBOARD = "GUIDED";
static const std::string APM_MODE_LAND = "LAND";
static const std::string APM_MODE_RTL = "RTL";
static const float REQUEST_INTERVAL = 2.0;
static const float THROTTLE_NUDGE_INTERVAL = 5.0;
static const float THROTTLE_NUDGE_HEIGHT = 10.0;
static const unsigned int RC_THROTTLE_NUDGE_VALUE = 1100;
static const int LANDING_GEAR_CYCLES = 3;

MavrosAdapter::MavrosAdapter(
  ros::NodeHandle &rosNode,
  ros::Rate rosRate,
  Autopilot autopilot,
  float takeoffHeight,
  UavControl* uavControl,
  void (UavControl::*armingCallback)(),
  void (UavControl::*localPoseCallback)(geometry_msgs::Pose pose)
) : mRosRate(rosRate),
    TAKEOFF_HEIGHT(takeoffHeight) {
  this->mNodeHandle = &rosNode;
  this->mStateSubscriber = this->mNodeHandle->subscribe<mavros_msgs::State>(TOPIC_MAVROS_STATE, QUEUE_SIZE, &MavrosAdapter::stateCallback, this);
  this->mGlobalPositionSubscriber = this->mNodeHandle->subscribe<sensor_msgs::NavSatFix>(TOPIC_MAVROS_POSITION_GLOBAL, QUEUE_SIZE, &MavrosAdapter::globalPositionCallback, this);
  this->mLocalPositionSubscriber = this->mNodeHandle->subscribe<geometry_msgs::PoseStamped>(TOPIC_MAVROS_POSITION_LOCAL, QUEUE_SIZE, &MavrosAdapter::localPositionCallback, this);
  this->mRcInSubscriber = this->mNodeHandle->subscribe<mavros_msgs::RCIn>(TOPIC_MAVROS_RC_IN, QUEUE_SIZE, &MavrosAdapter::rcInCallback, this);
  this->mWaypointPublisher = this->mNodeHandle->advertise<geometry_msgs::PoseStamped>(TOPIC_MAVROS_SETPOINT_POSITION_LOCAL, QUEUE_SIZE);
  this->mVelocityPublisher = this->mNodeHandle->advertise<geometry_msgs::TwistStamped>(TOPIC_MAVROS_SETPOINT_VELOCITY, QUEUE_SIZE);
  this->mRcOverridePublisher = this->mNodeHandle->advertise<mavros_msgs::OverrideRCIn>(TOPIC_MAVROS_RC_OVERRIDE, QUEUE_SIZE);
  this->mLoggerPublisher = this->mNodeHandle->advertise<std_msgs::String>(TOPIC_LOGGER, QUEUE_SIZE);
  this->mArmingService = this->mNodeHandle->serviceClient<mavros_msgs::CommandBool>(TOPIC_MAVROS_COMMAND_ARM);
  this->mTakeoffService = this->mNodeHandle->serviceClient<mavros_msgs::CommandTOL>(TOPIC_MAVROS_COMMAND_TAKEOFF);
  this->mSetModeService = this->mNodeHandle->serviceClient<mavros_msgs::SetMode>(TOPIC_MAVROS_SET_MODE);
  this->mSetStreamRateService = this->mNodeHandle->serviceClient<mavros_msgs::StreamRate>(TOPIC_MAVROS_SET_STREAM_RATE);
  this->mMavrosThread = NULL;
  this->mLastRequest = ros::Time::now();
  this->mTakeoffStart = ros::Time::now();
  this->mAction = Action::NONE;
  this->mIsRcInterrupt = false;
  this->mIsTakingOff = false;
  this->mTakeoffServiceResponse = true;
  this->mDoPublishVelocity = true;
  this->mRcFlightModePulseValue = 0;
  this->mFixedYaw = 0.0;
  this->configureAutopilot(autopilot);
  this->mUavControl = uavControl;
  this->mArmingCallback = armingCallback;
  this->mLocalPoseCallback = localPoseCallback;
  this->mLandingGear.initialize(*(this->mNodeHandle), this->mRcOverridePublisher, this->mLoggerPublisher);
}

MavrosAdapter::~MavrosAdapter() {
  if (this->mMavrosThread != NULL) {
    this->mMavrosThread->join();
    delete this->mMavrosThread;
  }
}

void MavrosAdapter::initialize() {
  this->connectToFlightController();
  this->setRcChannelStreamRate();
  this->mMavrosThread = new std::thread(&MavrosAdapter::threadLoop, this, this->mRosRate);
}

void MavrosAdapter::executeTakeoffSequence(float yaw) {
  if (!this->mCurrentState.armed) {
    this->logMessage("Initiating takeoff.");
    this->mFixedYaw = yaw;
    this->mAction = Action::TAKEOFF;
  }
  else {
    this->logMessage("FCU already armed; ignoring takeoff command.");
  }
}

void MavrosAdapter::executeLandingSequence() {
  if (this->mCurrentState.armed) {
    this->logMessage("Initiating landing.");
    this->mAction = Action::LAND;
  }
  else {
    this->logMessage("FCU not armed; ignoring landing command.");
  }
}

void MavrosAdapter::executeRtlSequence() {
  if (this->mCurrentState.armed) {
    this->logMessage("Initiating return to launch.");
    this->mAction = Action::RTL;
  }
  else {
    this->logMessage("FCU not armed; ignoring RTL command.");
  }
}

void MavrosAdapter::executeMoveWithVelocity(geometry_msgs::Twist velocity) {
  if (this->mCurrentState.armed) {
    if (!this->isTakingOff()) {
      this->mDoPublishVelocity = true;
      this->mAction = Action::VELOCITY;
      this->mVelocity = velocity;
    }
  }
  else {
    this->logMessage("FCU not armed; ignoring velocity command.");
  }
}

void MavrosAdapter::executeMoveWithWaypoint(geometry_msgs::Pose waypoint) {
  if (this->mCurrentState.armed) {
    if (!this->isTakingOff()) {
      this->mAction = Action::WAYPOINT;
      this->mWaypoint = waypoint;
    }
  }
  else {
    this->logMessage("FCU not armed; ignoring waypoint command.");
  }
}

void MavrosAdapter::executeMissionResume() {
  if (this->mIsRcInterrupt) {
    this->mIsRcInterrupt = false;
    this->logMessage("RC interrupt disabled; resuming mission: \"" + this->getActionString() + "\".");
  }
  else {
    this->logMessage("RC interrupt not already enabled.");
  }
  if (!this->mCurrentState.armed && (this->mAction == Action::LAND || this->mAction == Action::RTL)) {
    mavros_msgs::SetMode setModeCommand;
    setModeCommand.request.base_mode = setModeCommand.request.MAV_MODE_STABILIZE_DISARMED;
    if (this->mSetModeService.call(setModeCommand)) {
      this->mAction = Action::NONE;
      ROS_INFO("Mavros Adapter: Action set to NO ACTION.");
    }
    else {
      ROS_ERROR("Mavros Adapter: Could not set mode in resume mission.");
    }
  }
}

void MavrosAdapter::enableVelocityMode(bool doPublishVelocity) {
  if (this->mCurrentState.armed) {
    if (!this->isTakingOff()) {
      this->mDoPublishVelocity = doPublishVelocity;
      this->mAction = Action::VELOCITY;
    }
  }
  else {
    this->logMessage("FCU not armed; ignoring velocity enable mode.");
  }
}

void MavrosAdapter::enableGripper(bool enable) {
  mavros_msgs::OverrideRCIn rcOverrideMsg;
  for (int i = 0; i < 8; ++i) {
    rcOverrideMsg.channels[i] = rcOverrideMsg.CHAN_NOCHANGE;
  }
  rcOverrideMsg.channels[7] = enable ? 2000 : 1000;
  this->mRcOverridePublisher.publish(rcOverrideMsg);
  sleep(4);
  rcOverrideMsg.channels[7] = 1500;
  this->mRcOverridePublisher.publish(rcOverrideMsg);
}

bool MavrosAdapter::isArmed() const {
  return this->mCurrentState.armed;
}

bool MavrosAdapter::isInVelocityMode() const {
  return this->mAction == Action::VELOCITY && this->mDoPublishVelocity;
}

void MavrosAdapter::configureAutopilot(Autopilot autopilot) {
  switch (autopilot) {
  case Autopilot::PX4:
    this->mFcuModeOffboard = PX4_MODE_OFFBOARD;
    this->mFcuModeLand = PX4_MODE_LAND;
    this->mFcuModeRtl = PX4_MODE_RTL;
    this->logMessage("Mavros Adapter: Autopilot set to PX4.");
    break;
  case Autopilot::APM:
    this->mFcuModeOffboard = APM_MODE_OFFBOARD;
    this->mFcuModeLand = APM_MODE_LAND;
    this->mFcuModeRtl = APM_MODE_RTL;
    this->logMessage("Mavros Adapter: Autopilot set to APM.");
    break;
  default:
    ROS_ERROR("Mavros Adapter: Autopilot not configured.");
    return;
    break;
  }
}

void MavrosAdapter::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
  this->mCurrentState = *msg;
}

void MavrosAdapter::globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  this->mGlobalPosition = *msg;
}

void MavrosAdapter::localPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  this->mLocalPose = msg->pose;
  if (!this->mCurrentState.armed) {
    this->mPreArmPose = this->mLocalPose;
  }
  (*this->mUavControl.*this->mLocalPoseCallback)(this->mLocalPose);
}

void MavrosAdapter::rcInCallback(const mavros_msgs::RCIn::ConstPtr& msg) {
  if (msg->channels.size() >= 5) {
    if (this->mRcFlightModePulseValue != 0 && abs(this->mRcFlightModePulseValue - msg->channels[4]) > 5) {
      if (this->mIsRcInterrupt) {
        this->logMessage("RC interrupt already enabled; resume mission will result in \"" + this->getActionString() + "\".");
      }
      else {
        this->logMessage("RC interrupt detected; resume mission will result in \"" + this->getActionString() + "\".");
        this->logMessage("RC interrupt is enabled; send message to uav_control/resume_mission topic to resume.");
        this->mIsRcInterrupt = true;
        this->releaseAllChannels();
      }
    }
    this->mRcFlightModePulseValue = msg->channels[4];
  }
}

void MavrosAdapter::connectToFlightController() {
  // Before publishing anything, we wait for the connection to be established between mavros and the autopilot.
  // This loop should exit as soon as a heartbeat message is received.
  while (ros::ok() && !this->mCurrentState.connected) {
    ros::spinOnce();
    this->mRosRate.sleep();
  }
}

void MavrosAdapter::configureFlightMode(const std::string& flightMode) {
  if (this->mCurrentState.mode != flightMode && (ros::Time::now() - this->mLastRequest > ros::Duration(REQUEST_INTERVAL))) {
    // Before entering offboard mode, you must have already started streaming setpoints otherwise the mode switch will be rejected.
    if (flightMode == this->mFcuModeOffboard) {
      ros::Rate rate(30.0);
      for (int i = 50; ros::ok() && i > 0; --i) {
        this->publishWaypoint();
        ros::spinOnce();
        rate.sleep();
      }
    }
    this->logMessage("Attempting flight mode " + flightMode + ".");
    mavros_msgs::SetMode setModeCommand;
    setModeCommand.request.custom_mode = flightMode;
    if (this->mSetModeService.call(setModeCommand)) {
      this->logMessage("Setting flight mode to " + flightMode + ".");
    }
    this->mLastRequest = ros::Time::now();
  }
}

void MavrosAdapter::armFlightController() {
  if (!this->mCurrentState.armed && (ros::Time::now() - this->mLastRequest > ros::Duration(REQUEST_INTERVAL))) {
    this->logMessage("Attempting to arm.");
    mavros_msgs::CommandBool armCommand;
    armCommand.request.value = true;
    if (this->mArmingService.call(armCommand)) {
      this->logMessage("Vehicle arming.");
      (*this->mUavControl.*this->mArmingCallback)();
    }
    this->mLastRequest = ros::Time::now();
  }
}

void MavrosAdapter::callTakeoffService() {
  if (this->mCurrentState.armed && !this->mIsTakingOff) {
    mavros_msgs::CommandTOL takeoffCommand;
    tf::Quaternion quaternion;
    double roll, pitch, yaw;
    tf::quaternionMsgToTF(this->mPreArmPose.orientation, quaternion);
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    takeoffCommand.request.yaw = yaw;
    takeoffCommand.request.latitude = this->mGlobalPosition.latitude;
    takeoffCommand.request.longitude = this->mGlobalPosition.longitude;
    takeoffCommand.request.altitude = this->mGlobalPosition.altitude + TAKEOFF_HEIGHT;
    this->mTakeoffServiceResponse = this->mTakeoffService.call(takeoffCommand);
    if (this->mTakeoffServiceResponse) {
      this->mIsTakingOff = true;
      this->mTakeoffStart = ros::Time::now();
      this->logMessage("Start throttle nudge.");
      this->nudgeThrottle(true);
    }
    else {
      ROS_ERROR("Takeoff failed.");
    }
  }
}

void MavrosAdapter::setRcChannelStreamRate() {
  mavros_msgs::StreamRate streamRate;
  streamRate.request.stream_id = streamRate.request.STREAM_RC_CHANNELS;
  streamRate.request.message_rate = 10;
  streamRate.request.on_off = true;
  if (this->mSetStreamRateService.call(streamRate)) {
    this->logMessage("Setting RC channel stream rate.");
  }
}

void MavrosAdapter::nudgeThrottle(bool enable) const {
  mavros_msgs::OverrideRCIn rcOverrideMsg;
  for (int i = 0; i < 8; ++i) {
    rcOverrideMsg.channels[i] = rcOverrideMsg.CHAN_NOCHANGE;
  }
  rcOverrideMsg.channels[2] = enable ? RC_THROTTLE_NUDGE_VALUE : rcOverrideMsg.CHAN_RELEASE;
  this->mRcOverridePublisher.publish(rcOverrideMsg);
}

void MavrosAdapter::releaseAllChannels() const {
  mavros_msgs::OverrideRCIn rcOverrideMsg;
  for (int i = 0; i < 8; ++i) {
    rcOverrideMsg.channels[i] = rcOverrideMsg.CHAN_RELEASE;
  }
  this->mRcOverridePublisher.publish(rcOverrideMsg);
  this->logMessage("Releasing all channels.");
}

void MavrosAdapter::completeTakeoff() {
  if (this->mIsTakingOff &&
      (this->mLocalPose.position.z > THROTTLE_NUDGE_HEIGHT ||
       ros::Time::now() - this->mTakeoffStart > ros::Duration(THROTTLE_NUDGE_INTERVAL))) {
    this->mIsTakingOff = false;
    this->logMessage("Stop throttle nudge.");
    this->nudgeThrottle(false);
    sleep(1.0);
    this->nudgeThrottle(false);
    if (this->mTakeoffServiceResponse) {
      this->mVelocity = geometry_msgs::Twist();
      this->mWaypoint = this->mPreArmPose;
      this->mWaypoint.position.z = TAKEOFF_HEIGHT;
      this->mAction = Action::WAYPOINT;
    }
    else {
      this->logMessage("Problem taking off; disarming.");
      this->mAction = Action::NONE;
      mavros_msgs::CommandBool armCommand;
      armCommand.request.value = false;
      this->mArmingService.call(armCommand);
    }
  }
}

void MavrosAdapter::tryLandingGearRetract() {
  if (this->mLandingGear.getRetractAttempts() < LANDING_GEAR_CYCLES) {
    if (this->mLandingGear.getIsRetracted()) {
      this->mLandingGear.extend(true);
    }
    else {
      this->mLandingGear.retract(this->mLocalPose.position.z);
    }
  }
}

void MavrosAdapter::threadLoop(ros::Rate rosRate) {
  while (this->mNodeHandle->ok()) {
    if (!this->mIsRcInterrupt) {
      switch (this->mAction) {
      case Action::TAKEOFF:
        this->armFlightController();
        this->callTakeoffService();
        this->completeTakeoff();
        break;
      case Action::LAND:
        this->mLandingGear.extend(false);
        this->configureFlightMode(this->mFcuModeLand);
        break;
      case Action::RTL:
        this->mLandingGear.extend(false);
        this->configureFlightMode(this->mFcuModeRtl);
        break;
      case Action::VELOCITY:
        this->tryLandingGearRetract();
        this->configureFlightMode(this->mFcuModeOffboard);
        this->publishVelocity();
        break;
      case Action::WAYPOINT:
        this->tryLandingGearRetract();
        this->configureFlightMode(this->mFcuModeOffboard);
        this->publishWaypoint();
        break;
      default:
        break;
      }
    }
    ros::spinOnce();
    rosRate.sleep();
  }
}

void MavrosAdapter::publishVelocity() const {
  if (this->mDoPublishVelocity) {
    geometry_msgs::TwistStamped velocityMsg;
    velocityMsg.twist = this->mVelocity;
    this->mVelocityPublisher.publish(velocityMsg);
  }
}

void MavrosAdapter::publishWaypoint() const {
  geometry_msgs::PoseStamped waypointMsg;
  waypointMsg.pose = this->mWaypoint;
  this->mWaypointPublisher.publish(waypointMsg);
}

void MavrosAdapter::logMessage(std::string message) const {
  std::string ns = this->mNodeHandle->getNamespace();
  ns.erase(0, 2);
  std_msgs::String logMessage;
  logMessage.data = "[" + ns + "] [MavrosAdapter] " + message;
  this->mLoggerPublisher.publish(logMessage);
}

bool MavrosAdapter::isTakingOff() const {
  return this->mIsTakingOff || !this->mTakeoffServiceResponse;
}

std::string MavrosAdapter::getActionString() const {
  switch (this->mAction) {
  case Action::TAKEOFF:
    return "TAKEOFF";
    break;
  case Action::LAND:
    return "LAND";
    break;
  case Action::RTL:
    return "RTL";
    break;
  case Action::VELOCITY:
    return "VELOCITY";
    break;
  case Action::WAYPOINT:
    return "WAYPOINT";
    break;
  default:
    return "NO ACTION";
    break;
  }
}
