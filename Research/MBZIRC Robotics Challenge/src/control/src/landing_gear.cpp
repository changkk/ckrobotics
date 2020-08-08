#include <control/landing_gear.h>

#include <std_msgs/String.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <sstream>

static const float REQUEST_INTERVAL = 1.0;
static const float LANDING_GEAR_RETRACT_HEIGHT = 6.0;

LandingGear::LandingGear() {
  this->mIsRetracted = false;
  this->mRetractAttempts = 0;
  this->mLastRequest = ros::Time::now();
}

void LandingGear::initialize(ros::NodeHandle& rosNode, ros::Publisher& rcOverridePublisher, ros::Publisher& loggerPublisher) {
  this->mNodeHandle = rosNode;
  this->mRcOverridePublisher = rcOverridePublisher;
  this->mLoggerPublisher = loggerPublisher;
}

bool LandingGear::retract(float height) {
  if (height > LANDING_GEAR_RETRACT_HEIGHT &&
      ros::Time::now() - this->mLastRequest > ros::Duration(REQUEST_INTERVAL)) {
    this->mLastRequest = ros::Time::now();
    this->mIsRetracted = true;
    ++(this->mRetractAttempts);
    this->publishMessage(this->mIsRetracted);
    std::stringstream ss;
    ss << "Retracting landing gear at " << height << "m according to MAVROS.";
    this->logMessage(ss.str());
    return true;
  }
  return false;
}

bool LandingGear::extend(bool attemptingToRetract) {
  if (ros::Time::now() - this->mLastRequest > ros::Duration(REQUEST_INTERVAL)) {
    this->mLastRequest = ros::Time::now();
    this->mIsRetracted = false;
    if (!attemptingToRetract) {
      this->mRetractAttempts = 0;
    }
    this->publishMessage(this->mIsRetracted);
    this->logMessage("Extending landing gear.");
    return true;
  }
  return false;
}

bool LandingGear::getIsRetracted() const {
  return this->mIsRetracted;
}

int LandingGear::getRetractAttempts() const {
  return this->mRetractAttempts;
}

void LandingGear::publishMessage(bool retract) const {
  mavros_msgs::OverrideRCIn rcOverrideMsg;
  for (int i = 0; i < 8; ++i) {
    rcOverrideMsg.channels[i] = rcOverrideMsg.CHAN_NOCHANGE;
  }
  rcOverrideMsg.channels[6] = retract ? 1000 : 2000;
  this->mRcOverridePublisher.publish(rcOverrideMsg);
}

void LandingGear::logMessage(std::string message) const {
  std::string ns = this->mNodeHandle.getNamespace();
  ns.erase(0, 2);
  std_msgs::String logMessage;
  logMessage.data = "[" + ns + "] [LandingGear] " + message;
  this->mLoggerPublisher.publish(logMessage);
}
