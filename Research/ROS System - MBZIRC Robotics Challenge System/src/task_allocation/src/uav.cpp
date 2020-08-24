#include <task_allocation/uav.h>

#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <iostream>

static const std::string TOPIC_UAV_CONTROLLER_TAKEOFF = "uav_control/takeoff";
static const std::string TOPIC_UAV_CONTROLLER_RTL = "uav_control/rtl";
static const std::string TOPIC_UAV_CONTROLLER_WAYPOINT = "uav_control/waypoint";
static const std::string TOPIC_UAV_CONTROLLER_VELOCITY = "uav_control/velocity";
static const int QUEUE_SIZE = 10;

Uav::Uav(ros::NodeHandle &rosNode, std::string name) {
  std::string namespaceString = "/" + name + "/";
  this->mTakeoffPublisher = rosNode.advertise<std_msgs::Empty>(namespaceString + TOPIC_UAV_CONTROLLER_TAKEOFF, QUEUE_SIZE);
  this->mRtlPublisher = rosNode.advertise<std_msgs::Empty>(namespaceString + TOPIC_UAV_CONTROLLER_RTL, QUEUE_SIZE);
  this->mWaypointPublisher = rosNode.advertise<geometry_msgs::Pose>(namespaceString + TOPIC_UAV_CONTROLLER_WAYPOINT, QUEUE_SIZE);
  this->mVelocityPublisher = rosNode.advertise<geometry_msgs::Twist>(namespaceString + TOPIC_UAV_CONTROLLER_VELOCITY, QUEUE_SIZE);
}

void Uav::takeOff() {
  this->mTakeoffPublisher.publish(std_msgs::Empty());
}

void Uav::rtl() {
  this->mRtlPublisher.publish(std_msgs::Empty());
}
void Uav::setWaypoint(float xPos, float yPos, float zPos) {
  geometry_msgs::Pose waypoint;
  waypoint.position.x = xPos;
  waypoint.position.y = yPos;
  waypoint.position.z = zPos;
  this->mWaypointPublisher.publish(waypoint);
}

void Uav::setVelocity(float xVel, float yVel, float zVel) {
  geometry_msgs::Twist velocity;
  velocity.linear.x = xVel;
  velocity.linear.y = yVel;
  velocity.linear.z = zVel;
  this->mVelocityPublisher.publish(velocity);
}
