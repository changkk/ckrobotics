#include <control/uav_control.h>

#include <ros/ros.h>
#include <boost/algorithm/string.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "mavros_control");

  ros::NodeHandle rosNode;
  float rate = 30.0;
  std::string odometryTopic = "localizer/rtk_odom";
  std::string autopilot;
  bool isSimulation = false;
  rosNode.param(ros::this_node::getName() + "/rate", rate, rate);
  rosNode.param(ros::this_node::getName() + "/odometry_topic", odometryTopic, odometryTopic);
  rosNode.param(ros::this_node::getName() + "/autopilot", autopilot, autopilot);
  rosNode.param(ros::this_node::getName() + "/simulation", isSimulation, isSimulation);
  ros::Rate rosRate(rate);
  MavrosAdapter::Autopilot mavrosAutopilot = MavrosAdapter::Autopilot::UNDEFINED;
  boost::algorithm::to_lower(autopilot);
  if (autopilot == "px4") {
    mavrosAutopilot = MavrosAdapter::Autopilot::PX4;
  }
  else if (autopilot == "apm") {
    mavrosAutopilot = MavrosAdapter::Autopilot::APM;
  }
  else {
    ROS_ERROR("Autopilot not configured.");
    return 1;
  }
  UavControl uavControl(rosNode, rosRate, odometryTopic, mavrosAutopilot, isSimulation);
  uavControl.initializeMavros();

  while (rosNode.ok()) {
    ros::spinOnce();
    rosRate.sleep();
  }

  return 0;
}
