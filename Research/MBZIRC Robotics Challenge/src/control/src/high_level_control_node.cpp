#include <control/target_task_control.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <string>

int main(int argc, char** argv) {
  ros::init(argc, argv, "high_level_control");

  ros::NodeHandle rosNode;
  int rate = 10;
  std::string name = "hexacopter";
  float targetHeight = 0.5;
  rosNode.param(ros::this_node::getName() + "/rate", rate, rate);
  rosNode.param(ros::this_node::getName() + "/name", name, name);
  rosNode.param(ros::this_node::getName() + "/target_height", targetHeight, targetHeight);
  ros::Rate rosRate(rate);
  std::string dataDirectory = ros::package::getPath("control") + "/data/search_patterns/" + name;
  TargetTaskControl targetTaskControl(rosNode, targetHeight);
  targetTaskControl.loadSearchPatterns(dataDirectory);

  while (rosNode.ok()) {
    ros::spinOnce();
    rosRate.sleep();
  }

  return 0;
}
