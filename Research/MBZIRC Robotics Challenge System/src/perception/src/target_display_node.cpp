#include <perception/target_display.h>

#include <ros/ros.h>
#include <ros/package.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "target_display");

  ros::NodeHandle rosNode;
  int rate = 10;
  std::string targetTopic = "perception/targets";
  std::string odometryTopic = "localizer/global_odom";
  std::string arenaImage = ros::package::getPath("perception") + "/data/mbzirc_arena.png";
  rosNode.param(ros::this_node::getName() + "/rate", rate, rate);
  rosNode.param(ros::this_node::getName() + "/target_topic", targetTopic, targetTopic);
  rosNode.param(ros::this_node::getName() + "/odometry_topic", odometryTopic, odometryTopic);
  ros::Rate rosRate(rate);
  TargetDisplay targetDisplay(rosNode, targetTopic, odometryTopic, arenaImage);

  while (rosNode.ok()) {
    ros::spinOnce();
    rosRate.sleep();
  }

  return 0;
}
