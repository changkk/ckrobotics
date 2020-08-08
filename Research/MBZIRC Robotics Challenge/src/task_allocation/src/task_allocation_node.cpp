#include <ros/ros.h>

#include <task_allocation/uav.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "task_allocation");

  ros::NodeHandle rosNode;
  int rate = 10;
  rosNode.param(ros::this_node::getName() + "/rate", rate, rate);
  ros::Rate rosRate(rate);

  Uav uav0(rosNode, "hexacopter0");
  Uav uav1(rosNode, "hexacopter1");
  Uav uav2(rosNode, "hexacopter2");

  ros::Duration(2.0).sleep();
  uav1.takeOff();
  ros::Duration(20.0).sleep();
  uav1.setVelocity(-1.0, 0.0, 0.5);
  ros::Duration(20.0).sleep();
  uav1.setWaypoint(-40.0, 40.0, 5.0);
  ros::Duration(20.0).sleep();
  uav1.rtl();

  while (rosNode.ok())
  {
    ros::spinOnce();
    rosRate.sleep();
  }

  return 0;
}
