/*
  This node simulates RTK-GPS

  Input :
    Subscribes to gazebo model state

  Output  :
    Publishes UAV pose with associated uncertainty

  Author  : janindu@vt.edu
*/

// ROS dependencies
#include <ros/ros.h>

// Project dependencies
#include "rtk_simulator.cpp"

static const std::string PARAM_UAV_POSE_SUB_TOPIC = "/uav_pose/subscribe_topic";
static const std::string PARAM_UAV_POSE_PUB_TOPIC = "/uav_pose/publish_topic";
static const std::string PARAM_UAV_ODOM_PUB_TOPIC = "/uav_odom/publish_topic";
static const std::string PARAM_UAV_MODEL_NAME = "/uav_gazebo_model_name";
static const std::string PARAM_GAUSSIAN_ERROR = "/gaussian_error_variance";

static const std::string DEF_SUB_UAV_POSE_TOPIC = "/gazebo/model_states";
static const std::string DEF_PUB_UAV_POSE_TOPIC = "/localizer/rtk_pose";
static const std::string DEF_PUB_UAV_ODOM_TOPIC = "/localizer/rtk_odom";
static const std::string DEF_UAV_MODEL_NAME = "hexacopter";
static const double DEF_ERROR_VAR = 0.01;

int main(int argc, char** argv) {
  ros::init(argc, argv, argv[1]);
  ros::NodeHandle nh;

  std::string subUavPoseTopic, pubUavPoseTopic, pubUavOdomTopic, uavModelName;
  double errorVar;

  // Read parameters passed from the launch file
  nh.param<std::string>(argv[1] + PARAM_UAV_POSE_SUB_TOPIC, subUavPoseTopic, DEF_SUB_UAV_POSE_TOPIC);
  nh.param<std::string>(argv[1] + PARAM_UAV_POSE_PUB_TOPIC, pubUavPoseTopic, DEF_PUB_UAV_POSE_TOPIC);
  nh.param<std::string>(argv[1] + PARAM_UAV_ODOM_PUB_TOPIC, pubUavOdomTopic, DEF_PUB_UAV_ODOM_TOPIC);
  nh.param<std::string>(argv[1] + PARAM_UAV_MODEL_NAME, uavModelName, DEF_UAV_MODEL_NAME);
  nh.param<double>(argv[1] + PARAM_GAUSSIAN_ERROR, errorVar, DEF_ERROR_VAR);

  RTKSimulator rtkSim(subUavPoseTopic, pubUavPoseTopic, pubUavOdomTopic, uavModelName, errorVar);
  rtkSim.start(nh);

  ros::spin();

  std::cout << "Terminating rtk_simulator_node" << std::endl;
  nh.shutdown();

  return 0;
}

