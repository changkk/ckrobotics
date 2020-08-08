#include <ros/ros.h>
#include <ros/package.h>

#include <perception/perception_engine.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "challenge3_perception");

  ros::NodeHandle rosNode;
  int rate = 10;
  std::string panTiltCameraTopic = "perspective_camera/image_raw";
  std::string fisheyeCameraTopic = "fisheye_camera/image_raw";
  std::string fisheyeCalibrationFile = "/etc/camera_info/fisheye_camera.txt";
  std::string odometryTopic = "localizer/rtk_odom";
  std::string location = "mbzirc";
  int minSaturation = 100;
  int minValue = 100;
  rosNode.param(ros::this_node::getName() + "/rate", rate, rate);
  rosNode.param(ros::this_node::getName() + "/perspective_camera", panTiltCameraTopic, panTiltCameraTopic);
  rosNode.param(ros::this_node::getName() + "/fisheye_camera", fisheyeCameraTopic, fisheyeCameraTopic);
  rosNode.param(ros::this_node::getName() + "/fisheye_calibration", fisheyeCalibrationFile, fisheyeCalibrationFile);
  rosNode.param(ros::this_node::getName() + "/odometry_topic", odometryTopic, odometryTopic);
  rosNode.param(ros::this_node::getName() + "/location", location, location);
  ros::Rate rosRate(rate);
  std::string filtersFilename = ros::package::getPath("perception") + "/data/color_filters/" + location + ".txt";
  std::string roiDirectory = ros::package::getPath("perception") + "/data/regions_of_interest";
  PerceptionEngine perceptionEngine(rosNode, panTiltCameraTopic, fisheyeCameraTopic, fisheyeCalibrationFile, odometryTopic);
  perceptionEngine.initialize(rosRate, filtersFilename, roiDirectory);

  while (rosNode.ok()) {
    ros::spinOnce();
    rosRate.sleep();
  }

  return 0;
}
