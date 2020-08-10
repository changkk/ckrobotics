/*
  This node transforms pan/tilt camera image to bird's eye view at a set height

  Input :
    Subscribes to altitude, camera info and camera image topics

  Output  :
    Publishes the transformed image as a laser scan

  Author  : janindu@vt.edu
*/

// ROS dependencies
#include <ros/ros.h>

// Project dependencies
#include "image_transformer.cpp"

static const std::string PARAM_IMAGE_SUB_TOPIC = "/camera/image_topic";
static const std::string PARAM_CAM_INFO_SUB_TOPIC = "/camera/info_topic";
static const std::string PARAM_CAM_POSE_SUB_TOPIC = "/camera/pose_topic";
static const std::string PARAM_TF_IMAGE_PUB_TOPIC = "camera/transformed_image_topic";
static const std::string PARAM_IMU_SUB_TOPIC = "/uav/imu_topic";
static const std::string PARAM_UAV_MODEL_NAME = "/uav_gazebo_model_name";
static const std::string PARAM_EDGE_DET_LT = "/edge_det/low_t";
static const std::string PARAM_EDGE_DET_HT = "/edge_det/high_t";
static const std::string PARAM_TF_HEIGHT = "/tf_height";

static const std::string DEF_SUB_IMAGE_TOPIC = "/hexacopter/pan_tilt_camera/image_raw";
static const std::string DEF_SUB_CAM_INFO_TOPIC = "/hexacopter/pan_tilt_camera/camera_info";
static const std::string DEF_SUB_CAM_POSE_TOPIC = "/gazebo/model_states";
static const std::string DEF_PUB_TF_IMAGE_TOPIC = "/localizer/tf_image";
static const std::string DEF_SUB_IMU_TOPIC = "/hexacopter/mavros/imu/data";
static const std::string DEF_UAV_MODEL_NAME = "hexacopter";
static const int DEF_EDGE_DET_LT = 250;
static const double DEF_TF_HEIGHT = 30.0;

int main(int argc, char** argv) {
  ros::init(argc, argv, argv[1]);
  ros::NodeHandle nh;

  std::string subImageTopic, subCamInfoTopic, subCamPoseTopic, subImuTopic, pubTfImageTopic, uavModelName;
  int edgeDtLt, edgeDtHt;
  double tfHeight;

  // Read parameters passed from the launch file
  nh.param<std::string>(argv[1] + PARAM_IMAGE_SUB_TOPIC, subImageTopic, DEF_SUB_IMAGE_TOPIC);
  nh.param<std::string>(argv[1] + PARAM_CAM_INFO_SUB_TOPIC, subCamInfoTopic, DEF_SUB_CAM_INFO_TOPIC);
  nh.param<std::string>(argv[1] + PARAM_CAM_POSE_SUB_TOPIC, subCamPoseTopic, DEF_SUB_CAM_POSE_TOPIC);
  nh.param<std::string>(argv[1] + PARAM_IMU_SUB_TOPIC, subImuTopic, DEF_SUB_IMU_TOPIC);
  nh.param<std::string>(argv[1] + PARAM_TF_IMAGE_PUB_TOPIC, pubTfImageTopic, DEF_PUB_TF_IMAGE_TOPIC);
  nh.param<std::string>(argv[1] + PARAM_UAV_MODEL_NAME, uavModelName, DEF_UAV_MODEL_NAME);
  nh.param<int>(argv[1] + PARAM_EDGE_DET_LT, edgeDtLt, DEF_EDGE_DET_LT);
  nh.param<int>(argv[1] + PARAM_EDGE_DET_HT, edgeDtHt, DEF_EDGE_DET_LT*3);
  nh.param<double>(argv[1] + PARAM_TF_HEIGHT, tfHeight, DEF_TF_HEIGHT);

  ImageTransformer imageTransformer(subImageTopic, subCamInfoTopic, subCamPoseTopic, subImuTopic, pubTfImageTopic, uavModelName, edgeDtLt, edgeDtHt, tfHeight);
  imageTransformer.start(nh);

  ros::spin();
  
  std::cout << "Terminating image_tf_node" << std::endl;
  nh.shutdown();
  
  return 0;
}
