/*
  This node undistorts fisheye camera images

  Input : 
    Subscribes to fisheye image feed

  Output  :
    Publishes undistorted image feed

  Author  : janindu@vt.edu
*/

// ROS dependencies
#include <ros/ros.h>

// Project dependencies
#include "localizer/fisheye_corrector.hpp"

// static const std::string PARAM_IMAGE_PUB_TOPIC = "pub_image_topic";
static const std::string PARAM_CALIB_FILE_NAME = "calib_data_file";
static const std::string PARAM_CAM_IMG_HEIGHT = "cam_img_height";
static const std::string PARAM_CAM_IMG_WIDTH = "cam_img_width";
// static const std::string PARAM_UAV_POSE_TOPIC = "sub_uavpose_topic";
// static const std::string PARAM_UAV_GPSPOSE_TOPIC = "sub_uavgpspose_topic";
static const std::string PARAM_FISH_EYE_CORRECTION = "enable_fisheye_correction";

// static const std::string PARAM_FEIMAGE_SUB_TOPIC = "sub_feimage_topic";
// static const std::string PARAM_PCIMAGE_SUB_TOPIC = "sub_pcimage_topic";
// static const std::string PARAM_FECAMINFO_SUB_TOPIC = "sub_fecaminfo_topic";
// static const std::string PARAM_PCCAMINFO_SUB_TOPIC = "sub_pccaminfo_topic";

static const std::string DEF_SUB_FEIMAGE_TOPIC = "fisheye_camera/image_raw";
static const std::string DEF_SUB_FECAMINFO_TOPIC = "fisheye_camera/camera_info";
static const std::string DEF_SUB_PCIMAGE_TOPIC = "perspective_camera/image_raw";
static const std::string DEF_SUB_PCCAMINFO_TOPIC = "perspective_camera/camera_info";

static const std::string DEF_PUB_IMAGE_TOPIC = "fisheye_camera/image_rectified";
static const std::string DEF_SUB_UAVPOSE_TOPIC = "uav_pose";
static const std::string DEF_SUB_UAVGPSPOSE_TOPIC = "rtkfix";

static const std::string PARAM_SRV_TOPIC = "fisheye_service_topic";
static const std::string PARAM_LASER_TOPIC = "laser_topic";

static const std::string DEF_CALIB_FILE_NAME = "./calib_results.txt";
static const std::string DEF_SRV_TOPIC = "fisheye_srv";
static const std::string DEF_LASER_TOPIC = "/hexacopter/localizer/scanpos";
static const int DEF_CAM_IMG_HEIGHT = 480;
static const int DEF_CAM_IMG_WIDTH = 640;

int main(int argc, char** argv) {
  ros::init(argc, argv, argv[1]);
  ros::NodeHandle nh;
  ros::NodeHandle privatenh("~");

  std::string calibFileName;
  FisheyeCorrector_Topics topics_base;
  int imgH, imgW;
  bool enFisheyeCorrection;

  // Read parameters passed from the launch file
  //Camera Images topics
  topics_base.subFEImageTopic = DEF_SUB_FEIMAGE_TOPIC;
  topics_base.subPTImageTopic = DEF_SUB_PCIMAGE_TOPIC;
  //CameraInfo topics
  topics_base.subFECamInfo = DEF_SUB_FECAMINFO_TOPIC;
  topics_base.subPTCamInfo = DEF_SUB_PCCAMINFO_TOPIC;

  // privatenh.param<std::string>(PARAM_IMAGE_PUB_TOPIC, topics_base.pubImageTopic, DEF_PUB_IMAGE_TOPIC);

  privatenh.param<std::string>(PARAM_CALIB_FILE_NAME, calibFileName, DEF_CALIB_FILE_NAME);
  privatenh.param<int>(PARAM_CAM_IMG_HEIGHT, imgH, DEF_CAM_IMG_HEIGHT);
  privatenh.param<int>(PARAM_CAM_IMG_WIDTH, imgW, DEF_CAM_IMG_WIDTH);

  topics_base.subUAVPoseTopic = DEF_SUB_UAVPOSE_TOPIC;
  topics_base.subUAVGPSPoseTopic = DEF_SUB_UAVGPSPOSE_TOPIC;
  
  
  privatenh.param<bool>(PARAM_FISH_EYE_CORRECTION, enFisheyeCorrection, true);

  privatenh.param<std::string>(PARAM_SRV_TOPIC, topics_base.srvTopic, DEF_SRV_TOPIC);
  privatenh.param<std::string>(PARAM_LASER_TOPIC, topics_base.laserTopic, DEF_LASER_TOPIC);

  std::cout << "Instantiating fisheyeCorrector object\n";
  FisheyeCorrector fisheyeCorrector(&topics_base, calibFileName,imgH, imgW, enFisheyeCorrection);
  fisheyeCorrector.start(nh);

  return 0;
}