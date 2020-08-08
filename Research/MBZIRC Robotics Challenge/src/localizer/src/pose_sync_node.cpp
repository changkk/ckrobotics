/*
  This node time-synchronises data from ~/mavros/imu/data and ~/gps/rtk_fix and publishes odom msg with pose from gps frame
  Publishes tf from rtkgps frame to rtkreciever frame and arm frame

  Note : Both GPS and IMU publishes data in an ENU coordinate system

  Input :
    Subscribes to ~/mavros/imu/data and ~/gps/rtk_fix messages

  Output :
    Publishes an odom msg from gps frame

  Author  : janindu@vt.edu
*/

// ROS dependencies
#include <ros/ros.h>

// Project dependencies
#include "pose_sync.cpp"

static const std::string PARAM_IMU_SUB_TOPIC = "/sub_imu_topic";
static const std::string PARAM_RTKGPS_SUB_TOPIC = "/sub_rtkgps_topic";
static const std::string PARAM_ODOM_PUB_TOPIC = "/pub_odom_topic";
static const std::string PARAM_RTK_BASE_FRAME_ID = "/rtk_base_frame_id";
static const std::string PARAM_RTK_RECIEVER_FRAME_ID = "/rtk_reciever_frame_id";
static const std::string PARAM_ARM_FRAME_ID = "/arm_frame_id";
static const std::string PARAM_BASELINK_FRAME_ID = "/baselink_frame_id";
static const std::string PARAM_TF_RTK_RECIEVER_BASE_LINK = "/tf_rtk_reciever_base_link";
static const std::string DEF_IMU_SUB_TOPIC = "mavros/imu/data";
static const std::string DEF_RTKGPS_SUB_TOPIC = "gps/rtkfix";
static const std::string DEF_ODOM_PUB_TOPIC = "localizer/gps_imu_odom";
static const std::string DEF_RTK_BASE_FRAME_ID = "gps";
static const std::string DEF_RTK_RECIEVER_FRAME_ID = "rtk_reciever";
static const std::string DEF_ARM_FRAME_ID = "map";
static const std::string DEF_BASELINK_FRAME_ID = "base_link";
static const std::string DEF_TF_RTK_RECIEVER_BASE_LINK = "0.0 -0.12 -0.23 0.0 0.0 0.0"; // x y z y p r

int main(int argc, char** argv) {
  ros::init(argc, argv, argv[1]);
  ros::NodeHandle nh;

  std::string subImuTopic, subRtkgpsTopic, pubOdomTopic, rtkBaseFrameId, rtkRecieverFrameId, armFrameId, baselinkFrameId, tfRtkrecieverBaselink;

  // Read parameters passed from the launch file
  nh.param<std::string>(argv[1] + PARAM_IMU_SUB_TOPIC, subImuTopic, DEF_IMU_SUB_TOPIC);
  nh.param<std::string>(argv[1] + PARAM_RTKGPS_SUB_TOPIC, subRtkgpsTopic, DEF_RTKGPS_SUB_TOPIC);
  nh.param<std::string>(argv[1] + PARAM_ODOM_PUB_TOPIC, pubOdomTopic, DEF_ODOM_PUB_TOPIC);
  nh.param<std::string>(argv[1] + PARAM_RTK_BASE_FRAME_ID, rtkBaseFrameId, DEF_RTK_BASE_FRAME_ID);
  nh.param<std::string>(argv[1] + PARAM_RTK_RECIEVER_FRAME_ID, rtkRecieverFrameId, DEF_RTK_RECIEVER_FRAME_ID);
  nh.param<std::string>(argv[1] + PARAM_ARM_FRAME_ID, armFrameId, DEF_ARM_FRAME_ID);
  nh.param<std::string>(argv[1] + PARAM_BASELINK_FRAME_ID, baselinkFrameId, DEF_BASELINK_FRAME_ID);
  nh.param<std::string>(argv[1] + PARAM_TF_RTK_RECIEVER_BASE_LINK, tfRtkrecieverBaselink, DEF_TF_RTK_RECIEVER_BASE_LINK);

  PoseSync poseSync(subImuTopic, subRtkgpsTopic, pubOdomTopic, rtkBaseFrameId, rtkRecieverFrameId, armFrameId, baselinkFrameId,  tfRtkrecieverBaselink);
  poseSync.start(nh);

  ros::spin();

  std::cout << "Terminating pose_sync node" << std::endl;

  nh.shutdown();

  return 0;
}
