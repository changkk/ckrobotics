/*
  Fisheye Corrector class header file
*/
#ifndef LOCALIZER_FISHEYE_CORRECTOR
#define LOCALIZER_FISHEYE_CORRECTOR

// ROS dependencies
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "sensor_msgs/CameraInfo.h"
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "localizer/ocam_functions.h"

// C++ dependencies
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/once.hpp>
#include <boost/thread/locks.hpp>
//Project dependencies
#include <clog_msgs/ScanPose.h>
#include <clog_msgs/imgForward.h>

typedef struct _FisheyeCorrector_Topics
{
    std::string subFEImageTopic;
    std::string subPTImageTopic;
    std::string subFECamInfo;
    std::string subPTCamInfo;
    std::string pubImageTopic;
    std::string subUAVPoseTopic;
    std::string subUAVGPSPoseTopic;
    std::string srvTopic;
    std::string laserTopic;

} FisheyeCorrector_Topics;


class FisheyeCorrector
{
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry> ImgPoseSyncPolicy;

private:
  static const int SUB_IMAGE_QUEUE_SIZE;
  static const int PUB_IMAGE_QUEUE_SIZE;
  static const double MAP_RESOLUTION;

  //FisheyeCorrector object states 
  static const uint8_t FISHEYECORR_STATE_INITIALISED;
  static const uint8_t FISHEYECORR_STATE_FISHEYE_CAMERAINFO;
  static const uint8_t FISHEYECORR_STATE_PANTILT_CAMERAINFO;
  static const uint8_t FISHEYECORR_STATE_FISHEYEPANTILT_CAMERAINFO;
  static const uint8_t FISHEYECORR_STATE_READY;
  static const uint8_t FISHEYECORR_STATE_BUSY;

  FisheyeCorrector_Topics* _topic_base;
  std::string calibFileName;

  int imgH;
  int imgW;

  bool enFisheyeCorrection;

  ros::Subscriber imageSubscriber;
  ros::Publisher imagePublisher;
  ros::Publisher scanPublisher;
  ros::Subscriber fisheye_camera_info_sub;
  ros::Subscriber pt_camera_info_sub;
  ros::NodeHandle* nh_;
  message_filters::Subscriber<nav_msgs::Odometry> pose_sub_;
  message_filters::Subscriber<sensor_msgs::Image> img_sub_;
  sensor_msgs::CameraInfo fisheye_camera_info_;
  sensor_msgs::CameraInfo pantilt_camera_info_;

  boost::thread *cvThread;
  boost::mutex newImgMtx;
  boost::mutex statusMtx;
  boost::condition_variable   newImgCondition_;
  bool newImage_;
  bool exitflag_;
  uint8_t cameratype_;

  /* Interpretation of statusFlag
     values : 0x00 - initialised
              0x01 - Fisheye camera CameraInfo topic received
              0x02 - Pan-tilt camera CameraInfo topic receied
              0x03 - Both Fisheye camera and Pan-tilt camera CameraInfo topics received
              0x07 - fisheye_corrector object is ready to process incoming messages
              0x0F - fisheye_corrector object is busy to process incoming messages
   */           
  uint8_t statusFlag; 

  //Container to publish extracted edge points as a laser scan together with the UAV pose estimate.
  //fisheye_corrector node will publish this topic
  clog_msgs::ScanPose out_msg_;
  nav_msgs::Odometry gpspose_;
 
  //Counter to generate the sequnce number for ScanPose messages.
  uint32_t msg_cnt_;

  ocam_model ocamModel;

  cv_bridge::CvImagePtr cvPtr;

  int skippedImgCnt;

  ros::ServiceServer scanService_;

  message_filters::Synchronizer<ImgPoseSyncPolicy>* sync_;
  
  void imageSubscriberCb(const sensor_msgs::Image::ConstPtr &);
  void gpsSubscriberCb(const nav_msgs::OdometryConstPtr &);
  void imgposeSubscriberCb(const sensor_msgs::Image::ConstPtr&, const nav_msgs::OdometryConstPtr&);
  void cvProcess();
  void fisheye_camera_info_cb(const sensor_msgs::CameraInfo::ConstPtr& );
  void pt_camera_info_cb(const sensor_msgs::CameraInfo::ConstPtr& );
  bool serviceInterface(clog_msgs::imgForward::Request&, clog_msgs::imgForward::Response&);

public:
  FisheyeCorrector(FisheyeCorrector_Topics *, std::string, int, int, bool);
  void start(ros::NodeHandle);

  virtual ~FisheyeCorrector();
  std::string type2str(int type);
};

#endif
