#include <ros/ros.h>
#include <signal.h>
#include <memory>
#include <queue>
#include <string>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/once.hpp>
#include <boost/thread/locks.hpp>
#include <boost/shared_ptr.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <fstream>

#include "localizer/clog_core.hpp"

// For transform support
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"

//ROS message headers
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "clog_msgs/ScanPose.h"

#define DECODE_OPT_STRING(str) ((std::stoi(std::string(1,str.at(0))) << 7) + \
                     (std::stoi(std::string(1,str.at(1))) << 6) + \
                     (std::stoi(std::string(1,str.at(2))) << 5) + \
                     (std::stoi(std::string(1,str.at(3))) << 4) + \
                     (std::stoi(std::string(1,str.at(4))) << 3) + \
                     (std::stoi(std::string(1,str.at(5))) << 2) + \
                     (std::stoi(std::string(1,str.at(6))) << 1) + \
                     (std::stoi(std::string(1,str.at(7)))))

namespace MBZIRC
{

template <class T>
bool from_string(T &t,
                 const std::string &s,
                 std::ios_base &(*f)(std::ios_base &))
{
  std::istringstream iss(s);
  return !(iss >> f >> t).fail();
}

//Wrapper class to interface with CLOG library APIs
class CLOGlocaliser
{
  typedef enum __cameraType {NOT_SELECTED = -1, FISHEYE_CAMERA = 0, PT_CAMERA = 1} cameraType;

public:
  //Singleton
  static boost::shared_ptr<CLOGlocaliser> getInstance(void);
  virtual ~CLOGlocaliser();   //Destructor

  //Modifiers
  void subscribeRos();

  //ROS Subscribers
  void laserSubscriberCb(const clog_msgs::ScanPoseConstPtr &laserpose);
  void poseSubscriberCb(const nav_msgs::OdometryConstPtr &pose);
  void rtkposeSubscriberCb(const nav_msgs::OdometryConstPtr &pose);
  //thread entry : this thread is responsible for publishing the 6DOF pose of the UAV
  void startThreads(void);

  void timerCallback(const ros::TimerEvent& e);
private:
  //Private Methods
  //Private constructor
  CLOGlocaliser();
  
  //Private Modifiers
  void publish_pose(void);

  //Helper function to get yaw from pose
  double getYaw(tf::Pose &t);
  
  void printStatus(std::ostream &out,cd_debug_struct & ds, const clog_msgs::ScanPoseConstPtr &laserpose);

  void processThread(void);

  void startSrv(void);
  void continuestartSrv(void);
  void stopSrv(void);

private:
  //Private Members

  // Parameters for frames and topics
  std::string odomFrameId;
  std::string baseLinkFrameId;
  std::string globalFrameId;
  std::string imageSubTopic;
  std::string posePubTopic;
  std::string poseSubTopic;
  std::string mapImagePath;
  std::string rtkPoseTopic;
  std::string laserTopic;
  std::string fisheyeServiceName;

  double m_xoffset, m_yoffset;

  double m_map_resolution;
  double m_fisheye_camera_f;
  double m_perspective_camera_f;
  double m_rtkfix_timer_duration;
  uint8_t m_optim_config;

  //flag the status of the ROS node
  bool m_running;
  bool m_clog_activate;
  bool m_clog_busy;
  bool m_data_available;
  bool m_pose_available;
  bool m_debug;
  bool m_exitflag;

  clog_msgs::ScanPoseConstPtr m_laserpose;
  nav_msgs::OdometryConstPtr m_gpspose;
  boost::thread *mp_process_thread;
  boost::mutex newUpdateMtx;
  boost::condition_variable   newUpdateCondition_;

  //pose publisher
  ros::Publisher m_posepub;
  // Image publisher : debugging purposes
  ros::Publisher m_debug_imagepub;

  ros::Time img_time;
  ros::Timer m_rtkfix_timer;

  //frame counter
  uint32_t m_seqnum;

  //Transform variables and objects
  tf::TransformBroadcaster *tfb_;
  tf::TransformListener *tf_;
  ros::Duration transform_tolerance_;

  pose_vector current_pose;

  //This flag is used indicate the availability of an initial pose
  bool m_posUpdateIndicate;
  //This flag is used to indicate the readiness of the system, i.e. at least one inituial pose has been arrived.
  bool m_initStatus;

  //Once everything is settled we need to add real CLOG objcet to this class
  //so that we can access its APIs to localise the UAV using the image-based
  //message

  static boost::shared_ptr<CLOGlocaliser> localiser_obj;

  //Service client for calling the image node
  ros::ServiceClient m_cameraclient;

  //Selected camera type
  cameraType m_cam_type_;
  std::ofstream ofs_;

  // CLOG core object
  static clog_core clogcore_obj;
};
}
