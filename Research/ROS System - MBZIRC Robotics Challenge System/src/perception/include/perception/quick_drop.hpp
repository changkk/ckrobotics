/*
  Fisheye Corrector class header file
*/
#ifndef TEST_FISHEYE_INCLUDE
#define TEST_FISHEYE_INCLUDE

// ROS dependencies
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "sensor_msgs/CameraInfo.h"
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include "mavros_msgs/Mavlink.h"
#include "mavros_msgs/State.h"
#include "sensor_msgs/Imu.h"

// C++ dependencies
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/once.hpp>
#include <boost/thread/locks.hpp>
#include <ctype.h>


class test_challenge1
{
  typedef enum challenge1State__ { CHAL1_INIT=0, CHAL1_MONIPREPARE=1, CHAL1_MONIREADY= 2, 
                                   CHAL1_TRUCKDETECTED_STAGE1 =3, CHAL1_TRUCKDETECTED_STAGE2 =4,
                                   CHAL1_LAUNCH=5} challenge1State;

  typedef boost::mutex::scoped_lock slock;

private:

  //Private Members

  bool armed , guided ;
  std::string flightmode;
  bool start_process;
  bool laser_flag;
  float distance_totruck;
  //Subscribers
  ros::Subscriber start_sub;
  ros::Subscriber mavros_sub;
  ros::Subscriber pose_sub;
  ros::Subscriber laser_sub;
  ros::Subscriber lidar_sub;

  
  //Publishers
  ros::Publisher point_pub;
  ros::Publisher landing_pub;
  ros::Publisher takeoff_pub;
  ros::Publisher vel_pub;

  //ROS node handler
  ros::NodeHandle* nh_;

  cv::Point3f monitor_pose;
  cv::Point3f uav_pose;
  float disx_err, disy_err, disz_err;

  //State machine
  challenge1State m_state;
  boost::mutex processMtx1;

  boost::thread *Thread1;
  bool exitflag_;

  ros::Time tStart, tMonitor;

  //Private methods
  void start_callback(const std_msgs::Empty& msg);
  void chatterCallback(const nav_msgs::Odometry& msg);
  void MavrosStateCallback(const mavros_msgs::State msg);
  void lidar_callback(const std_msgs::Float64::ConstPtr& msg);
  void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void cvProcess2();

public:
  
  test_challenge1();
  void start(ros::NodeHandle);

  virtual ~test_challenge1();

};

#endif
