#include "localizer/clog_localiser.hpp"

#include <image_transport/subscriber_filter.h>
#include <tf/message_filter.h>
#include <tf/tf.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <clog_msgs/imgForward.h>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "opencv2/opencv.hpp"
#include <bitset>
#include <chrono>

using namespace MBZIRC;
using namespace boost;
using namespace cv;

boost::shared_ptr<CLOGlocaliser> CLOGlocaliser::localiser_obj;

#define PARAM_ODOM_FRAME_ID "frame_id/odom"
#define PARAM_BASE_LINK_FRAME_ID "frame_id/base_link"
#define PARAM_GLOBAL_FRAME_ID "frame_id/global"
#define PARAM_IMAGE_SUB_TOPIC "transformed_image_topic"
#define PARAM_POSE_PUB_TOPIC "pose_pub_topic"
#define PARAM_POSE_SUB_TOPIC "pose_sub_topic"
#define PARAM_MAP_IMAGE_PATH "map_image_path"
#define PARAM_TF_TOLERANCE "transform_tolerance"
#define PARAM_RTK_POSE_TOPIC "rtk_pose_topic"
#define PARAM_MAP_RESOLUTION "map_resolution"
#define PARAM_FISHEYE_CAMERA_F "fisheye_camera_f_length"
#define PARAM_PERS_CAMERA_F "perspective_camera_f_length"
#define PARAM_LASER_TOPIC "laser_topic"
#define PARAM_DEGREES_OF_FREEDOM "optimisation_dof"
#define PARAM_XOFFSET "xoffset"
#define PARAM_YOFFSET "yoffset"
#define PARAM_DEBUG "debug"
#define PARAM_RTKFIX_TIMER "rtkfix_timer"
#define PARAM_FISHEYE_SRV "fisheye_service_topic"

#define DEF_ODOM_FRAME_ID "/odom"
#define DEF_BASE_LINK_FRAME_ID "/base_link"
#define DEF_GLOBAL_FRAME_ID "/map"
#define DEF_IMAGE_SUB_TOPIC "/localizer/tf_image"
#define DEF_POSE_PUB_TOPIC "localizer/clog_pose"
#define DEF_POSE_SUB_TOPIC "/rtk_pose"
#define DEF_MAP_IMAGE_PATH "data/mbzirc_arena.png"
#define DEF_TF_TOLERANCE 0.1
#define DEF_RTK_POSE_TOPIC "/localizer/rtk_pose"
#define DEF_LASER_TOPIC "scanpos"
#define DEGREES_OF_FREEDOM 0b10111001
#define DEF_FISHEYE_SRV "localizer/fisheye_srv"

#define MAP_RESOLUTION 0.05
// #define MAP_RESOLUTION 0.1255
#define CAMERA_F 12.0
#define PT_CAMERA_THRESHOLD .10
#define POSE_COVARIANCE_THRESHOLD 0.0

#define START_SUBSCRIPTION 1
#define STOP_SUBSCRIPTION 3
#define RESUME_SUBSCRIPTION 2


#define PT_MODE 0b10000000
#define FISHEYE_MODE 0b01111111

clog_core CLOGlocaliser::clogcore_obj = clog_core();

//MBZIRC::CLOGlocaliser::getInstance- singleton
shared_ptr<CLOGlocaliser> CLOGlocaliser::getInstance(void)
{
  std::cout << "Calling CLOGlocaliser singleton" << std::endl
            << std::flush;

  if (localiser_obj.get() == 0)
  {
    localiser_obj = boost::shared_ptr<CLOGlocaliser>(new CLOGlocaliser());
  }

  return localiser_obj;
}

//MBZIRC::CLOGlocaliser::CLOGlocaliser - private constructor
CLOGlocaliser::CLOGlocaliser() : m_running(false),
                                 m_clog_activate(false), //Should be false for normal operation!!!!!!!
                                 m_clog_busy(false),
                                 m_data_available(false),
                                 m_pose_available(false),
                                 //  clogcore_obj(new clog_core()),
                                 m_posUpdateIndicate(false),
                                 m_initStatus(false),
                                 m_debug(false),
                                 m_exitflag(false),
                                 m_cam_type_(NOT_SELECTED),
                                 ofs_("debug.csv", std::ofstream::out)
{
  std::cout << "CLOGlocaliser constructor" << std::endl << std::flush;

}

//MBZIRC::CLOGlocaliser::~CLOGlocaliser - destructor
CLOGlocaliser::~CLOGlocaliser()
{
  std::cout << "CLOGlocaliser destructor" << std::endl << std::flush;

  {
    newUpdateMtx.lock();
    m_exitflag = true;
    newUpdateMtx.unlock();
    newUpdateCondition_.notify_all();
  }
  
  mp_process_thread->interrupt();
  mp_process_thread->join();
  delete[] mp_process_thread;
  ofs_.close();
}

//MBZIRC::CLOGlocaliser::laserSubscriberCb - laser call back to receive the
//edge coordinates.
void CLOGlocaliser::laserSubscriberCb(const clog_msgs::ScanPoseConstPtr &laserpose)
{
  //std::cout << "laserSubscriberCb\n";
  bool clog_status;

  newUpdateMtx.lock();
  clog_status = m_clog_activate;
  newUpdateMtx.unlock();

  if (clog_status )
  {
    newUpdateMtx.lock();
    m_laserpose = laserpose;
    m_data_available = true;
    if (laserpose->imgType == 0)
    {
      m_optim_config &= FISHEYE_MODE;
    }
    else
    {
      m_optim_config |= PT_MODE;
    }
    newUpdateMtx.unlock();
    newUpdateCondition_.notify_all();
  }
}

void CLOGlocaliser::poseSubscriberCb(const nav_msgs::OdometryConstPtr &pose)
{
  //std::cout << "poseSubscriberCb\n"; 
  bool clog_status;

  newUpdateMtx.lock();
  clog_status = m_clog_activate;
  newUpdateMtx.unlock();

  if (!clog_status)
  {
    newUpdateMtx.lock();
    m_gpspose = pose;
    m_pose_available = true;
    newUpdateMtx.unlock();
    newUpdateCondition_.notify_all();
  }

}

void CLOGlocaliser::timerCallback(const ros::TimerEvent& e)
{
  ROS_INFO("Time between callbacks: %f", (e.current_real - e.last_real).toSec());
  if (e.last_real.toSec()==0)
  {
    m_rtkfix_timer.stop();
    m_rtkfix_timer.setPeriod(ros::Duration(m_rtkfix_timer_duration));
    m_rtkfix_timer.start();
    std::cout << "reinitialising the timer\n";
    return;
  }

  bool  clog_busy;
  newUpdateMtx.lock();
  clog_busy = m_clog_busy;
  newUpdateMtx.unlock();
 
  if(!clog_busy)
  {
    newUpdateMtx.lock();
    if (!m_clog_activate)
    {
      //We don't have any height information 
      m_cam_type_ = PT_CAMERA;
      startSrv();
    }
    newUpdateMtx.unlock();
  }  
  m_rtkfix_timer.stop();
  m_rtkfix_timer.setPeriod(ros::Duration(m_rtkfix_timer_duration));
}

void CLOGlocaliser::startSrv(void)
{
  clog_msgs::imgForward srv;

  std::cout << "calling CLOGlocaliser::startSrv\n";

  srv.request.cmd = START_SUBSCRIPTION;
  srv.request.type = m_cam_type_;
  m_cameraclient.call(srv);

  if(srv.response.ret == 1)
  {
    m_clog_activate = true;
  }
}

void CLOGlocaliser::continuestartSrv(void)
{
  clog_msgs::imgForward srv;
  
  std::cout << "calling CLOGlocaliser::continuestartSrv\n";

  srv.request.cmd = RESUME_SUBSCRIPTION;
  srv.request.type = m_cam_type_;
  m_cameraclient.call(srv);
  if(srv.response.ret == 1)
  {
    m_clog_activate = true;
  }
}

void CLOGlocaliser::stopSrv(void)
{
  clog_msgs::imgForward srv;

  std::cout << "calling CLOGlocaliser::stopSrv\n";
  srv.request.cmd = STOP_SUBSCRIPTION;
  srv.request.type = m_cam_type_;

  m_cameraclient.call(srv);
  if(srv.response.ret == 1)
  {
    m_clog_activate = false;
  }
}


void CLOGlocaliser::rtkposeSubscriberCb(const nav_msgs::OdometryConstPtr& pose)
{
   bool clog_busy;

   std::cout << "rtkposeSubscriberCb -- m_clog_activate= " << m_clog_activate << ", " << pose->pose.covariance[0] << "\n";
   
   newUpdateMtx.lock();
   clog_busy = m_clog_busy;
   newUpdateMtx.unlock();
   
   if(!clog_busy)
   {
     newUpdateMtx.lock();
     if (true) //(pose->pose.covariance[0]>=POSE_COVARIANCE_THRESHOLD)
     {
       if (!m_clog_activate)
       {
         if (pose->pose.pose.position.z > PT_CAMERA_THRESHOLD)
         {
           m_cam_type_ = PT_CAMERA;
         }
         else
         {
           m_cam_type_ = FISHEYE_CAMERA;
         }
         std::cout << "m_cam_type_= " << m_cam_type_ << " " << pose->pose.pose.position.z << "\n";
         startSrv();
         //Stop the timer
         m_rtkfix_timer.stop();
       }
       else
       {
         if ((pose->pose.pose.position.z > PT_CAMERA_THRESHOLD) && (m_cam_type_== FISHEYE_CAMERA))
         {
           stopSrv();
           m_cam_type_ = PT_CAMERA;
           startSrv();
         }
         else if((pose->pose.pose.position.z < PT_CAMERA_THRESHOLD) && (m_cam_type_== PT_CAMERA))
         {
           stopSrv();
           m_cam_type_ = FISHEYE_CAMERA;
           startSrv();
         }
       }
     }
     else
     {
       if (m_clog_activate)
       {
         stopSrv();
         //Stop the timer
         m_rtkfix_timer.setPeriod(ros::Duration(m_rtkfix_timer_duration));
         m_rtkfix_timer.start();
       }
       else
       {
         m_rtkfix_timer.stop();
         m_rtkfix_timer.setPeriod(ros::Duration(m_rtkfix_timer_duration));
         m_rtkfix_timer.start();
       }
     }
     newUpdateMtx.unlock();
   }
     
}

void CLOGlocaliser::processThread(void)
{
  bool exitflag = false;
  clog_msgs::ScanPoseConstPtr laserpose;
  nav_msgs::OdometryConstPtr gpspose;
  bool data_available = false,  pose_available = false, clog_status=false;
  uint8_t optim_config;
  double camera_f;

  ROS_INFO("Process thread started");

  {
    newUpdateMtx.lock();
    exitflag = m_exitflag;
    newUpdateMtx.unlock();
  }

  while (!exitflag && ros::ok())
  {
    {
      boost::unique_lock<boost::mutex> lock(newUpdateMtx);
      //std::cout << "waiting for condition\n";
      while (!m_data_available && !m_pose_available) {
        newUpdateCondition_.wait(lock);
        //std::cout << "out of waiting condition\n";
        if(m_exitflag)
        {
          exitflag=m_exitflag;
          continue;
        }
      }
    }

    newUpdateMtx.lock();
    data_available = m_data_available;
    pose_available = m_pose_available;
    if (data_available)
    {
      laserpose = m_laserpose;
      m_laserpose.reset();
      m_data_available = false;
      optim_config = m_optim_config;
      camera_f = (m_optim_config & PT_MODE) ? m_perspective_camera_f : m_fisheye_camera_f;
    }
    else if (pose_available)
    {
      gpspose = m_gpspose;
      m_pose_available = false;
      m_gpspose.reset();
    }
    newUpdateMtx.unlock();

    if (data_available)
    {
      ROS_INFO("Porcessing message");
      uint8_t *clog_status;

      if (laserpose->scan.ranges.size() > 100)
      {
        newUpdateMtx.lock();
        m_clog_busy = true;
        newUpdateMtx.unlock();

        cd_debug_struct  dreport_;
        dreport_.out_pose.resize(6);

        geometry_msgs::PoseStamped out_geopose = clogcore_obj.processlaser(laserpose, clog_status, optim_config, camera_f,&dreport_);

        printStatus(std::cout, dreport_,laserpose );
        laserpose.reset();
    
        newUpdateMtx.lock();
        m_clog_busy = false;
        newUpdateMtx.unlock();

        nav_msgs::Odometry out_odom;
        out_odom.header = out_geopose.header;
        out_odom.pose.pose = out_geopose.pose;
        m_posepub.publish(out_odom);
      }
      else
      {
        ROS_ERROR("Poor visibility, Cannot localize");
      }
    }
    else if (pose_available)
    {
      m_posepub.publish(gpspose);
    }

    newUpdateMtx.lock();
    exitflag = m_exitflag;
    clog_status =m_clog_activate;
    newUpdateMtx.unlock();

    if(!exitflag && clog_status)
    {
      continuestartSrv();
    }
  }
  std::cout << "discontinue\n";
  boost::this_thread::interruption_point();
}

void CLOGlocaliser::printStatus(std::ostream &out, cd_debug_struct & ds, const clog_msgs::ScanPoseConstPtr &laserpose)
{
  out << "\n  " << std::setw(5) << "CD= " << ds.cd_val << ", opt_time= " << ds.t_val 
            << ", cnt= " << ds.cnt << std::endl << std::flush;

  std_msgs::Header hrtk = laserpose->gps.header;
  std_msgs::Header hodom = laserpose->pose.header;

  ros::Duration dtime = hrtk.stamp - hodom.stamp;
  double rtk_secs =hrtk.stamp.toSec();
  double odom_secs = hodom.stamp.toSec();

  //std::cout << "  " << std::setw(5) << "rtk time= " << boost::posix_time::to_simple_string(hrtk.stamp.toBoost()).c_str()
  //         << "odom time " << boost::posix_time::to_simple_string(hodom.stamp.toBoost()).c_str() << std::endl << std::flush;
  out << "  " << std::setw(5) << "rtk time= " << rtk_secs << " odom time= " << odom_secs << ", diff= " << dtime << std::endl << std::flush;
    
  out << "  " << std::setw(5) << "X= " << laserpose->gps.pose.pose.position.x << ", " 
            << laserpose->pose.pose.pose.position.x << ", " << ds.out_pose[0] << std::endl << std::flush;
  out << "  " << std::setw(5) << "Y= " << laserpose->gps.pose.pose.position.y << ", " 
            << laserpose->pose.pose.pose.position.y << ", " << ds.out_pose[1] << std::endl << std::flush;
  out << "  " << std::setw(5) << "Z= " << laserpose->gps.pose.pose.position.z << ", " 
            << laserpose->pose.pose.pose.position.z << ", " << ds.out_pose[2] << std::endl << std::flush;
      
  tf::Quaternion q(laserpose->pose.pose.pose.orientation.x,
                   laserpose->pose.pose.pose.orientation.y,
                   laserpose->pose.pose.pose.orientation.z,
                   laserpose->pose.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  std::vector<double> ori(3);
  m.getRPY(ori[0], ori[1], ori[2]);

  std::cout << "  " << std::setw(5) << "roll= " << ori[0] << ", " << ds.out_pose[3] << std::endl << std::flush;
  std::cout << "  " << std::setw(5) << "pitch= " << ori[1] << ", " << ds.out_pose[4] << std::endl << std::flush;
  std::cout << "  " << std::setw(5) << "yaw= " << ori[2] << ", " << ds.out_pose[5] << std::endl << std::flush;
  std::cout << std::endl << std::flush;

  ofs_ << ds.cd_val << "," << ds.t_val << "," << ds.cnt << "," << rtk_secs << "," << odom_secs << "," 
      << laserpose->gps.pose.pose.position.x << "," << laserpose->pose.pose.pose.position.x << "," << ds.out_pose[0] << ","
      << laserpose->gps.pose.pose.position.y << "," << laserpose->pose.pose.pose.position.y << "," << ds.out_pose[1] << ","
      << laserpose->gps.pose.pose.position.z << "," << laserpose->pose.pose.pose.position.z << "," << ds.out_pose[2] << ","
      << ori[0] << "," << ds.out_pose[3] << "," << ori[1] << "," << ds.out_pose[4] << "," << ori[2] << "," << ds.out_pose[5] 
      << std::endl << std::flush;
}

void CLOGlocaliser::startThreads(void)
{
  mp_process_thread = new boost::thread(&CLOGlocaliser::processThread, this);
}

//Handle subscription to ROS topic
void CLOGlocaliser::subscribeRos()
{
  double tfTolerance;
  std::string deg_freedom_str;
  ros::NodeHandle n;

  ros::NodeHandle pnh("~");

  pnh.param<std::string>(PARAM_ODOM_FRAME_ID, this->odomFrameId, DEF_ODOM_FRAME_ID);
  pnh.param<std::string>(PARAM_BASE_LINK_FRAME_ID, this->baseLinkFrameId, DEF_BASE_LINK_FRAME_ID);
  pnh.param<std::string>(PARAM_GLOBAL_FRAME_ID, this->globalFrameId, DEF_GLOBAL_FRAME_ID);
  pnh.param<std::string>(PARAM_IMAGE_SUB_TOPIC, this->imageSubTopic, DEF_IMAGE_SUB_TOPIC);
  pnh.param<std::string>(PARAM_POSE_PUB_TOPIC, this->posePubTopic, DEF_POSE_PUB_TOPIC);
  pnh.param<std::string>(PARAM_POSE_SUB_TOPIC, this->poseSubTopic, DEF_POSE_SUB_TOPIC);
  pnh.param<std::string>(PARAM_MAP_IMAGE_PATH, this->mapImagePath, DEF_MAP_IMAGE_PATH);
  pnh.param<std::string>(PARAM_LASER_TOPIC, this->laserTopic, DEF_LASER_TOPIC);
  pnh.param<double>(PARAM_TF_TOLERANCE, tfTolerance, DEF_TF_TOLERANCE);
  pnh.param<std::string>(PARAM_RTK_POSE_TOPIC, this->rtkPoseTopic, DEF_RTK_POSE_TOPIC);
  pnh.param<double>(PARAM_MAP_RESOLUTION, this->m_map_resolution, MAP_RESOLUTION);
  pnh.param<double>(PARAM_FISHEYE_CAMERA_F, this->m_fisheye_camera_f, CAMERA_F);
  pnh.param<double>(PARAM_PERS_CAMERA_F, this->m_perspective_camera_f, CAMERA_F);
  pnh.param<std::string>(PARAM_DEGREES_OF_FREEDOM, deg_freedom_str, "");
  pnh.param<double>(PARAM_XOFFSET, this->m_xoffset, 0);
  pnh.param<double>(PARAM_YOFFSET, this->m_yoffset, 0);
  pnh.param<bool>(PARAM_DEBUG, this->m_debug, false);
  pnh.param<double>(PARAM_RTKFIX_TIMER, this->m_rtkfix_timer_duration, 0.1);
  pnh.param<std::string>(PARAM_FISHEYE_SRV, this->fisheyeServiceName, DEF_FISHEYE_SRV);

  if (deg_freedom_str.length() != 8)
  {
    ROS_ERROR_STREAM("Option string invalid: " << deg_freedom_str << ". Please check the parameter.");
    m_optim_config = DEGREES_OF_FREEDOM;
  }
  else
    m_optim_config = DECODE_OPT_STRING(deg_freedom_str);

  ROS_INFO_STREAM("Optimisation Configuration set to: " << std::bitset<8>(m_optim_config));

  // this->transform_tolerance_.fromSec(tfTolerance);
  // this->tfb_ = new tf::TransformBroadcaster();
  // this->tf_ = new tf::TransformListener();

  //subscribe to topics: eg. sensor_msgs::ImageConstPtr
  //you need to specify the proper topic name here

  //Subscribe to receive the initial pose
  ros::Subscriber laser_sub = n.subscribe(this->laserTopic, 1, &CLOGlocaliser::laserSubscriberCb, this);
  ros::Subscriber pose_sub = n.subscribe(this->poseSubTopic, 1, &CLOGlocaliser::poseSubscriberCb, this);
  ros::Subscriber rtkpose_sub = n.subscribe(this->rtkPoseTopic, 1, &CLOGlocaliser::rtkposeSubscriberCb, this);
  
  m_cameraclient = n.serviceClient<clog_msgs::imgForward>(this->fisheyeServiceName);

  //Refgister a timer callback
  m_rtkfix_timer = n.createTimer(ros::Duration(this->m_rtkfix_timer_duration), &CLOGlocaliser::timerCallback, this);
  

  if (m_debug)
  {
    m_debug_imagepub = n.advertise<sensor_msgs::Image>("debug_image", 10, true);
    clogcore_obj.set_pubhandle(m_debug_imagepub);
  }

  //set up to publish UAV pose
  m_posepub = n.advertise<nav_msgs::Odometry>(posePubTopic, 2);

  std::cout << "rostopic subscription completed" << std::endl
            << std::flush;

  cv::Mat img = cv::imread(mapImagePath.c_str(), 0);

  if (img.empty())
  {
    ROS_ERROR("Could not read MapImage.");
    ros::shutdown();
  }  

  clogcore_obj.initialise(img, m_map_resolution, m_xoffset, m_yoffset);

  this->startThreads();

  ros::spin();
}

int main(int argc, char **argv)
{

  boost::shared_ptr<CLOGlocaliser> locObj = CLOGlocaliser::getInstance();

  ros::init(argc, argv, "clog_localiser_node");

  locObj->subscribeRos();

  
  std::cout << "Exit Ros spin" << std::endl
            << std::flush;
  return 0;
}
