#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include "clog_msgs/ScanPose.h"


typedef struct
{
  double v[3];
} pose_vector;

typedef struct _cd_debug_struct
{
   std::vector<double> out_pose;
   double              cd_val;
   uint16_t            cnt;
   double              t_val;
} cd_debug_struct;

class clog_core
{
public:
  clog_core();
  void initialise(cv::Mat &img, double map_resolution, double m_xoffset, double m_yoffset);
  geometry_msgs::PoseStamped processlaser(const clog_msgs::ScanPoseConstPtr &laserpose,
                                          uint8_t *clog_status, uint8_t degrees_f, 
                                          double camera_f, cd_debug_struct *dreport
                                          );                   
  void set_pubhandle(ros::Publisher debug_imagepub);
};
