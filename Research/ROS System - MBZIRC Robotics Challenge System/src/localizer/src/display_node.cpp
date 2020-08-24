/*
  This node is only used to display localization on RVIZ for the purpose of 2nd progress report
*/

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

const std::string pubTopic = "/uav_pose";
const std::string subTopic = "/localizer/rtk_pose";
const std::string pub2DTopic = "/localizer/uav_pose_2D";

const float resolution =  0.22831050228;
const int imgHeight = 526;
const int imgWidth = 526;

class Displayer {
  public:
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Publisher pub2D;
    tf::TransformBroadcaster tfBroadcaster;
    cv::Mat map2D;

    void subscriberCb(const geometry_msgs::Pose::ConstPtr&);
};

void Displayer::subscriberCb(const geometry_msgs::Pose::ConstPtr& msg) {
  geometry_msgs::TransformStamped odomTrans; 
  nav_msgs::Odometry odom;

  odomTrans.header.stamp = ros::Time::now();
  odomTrans.header.frame_id = "map";
  odomTrans.child_frame_id = "base_link";
  
  odomTrans.transform.translation.x = msg->position.x;
  odomTrans.transform.translation.y = msg->position.y;
  odomTrans.transform.translation.z = msg->position.z;
  odomTrans.transform.rotation = msg->orientation;

  this->tfBroadcaster.sendTransform(odomTrans);

  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "map";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = msg->position.x;
  odom.pose.pose.position.y = msg->position.y;
  odom.pose.pose.position.z = msg->position.z;
  odom.pose.pose.orientation = msg->orientation;

  this->pub.publish(odom);

  // Calculate 2D pixels
  int xPx = imgHeight/2 + msg->position.x / resolution;
  int yPx = imgWidth/2 - msg->position.y / resolution;
  cv::Mat bgImg = this->map2D.clone();
  cv::Point p = cv::Point(xPx, yPx);

  cv::circle(bgImg, p, 3, cv::Scalar(255,0,0), -1); 

  sensor_msgs::ImagePtr pubImage = cv_bridge::CvImage(std_msgs::Header(), "rgb8", bgImg).toImageMsg();

  this->  pub2D.publish(pubImage);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "display_node");
  ros::NodeHandle nh;
  Displayer dp;

  dp.map2D = cv::imread("/home/victor/maps/mbzirc_map_edited.png", CV_LOAD_IMAGE_COLOR);
  dp.pub = nh.advertise<nav_msgs::Odometry>(pubTopic, 10, false);
  dp.pub2D = nh.advertise<sensor_msgs::Image>(pub2DTopic, 10, false);
  dp.sub = nh.subscribe(subTopic, 10, &Displayer::subscriberCb, &dp);

  ros::spin();
  
  return 0;
}
