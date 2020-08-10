#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <vector>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/tf.h>

#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

using namespace cv;
using std::vector;
using namespace std;
ofstream myfile;

ros::Publisher marker_pub;

geometry_msgs::Vector3Stamped normal_vector, normal_vector2, rotated_normal_vector, rotated_normal_vector2;
geometry_msgs::Vector3Stamped rotated_fisheye_vector_usl, rotated_fisheye_vector_usl2, rotated_fisheye_vector_nsl, rotated_fisheye_vector_nsl2;
geometry_msgs::Vector3Stamped fisheye_vector_usl, fisheye_vector_usl2, fisheye_vector_nsl, fisheye_vector_nsl2;
visualization_msgs::Marker arrow_normal, arrow_normal2, arrow_fisheye_usl, arrow_fisheye_usl2, arrow_fisheye_nsl, arrow_fisheye_nsl2, target_marker;

double pi = 3.141592;
double logi_C270_focal = 320/tan(30*pi/180); // in px
bool initialized1, initialized2, initialized3, initialized4, initialized5, initialized6;


void normal_point_yolo(const geometry_msgs::PolygonStamped& msg)	
{

    if(msg.polygon.points.empty()) return;
       
    double normal_x = msg.polygon.points[0].x;
    double normal_y = msg.polygon.points[0].y;
    double imgW = msg.polygon.points[1].x;
    double imgH = msg.polygon.points[1].y;
    double BBS_x = msg.polygon.points[2].x;
		double BBS_y = msg.polygon.points[2].y;

    if(BBS_x < 100)
    {
      double n_x=normal_x-imgW/2;
      double n_y=imgH/2-normal_y;
      // double n_y=imgH/2-(normal_y-BBS_y*0.4);

      normal_vector.header.frame_id = "gimbal_nsl";
      normal_vector.header.stamp = ros::Time(0);


      // x -> front y->left z-> up (RVIZ)
      normal_vector.vector.x = logi_C270_focal; // Front
      normal_vector.vector.y = -n_x; // left
      normal_vector.vector.z = n_y; // up
      //cout<<fl_2<<endl;
      //cout<<"gimbal vec: "<<normal_vector.vector.x<<" "<<normal_vector.vector.y<<" "<<normal_vector.vector.z<<endl;

      arrow_normal.header.frame_id = "gimbal_nsl";
      arrow_normal.header.stamp = ros::Time::now();

      arrow_normal.ns = "arrow_gimbal";
      arrow_normal.id = 0;

      // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      arrow_normal.type = visualization_msgs::Marker::ARROW;
      // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
      arrow_normal.action = visualization_msgs::Marker::ADD;

      arrow_normal.points.resize(2);
      arrow_normal.points[0].x = -0;
      arrow_normal.points[0].y = 0;
      arrow_normal.points[0].z = -0;
      arrow_normal.points[1].x = normal_vector.vector.x*1;
      arrow_normal.points[1].y = normal_vector.vector.y*1;
      arrow_normal.points[1].z = normal_vector.vector.z*1;

      // Set the scale of the mararrow_normal
      arrow_normal.scale.x = 0.1;
      arrow_normal.scale.y = 0.2;

      // Set the color -- be sure to set alpha to something non-zero!
      arrow_normal.color.r = 0.0f;
      arrow_normal.color.g = 1.0f;
      arrow_normal.color.b = 0.0f;
      arrow_normal.color.a = 1.0;

      marker_pub.publish(arrow_normal);

    }

    if(msg.polygon.points.size() > 3)
    {
      double normal_x = msg.polygon.points[3].x;
      double normal_y = msg.polygon.points[3].y;
      double imgW = msg.polygon.points[4].x;
      double imgH = msg.polygon.points[4].y;
      double BBS_x = msg.polygon.points[5].x;
      double BBS_y = msg.polygon.points[5].y;

      if(BBS_x < 100)
      {
        double n_x=normal_x-imgW/2;
        double n_y=imgH/2-normal_y;
        // double n_y=imgH/2-(normal_y-BBS_y*0.4);

        normal_vector2.header.frame_id = "gimbal_nsl";
        normal_vector2.header.stamp = ros::Time(0);


        // x -> front y->left z-> up (RVIZ)
        normal_vector2.vector.x = logi_C270_focal; // Front
        normal_vector2.vector.y = -n_x; // left
        normal_vector2.vector.z = n_y; // up


        arrow_normal2.header.frame_id = "gimbal_nsl";
        arrow_normal2.header.stamp = ros::Time::now();

        arrow_normal2.ns = "arrow_gimbal2";
        arrow_normal2.id = 6;

        arrow_normal2.type = visualization_msgs::Marker::ARROW;
        arrow_normal2.action = visualization_msgs::Marker::ADD;

        arrow_normal2.points.resize(2);
        arrow_normal2.points[0].x = -0;
        arrow_normal2.points[0].y = 0;
        arrow_normal2.points[0].z = -0;
        arrow_normal2.points[1].x = normal_vector2.vector.x*1;
        arrow_normal2.points[1].y = normal_vector2.vector.y*1;
        arrow_normal2.points[1].z = normal_vector2.vector.z*1;

        // Set the scale of the mararrow_normal
        arrow_normal2.scale.x = 0.1;
        arrow_normal2.scale.y = 0.2;

        // Set the color -- be sure to set alpha to something non-zero!
        arrow_normal2.color.r = 0.0f;
        arrow_normal2.color.g = 1.0f;
        arrow_normal2.color.b = 0.0f;
        arrow_normal2.color.a = 1.0;

        marker_pub.publish(arrow_normal2);
        initialized4 = true;
      }
    }

  initialized1 = true;

}

void normal_point_tracker(const geometry_msgs::Twist& msg)	
{
       
    double normal_x = msg.linear.x;
    double normal_y = msg.linear.y;
    double imgW = 640;
    double imgH = 480;

    double n_x=normal_x-imgW/2;
    double n_y=imgH/2-normal_y;
    // double n_y=imgH/2-(normal_y-BBS_y*0.4);

    normal_vector.header.frame_id = "gimbal_nsl";
    normal_vector.header.stamp = ros::Time(0);


    // x -> front y->left z-> up (RVIZ)
    normal_vector.vector.x = logi_C270_focal; // Front
    normal_vector.vector.y = -n_x; // left
    normal_vector.vector.z = n_y; // up
    //cout<<fl_2<<endl;
    //cout<<"gimbal vec: "<<normal_vector.vector.x<<" "<<normal_vector.vector.y<<" "<<normal_vector.vector.z<<endl;

    arrow_normal.header.frame_id = "gimbal_nsl";
    arrow_normal.header.stamp = ros::Time::now();

    arrow_normal.ns = "arrow_gimbal";
    arrow_normal.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    arrow_normal.type = visualization_msgs::Marker::ARROW;
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    arrow_normal.action = visualization_msgs::Marker::ADD;

    arrow_normal.points.resize(2);
    arrow_normal.points[0].x = -0;
    arrow_normal.points[0].y = 0;
    arrow_normal.points[0].z = -0;
    arrow_normal.points[1].x = normal_vector.vector.x*1;
    arrow_normal.points[1].y = normal_vector.vector.y*1;
    arrow_normal.points[1].z = normal_vector.vector.z*1;

    // Set the scale of the mararrow_normal
    arrow_normal.scale.x = 0.1;
    arrow_normal.scale.y = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    arrow_normal.color.r = 0.0f;
    arrow_normal.color.g = 1.0f;
    arrow_normal.color.b = 0.0f;
    arrow_normal.color.a = 1.0;

    marker_pub.publish(arrow_normal);



  initialized1 = true;

}

void fisheye_point_usl(const geometry_msgs::Twist& msg)	
{

    // bool fisheye_front;
    // nh_2.getParam("/range_estimation_correction/fisheye_front2",fisheye_front);

    fisheye_vector_usl.header.frame_id = "omnicam_usl";
    fisheye_vector_usl.header.stamp = ros::Time(0); 


    // x -> front y->left z-> up (RVIZ)
    fisheye_vector_usl.vector.x = msg.linear.z; // Camera frame -> fcu frame
    fisheye_vector_usl.vector.y = -msg.linear.x; // Camera frame -> fcu frame
    fisheye_vector_usl.vector.z = msg.linear.y; // Camera frame -> fcu frame

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    arrow_fisheye_usl.header.frame_id = "omnicam_usl";
    arrow_fisheye_usl.header.stamp = ros::Time::now();

    arrow_fisheye_usl.ns = "arrow_omnicam_usl";
    arrow_fisheye_usl.id = 1;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    arrow_fisheye_usl.type = visualization_msgs::Marker::ARROW;
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    arrow_fisheye_usl.action = visualization_msgs::Marker::ADD;

    arrow_fisheye_usl.points.resize(2);
    arrow_fisheye_usl.points[0].x = 0.0f;
    arrow_fisheye_usl.points[0].y = 0.0f;
    arrow_fisheye_usl.points[0].z = 0.0f;
    arrow_fisheye_usl.points[1].x = fisheye_vector_usl.vector.x*50;
    arrow_fisheye_usl.points[1].y = fisheye_vector_usl.vector.y*50;
    arrow_fisheye_usl.points[1].z = fisheye_vector_usl.vector.z*50;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    arrow_fisheye_usl.scale.x = 0.1;
    arrow_fisheye_usl.scale.y = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    arrow_fisheye_usl.color.r = 1.0f;
    arrow_fisheye_usl.color.g = 0.0f;
    arrow_fisheye_usl.color.b = 0.0f;
    arrow_fisheye_usl.color.a = 1.0;

    marker_pub.publish(arrow_fisheye_usl);


    if(msg.angular.x != 0)
    {
      fisheye_vector_usl2.header.frame_id = "omnicam_usl";
      fisheye_vector_usl2.header.stamp = ros::Time(0); 


      // x -> front y->left z-> up (RVIZ)
      fisheye_vector_usl2.vector.x = msg.angular.z; // Camera frame -> fcu frame
      fisheye_vector_usl2.vector.y = -msg.angular.x; // Camera frame -> fcu frame
      fisheye_vector_usl2.vector.z = msg.angular.y; // Camera frame -> fcu frame

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      arrow_fisheye_usl2.header.frame_id = "omnicam_usl";
      arrow_fisheye_usl2.header.stamp = ros::Time::now();

      arrow_fisheye_usl2.ns = "arrow_omnicam_usl2";
      arrow_fisheye_usl2.id = 2;

      // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      arrow_fisheye_usl2.type = visualization_msgs::Marker::ARROW;
      // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
      arrow_fisheye_usl2.action = visualization_msgs::Marker::ADD;

      arrow_fisheye_usl2.points.resize(2);
      arrow_fisheye_usl2.points[0].x = 0.0f;
      arrow_fisheye_usl2.points[0].y = 0.0f;
      arrow_fisheye_usl2.points[0].z = 0.0f;
      arrow_fisheye_usl2.points[1].x = fisheye_vector_usl2.vector.x*50;
      arrow_fisheye_usl2.points[1].y = fisheye_vector_usl2.vector.y*50;
      arrow_fisheye_usl2.points[1].z = fisheye_vector_usl2.vector.z*50;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      arrow_fisheye_usl2.scale.x = 0.1;
      arrow_fisheye_usl2.scale.y = 0.2;

      // Set the color -- be sure to set alpha to something non-zero!
      arrow_fisheye_usl2.color.r = 1.0f;
      arrow_fisheye_usl2.color.g = 0.0f;
      arrow_fisheye_usl2.color.b = 0.0f;
      arrow_fisheye_usl2.color.a = 1.0;

      marker_pub.publish(arrow_fisheye_usl2);
      initialized5 = true;
    }

  initialized2 = true;


}



void fisheye_point_nsl(const geometry_msgs::Twist& msg)	
{

    // bool fisheye_front;
    // nh_2.getParam("/range_estimation_correction/fisheye_front2",fisheye_front);

    fisheye_vector_nsl.header.frame_id = "omnicam_usl";
    fisheye_vector_nsl.header.stamp = ros::Time(0); 


    // x -> front y->left z-> up (RVIZ)
    fisheye_vector_nsl.vector.x = msg.linear.z; // Camera frame -> fcu frame
    fisheye_vector_nsl.vector.y = -msg.linear.x; // Camera frame -> fcu frame
    fisheye_vector_nsl.vector.z = msg.linear.y; // Camera frame -> fcu frame

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    arrow_fisheye_nsl.header.frame_id = "omnicam_nsl";
    arrow_fisheye_nsl.header.stamp = ros::Time::now();

    arrow_fisheye_nsl.ns = "arrow_omnicam_nsl";
    arrow_fisheye_nsl.id = 3;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    arrow_fisheye_nsl.type = visualization_msgs::Marker::ARROW;
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    arrow_fisheye_nsl.action = visualization_msgs::Marker::ADD;

    arrow_fisheye_nsl.points.resize(2);
    arrow_fisheye_nsl.points[0].x = 0.0f;
    arrow_fisheye_nsl.points[0].y = 0.0f;
    arrow_fisheye_nsl.points[0].z = 0.0f;
    arrow_fisheye_nsl.points[1].x = fisheye_vector_nsl.vector.x*50;
    arrow_fisheye_nsl.points[1].y = fisheye_vector_nsl.vector.y*50;
    arrow_fisheye_nsl.points[1].z = fisheye_vector_nsl.vector.z*50;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    arrow_fisheye_nsl.scale.x = 0.1;
    arrow_fisheye_nsl.scale.y = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    arrow_fisheye_nsl.color.r = 1.0f;
    arrow_fisheye_nsl.color.g = 0.0f;
    arrow_fisheye_nsl.color.b = 0.0f;
    arrow_fisheye_nsl.color.a = 1.0;

    marker_pub.publish(arrow_fisheye_nsl);


    if(msg.angular.x != 0)
    {
      fisheye_vector_nsl2.header.frame_id = "omnicam_nsl";
      fisheye_vector_nsl2.header.stamp = ros::Time(0); 


      // x -> front y->left z-> up (RVIZ)
      fisheye_vector_nsl2.vector.x = msg.angular.z; // Camera frame -> fcu frame
      fisheye_vector_nsl2.vector.y = -msg.angular.x; // Camera frame -> fcu frame
      fisheye_vector_nsl2.vector.z = msg.angular.y; // Camera frame -> fcu frame



      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      arrow_fisheye_nsl2.header.frame_id = "omnicam_nsl";
      arrow_fisheye_nsl2.header.stamp = ros::Time::now();

      arrow_fisheye_nsl2.ns = "arrow_omnicam_nsl2";
      arrow_fisheye_nsl2.id = 4;

      // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      arrow_fisheye_nsl2.type = visualization_msgs::Marker::ARROW;
      // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
      arrow_fisheye_nsl2.action = visualization_msgs::Marker::ADD;

      arrow_fisheye_nsl2.points.resize(2);
      arrow_fisheye_nsl2.points[0].x = 0.0f;
      arrow_fisheye_nsl2.points[0].y = 0.0f;
      arrow_fisheye_nsl2.points[0].z = 0.0f;
      arrow_fisheye_nsl2.points[1].x = fisheye_vector_nsl2.vector.x*50;
      arrow_fisheye_nsl2.points[1].y = fisheye_vector_nsl2.vector.y*50;
      arrow_fisheye_nsl2.points[1].z = fisheye_vector_nsl2.vector.z*50;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      arrow_fisheye_nsl2.scale.x = 0.1;
      arrow_fisheye_nsl2.scale.y = 0.2;

      // Set the color -- be sure to set alpha to something non-zero!
      arrow_fisheye_nsl2.color.r = 1.0f;
      arrow_fisheye_nsl2.color.g = 0.0f;
      arrow_fisheye_nsl2.color.b = 0.0f;
      arrow_fisheye_nsl2.color.a = 1.0;

      marker_pub.publish(arrow_fisheye_nsl2);
      initialized6 = true;
    }
  initialized3 = true;


}

vector<double> multiple_line_intersection(tf::StampedTransform transform, tf::StampedTransform transform2, tf::StampedTransform transform3)
{
 
  Mat I = (Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

  Mat omni_usl = (Mat_<double>(3,1) << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z()); 
  Mat normal_nsl = (Mat_<double>(3,1) << transform2.getOrigin().x(), transform2.getOrigin().y(), transform2.getOrigin().z()); 
  Mat omni_nsl = (Mat_<double>(3,1) << transform3.getOrigin().x(), transform3.getOrigin().y(), transform3.getOrigin().z()); 

  Mat omni_usl_t1 = (Mat_<double>(3,1) << rotated_fisheye_vector_usl.vector.x, rotated_fisheye_vector_usl.vector.y, rotated_fisheye_vector_usl.vector.z); 
  Mat normal_nsl_t1 = (Mat_<double>(3,1) << rotated_normal_vector.vector.x, rotated_normal_vector.vector.y, rotated_normal_vector.vector.z); 
  Mat omni_nsl_t1 = (Mat_<double>(3,1) << rotated_fisheye_vector_nsl.vector.x, rotated_fisheye_vector_nsl.vector.y, rotated_fisheye_vector_nsl.vector.z); 

  Mat omni_usl_n1 = omni_usl_t1 / sqrt(omni_usl_t1.at<double>(0,0)*omni_usl_t1.at<double>(0,0) + omni_usl_t1.at<double>(1,0)*omni_usl_t1.at<double>(1,0) + omni_usl_t1.at<double>(2,0)*omni_usl_t1.at<double>(2,0));
  Mat normal_nsl_n1 = normal_nsl_t1 / sqrt(normal_nsl_t1.at<double>(0,0)*normal_nsl_t1.at<double>(0,0) + normal_nsl_t1.at<double>(1,0)*normal_nsl_t1.at<double>(1,0) + normal_nsl_t1.at<double>(2,0)*normal_nsl_t1.at<double>(2,0));
  Mat omni_nsl_n1 = omni_nsl_t1 / sqrt(omni_nsl_t1.at<double>(0,0)*omni_nsl_t1.at<double>(0,0) + omni_nsl_t1.at<double>(1,0)*omni_nsl_t1.at<double>(1,0) + omni_nsl_t1.at<double>(2,0)*omni_nsl_t1.at<double>(2,0));

  Mat C = (omni_usl_n1*omni_usl_n1.t() - I)*omni_usl + (normal_nsl_n1*normal_nsl_n1.t() - I)*normal_nsl + (omni_nsl_n1*omni_nsl_n1.t() - I)*omni_nsl;
  Mat S = (omni_usl_n1*omni_usl_n1.t() - I) + (normal_nsl_n1*normal_nsl_n1.t() - I) + (omni_nsl_n1*omni_nsl_n1.t() - I);

  Mat inter = S.inv() * C;

  return {inter.at<double>(0,0), inter.at<double>(1,0), inter.at<double>(2,0)};
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter_range");

  ros::NodeHandle nh;
  tf::TransformListener listener;

  ros::Subscriber normal_sub = nh.subscribe("/nslhex/yolo_detection_box", 1, &normal_point_yolo);
  ros::Subscriber normal_sub2 = nh.subscribe("/image_frame_point_normal", 1, &normal_point_tracker);
  ros::Subscriber fisheye_sub = nh.subscribe("/image_frame_point_fisheye", 1, &fisheye_point_usl);
  ros::Subscriber fisheye_sub2 = nh.subscribe("/image_frame_point_fisheye_nsl", 1, &fisheye_point_nsl);
  marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

  myfile.open ("range.txt");

  while(ros::ok())
  {
    ros::spinOnce();

    // All the data should be initially received
    if(initialized1 == false || initialized2 == false || initialized3 == false)  
    {
      cout<<initialized1<<" "<<initialized2<<" "<<initialized3<<endl;
      continue;
    }

    tf::StampedTransform transform, transform2, transform3, transform_threat, transform_t_usl, transform_t_nsl;

    listener.lookupTransform("reference","omnicam_usl", ros::Time(0), transform); // from omnicam frame to gimbal frame
    listener.lookupTransform("reference","gimbal_nsl", ros::Time(0), transform2); // from omnicam frame to gimbal frame
    listener.lookupTransform("reference","omnicam_nsl", ros::Time(0), transform3); // from omnicam frame to gimbal frame
    listener.lookupTransform("reference","threat1", ros::Time(0), transform_threat); // from omnicam frame to gimbal frame
    listener.lookupTransform("uslhex","threat1", ros::Time(0), transform_t_usl); // from omnicam frame to gimbal frame
    listener.lookupTransform("nslhex","threat1", ros::Time(0), transform_t_nsl); // from omnicam frame to gimbal frame
    


    listener.transformVector("reference", ros::Time(0), normal_vector, "gimbal_nsl", rotated_normal_vector);
    if(initialized4) listener.transformVector("reference", ros::Time(0), normal_vector2, "gimbal_nsl", rotated_normal_vector2);
    
    listener.transformVector("reference", ros::Time(0), fisheye_vector_usl, "omnicam_usl", rotated_fisheye_vector_usl);
    if(initialized5) listener.transformVector("reference", ros::Time(0), fisheye_vector_usl2, "omnicam_usl", rotated_fisheye_vector_usl2);

    listener.transformVector("reference", ros::Time(0), fisheye_vector_nsl, "omnicam_nsl", rotated_fisheye_vector_nsl);
    if(initialized6) listener.transformVector("reference", ros::Time(0), fisheye_vector_nsl2, "omnicam_nsl", rotated_fisheye_vector_nsl2);



    Mat fisheye_location = (Mat_<double>(3,1) << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z()); // ENU frame
    Mat normal_location = (Mat_<double>(3,1) << transform2.getOrigin().x(), transform2.getOrigin().y(), transform2.getOrigin().z()); // ENU frame
    Mat fisheye2_location = (Mat_<double>(3,1) << transform3.getOrigin().x(), transform3.getOrigin().y(), transform3.getOrigin().z()); // ENU frame

    Mat baseline_vector = fisheye_location - normal_location;   //ENU frame

    Mat p_l = (Mat_<double>(3,1) << rotated_fisheye_vector_usl.vector.x, rotated_fisheye_vector_usl.vector.y, rotated_fisheye_vector_usl.vector.z); 
    Mat p_r = (Mat_<double>(3,1) << rotated_normal_vector.vector.x, rotated_normal_vector.vector.y, rotated_normal_vector.vector.z); 
    //Mat p_r = (Mat_<double>(3,1) << rotated_fisheye_vector_nsl.vector.x, rotated_fisheye_vector_nsl.vector.y, rotated_fisheye_vector_nsl.vector.z); 


    vector<double> intersection = multiple_line_intersection(transform, transform2, transform3);

    double error3 = sqrt((intersection[0]-transform_threat.getOrigin().x())*(intersection[0]-transform_threat.getOrigin().x()) + (intersection[1]-transform_threat.getOrigin().y())*(intersection[1]-transform_threat.getOrigin().y()) + (intersection[2]-transform_threat.getOrigin().z())*(intersection[2]-transform_threat.getOrigin().z()));

    target_marker.header.frame_id = "reference";
    target_marker.header.stamp = ros::Time::now();

    target_marker.ns = "Target2";
    target_marker.id = 7;
    target_marker.type = visualization_msgs::Marker::SPHERE;
    target_marker.action = visualization_msgs::Marker::ADD;

    target_marker.pose.position.x = intersection[0]; 
    target_marker.pose.position.y = intersection[1];  
    target_marker.pose.position.z = intersection[2]; 
    target_marker.pose.orientation.x = 0.0;
    target_marker.pose.orientation.y = 0.0;
    target_marker.pose.orientation.z = 0.0;
    target_marker.pose.orientation.w = 1.0;

    target_marker.scale.x = 1;
    target_marker.scale.y = 1;
    target_marker.scale.z = 1;

    target_marker.color.r = 0.0f;
    target_marker.color.g = 0.0f;
    target_marker.color.b = 1.0f;
    target_marker.color.a = 1.0;

    marker_pub.publish(target_marker);

    ///////////////Method 1. Cross product method -> Working better
    double x_l = p_l.at<double>(0,0);
    double y_l = p_l.at<double>(0,1);
    double z_l = p_l.at<double>(0,2);
    double x_r = p_r.at<double>(0,0);
    double y_r = p_r.at<double>(0,1);
    double z_r = p_r.at<double>(0,2);
    double x_b = baseline_vector.at<double>(0,0);
    double y_b = baseline_vector.at<double>(0,1);
    double z_b = baseline_vector.at<double>(0,2);


    Mat pr_vb = (Mat_<double>(3,1) << -z_r*y_b+y_r*z_b, z_r*x_b-x_r*z_b, -y_r*x_b+x_r*y_b);
    Mat pr_pl = (Mat_<double>(3,1) << -z_r*y_l+y_r*z_l, z_r*x_l-x_r*z_l, -y_r*x_l+x_r*y_l);

    double prvb=sqrt((-z_r*y_b+y_r*z_b)*(-z_r*y_b+y_r*z_b)+(z_r*x_b-x_r*z_b)*(z_r*x_b-x_r*z_b)+(-y_r*x_b+x_r*y_b)*(-y_r*x_b+x_r*y_b));
    double prpl=sqrt((-z_r*y_l+y_r*z_l)*(-z_r*y_l+y_r*z_l)+(z_r*x_l-x_r*z_l)*(z_r*x_l-x_r*z_l)+(-y_r*x_l+x_r*y_l)*(-y_r*x_l+x_r*y_l));
    
    Mat target_location = prvb/prpl*p_l+fisheye_location;
    double error2 = sqrt((target_location.at<double>(0,0)-transform_threat.getOrigin().x())*(target_location.at<double>(0,0)-transform_threat.getOrigin().x()) + (target_location.at<double>(0,1)-transform_threat.getOrigin().y())*(target_location.at<double>(0,1)-transform_threat.getOrigin().y()) + (target_location.at<double>(0,2)-transform_threat.getOrigin().z())*(target_location.at<double>(0,2)-transform_threat.getOrigin().z()));

    target_marker.header.frame_id = "reference";
    target_marker.header.stamp = ros::Time::now();

    target_marker.ns = "Target";
    target_marker.id = 5;
    target_marker.type = visualization_msgs::Marker::SPHERE;
    target_marker.action = visualization_msgs::Marker::ADD;

    target_marker.pose.position.x = target_location.at<double>(0,0); 
    target_marker.pose.position.y = target_location.at<double>(0,1);  
    target_marker.pose.position.z = target_location.at<double>(0,2); 
    target_marker.pose.orientation.x = 0.0;
    target_marker.pose.orientation.y = 0.0;
    target_marker.pose.orientation.z = 0.0;
    target_marker.pose.orientation.w = 1.0;

    target_marker.scale.x = 1;
    target_marker.scale.y = 1;
    target_marker.scale.z = 1;

    target_marker.color.r = 1.0f;
    target_marker.color.g = 0.0f;
    target_marker.color.b = 0.0f;
    target_marker.color.a = 1.0;

    marker_pub.publish(target_marker);

    double distance_nsl = sqrt(transform_t_nsl.getOrigin().x()*transform_t_nsl.getOrigin().x() + transform_t_nsl.getOrigin().y()*transform_t_nsl.getOrigin().y() + transform_t_nsl.getOrigin().z()*transform_t_nsl.getOrigin().z());
    double distance_usl = sqrt(transform_t_usl.getOrigin().x()*transform_t_usl.getOrigin().x() + transform_t_usl.getOrigin().y()*transform_t_usl.getOrigin().y() + transform_t_usl.getOrigin().z()*transform_t_usl.getOrigin().z());

    cout<<"Distance from nsl and usl: "<<distance_nsl<<" "<<distance_usl<<endl;
    cout<<"Error using 2 and 3: "<<error2<<" "<<error3<<endl;


  }

  myfile.close();
  return 0;
}
