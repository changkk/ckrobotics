#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <vector>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <ros/package.h>
#include <sstream>
#include <string>
#include <nav_msgs/Odometry.h>
#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"
#include <sstream>      
#include "sensor_msgs/Imu.h"
#include <geometry_msgs/TransformStamped.h>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/core.hpp>

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

static const std::string OPENCV_WINDOW = "Image window";
using namespace cv;
using std::vector;
using namespace std;

int target_x, target_y;

ros::Publisher target_vector_pub;

Mat K = (Mat_<double>(3, 3) <<  445.17984003885283, 0.0, 721.2172155119429, 0.0, 445.8816513187479, 721.9473150432297, 0.0, 0.0, 1.0);
Mat D = (Mat_<double>(1, 4) << -0.017496068223756347, -0.003243014339251435, 0.000839223410670477, -0.0004219234265229322);

// List of tracker types in OpenCV 3.4.1
string trackerTypes[8] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};

// Create a tracker
string trackerType = trackerTypes[1];

Ptr<Tracker> tracker = TrackerMIL::create();
Ptr<Tracker> tracker2 = TrackerMIL::create();

bool ok, ok2;
Rect2d target_bbox, target_bbox2;
Mat img;


class ImageConverter
{

private:

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber normal_sub;

public:
  ImageConverter()
  : it_(nh_)
  {
    image_transport::TransportHints hints("compressed", ros::TransportHints());
    image_sub_ = it_.subscribe("/nslhex/camera/image_raw", 1, &ImageConverter::imageCb, this,hints);
    target_vector_pub = nh_.advertise<geometry_msgs::Twist>("/image_frame_point_normal", 10);
    normal_sub = nh_.subscribe("/nslhex/yolo_detection_box", 1000, &ImageConverter::yolo_CB, this);

  }

  ~ImageConverter()
  {
  }

    void yolo_CB(const geometry_msgs::PolygonStamped& msg)	
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
            // if(target_bbox.x == 0)
            // {
            //     Rect2d tmp(normal_x-BBS_x/2, normal_x-BBS_y/2, BBS_x, BBS_y);
            //     cout<<"New target is added: "<<normal_x<<" "<<normal_y<<endl;
            //     tracker->clear();
            //     tracker = TrackerMIL::create();
            //     tracker->init(img, tmp);
            // }
            // else if(target_bbox2.x == 0)
            // {
            //     Rect2d tmp(normal_x-BBS_x/2, normal_x-BBS_y/2, BBS_x, BBS_y);
            //     cout<<"New target is added: "<<normal_x<<" "<<normal_y<<endl;
            //     tracker2->clear();
            //     tracker2 = TrackerMIL::create();
            //     tracker2->init(img, tmp);
            // }
            // else
            // {
            //     double distance = sqrt((normal_x-target_bbox.x)*(normal_x-target_bbox.x)+(normal_y-target_bbox.y)*(normal_y-target_bbox.y));
            //     double distance2 = sqrt((normal_x-target_bbox2.x)*(normal_x-target_bbox2.x)+(normal_y-target_bbox2.y)*(normal_y-target_bbox2.y));

            //     if(distance < 20)
            //     {
            //     cout<<"Target1 is updated: "<<normal_x<<" "<<normal_y<<endl;
            //     Rect2d tmp(normal_x-BBS_x/2, normal_x-BBS_y/2, BBS_x, BBS_y);
            //     tracker->clear();
            //     tracker = TrackerMIL::create();
            //     tracker->init(img, tmp);
            //     }
            //     else if(distance2 < 20)
            //     {
            //     Rect2d tmp(normal_x-BBS_x/2, normal_x-BBS_y/2, BBS_x, BBS_y);
            //     cout<<"Target2 is updated: "<<normal_x<<" "<<normal_y<<endl;
            //     tracker2->clear();
            //     tracker2 = TrackerMIL::create();
            //     tracker2->init(img, tmp);
            //     }
            //     else
            //     cout<<"More than two targets cannot be added"<<endl;

            // }

        }

    }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    Mat src = cv_ptr->image;
    src.copyTo(img);
    ok = tracker->update(img, target_bbox);
    geometry_msgs::Twist vector_msg;

    if (ok)
    {
        // Tracking success : Draw the tracked object
        rectangle(img, target_bbox, Scalar( 255, 0, 0 ), 2, 1 );
        vector_msg.linear.x=target_bbox.x + target_bbox.width/2; // horzontal px (center)
        vector_msg.linear.y=-target_bbox.y + target_bbox.height/2; // vertical px (center)
        vector_msg.linear.z=0; // nothing

    }

    if(vector_msg.linear.x != 0)
      target_vector_pub.publish(vector_msg);

    imshow("orig",img);
    int key1 = waitKey(20);

}



};

static void onMouse( int event, int x, int y, int, void* )
{
    static int count(0);
    if ( event == cv::EVENT_LBUTTONDOWN) {
      
      Rect2d tmp(x-10, y-10, 20, 20);
      cout<<"New target is added: "<<x<<" "<<y<<endl;
      tracker->clear();
      tracker = TrackerMIL::create();
      tracker->init(img, tmp);

    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracker_normal");
    ImageConverter ic;
    namedWindow("orig");
    cv::setMouseCallback("orig", onMouse, 0 );

    while(ros::ok())
    {
        ros::spinOnce();

    }

    return 0;
}
