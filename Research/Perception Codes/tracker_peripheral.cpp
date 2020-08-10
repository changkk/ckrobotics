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
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <sstream>
#include <string>
#include <nav_msgs/Odometry.h>
#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <sstream>      
#include "sensor_msgs/Imu.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include "ocam_functions.h"
#include <opencv2/core/core.hpp>

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

static const std::string OPENCV_WINDOW = "Image window";
using namespace cv;
using std::vector;
using namespace std;

int target_x, target_y;
bool front_lens = true;
struct ocam_model o; // our ocam_models for the fisheye and catadioptric cameras   

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

public: 



private:

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

public:
  ImageConverter()
  : it_(nh_)
  {
    image_transport::TransportHints hints("compressed", ros::TransportHints());
    image_sub_ = it_.subscribe("/uslhex/omnicam/image_raw", 1, &ImageConverter::imageCb, this,hints);
    target_vector_pub = nh_.advertise<geometry_msgs::Twist>("/image_frame_point_fisheye", 10);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
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

    if(front_lens)
        img = src(Range::all(), Range(1504, 3008));
    else
        img = src(Range::all(), Range(0, 1504));        

    
    resize(img, img, Size(1440,1440), 0);
    flip(img,img,-1);
    ok = tracker->update(img, target_bbox);
    ok2 = tracker2->update(img, target_bbox2);
    geometry_msgs::Twist vector_msg;

    if (ok)
    {
        // Tracking success : Draw the tracked object
        rectangle(img, target_bbox, Scalar( 255, 0, 0 ), 2, 1 );
        // Undistort image point to a 3d vector
        double point3D[3], point2D[2]; // the image point in pixel coordinates  
        point2D[0] = target_bbox.x + target_bbox.width/2;
        point2D[1] = target_bbox.y + target_bbox.height/2;
        circle(img, Point(point2D[0], point2D[1]), 1, Scalar( 255, 0, 0 ), 2, 8, 0 );


        // OPENCV function - undistort points
        Mat_<Point2f> points(1,1);
        points(0) = Point2f(point2D[0],point2D[1]);
        vector<Point2f> dst;// leave empty, opencv will fill it.
        undistortPoints(points, dst, K, D); 
        // For range estimation
        // For USL hex, omnicam is mounted in the down direction.
        if(front_lens)
        {
            vector_msg.linear.x=dst[0].x; // x -> right
            vector_msg.linear.y=-dst[0].y; // y -> up
            vector_msg.linear.z=1; // z -> front
        }
        else
        {
            vector_msg.linear.x=-dst[0].x;
            vector_msg.linear.y=dst[0].y;
            vector_msg.linear.z=-1;
        }


        // // OCAMCALIB - UNDISTORT POINTS
        // // For range estimation
        // geometry_msgs::Twist vector_msg;
        // cam2world(point3D, point2D, &o);
        // // For USL hex, omnicam is mounted in the down direction.
        // if(front_lens)
        // {
        //     vector_msg.linear.x=point3D[0]; // x -> right
        //     vector_msg.linear.y=-point3D[1]; // y -> up
        //     vector_msg.linear.z=-point3D[2]; // z -> front
        // }
        // else
        // {
        //     vector_msg.linear.x=-point3D[0];
        //     vector_msg.linear.y=point3D[2];
        //     vector_msg.linear.z=-point3D[1];
        // }

    }
    else
        putText(img, "Tracking failure detected", Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);

    if (ok2)
    {
        // Tracking success : Draw the tracked object
        rectangle(img, target_bbox2, Scalar( 255, 0, 0 ), 2, 1 );
        // Undistort image point to a 3d vector
        double point3D[3], point2D[2]; // the image point in pixel coordinates  
        point2D[0] = target_bbox2.x + target_bbox2.width/2;
        point2D[1] = target_bbox2.y + target_bbox2.height/2;
        circle(img, Point(point2D[0], point2D[1]), 1, Scalar( 255, 0, 0 ), 2, 8, 0 );

        // OPENCV function - undistort points
        Mat_<Point2f> points(1,1);
        points(0) = Point2f(point2D[0],point2D[1]);
        vector<Point2f> dst;// leave empty, opencv will fill it.
        undistortPoints(points, dst, K, D); 
        // For range estimation
        // For USL hex, omnicam is mounted in the down direction.
        if(front_lens)
        {
            vector_msg.angular.x=dst[0].x; // x -> right
            vector_msg.angular.y=-dst[0].y; // y -> up
            vector_msg.angular.z=1; // z -> front
        }
        else
        {
            vector_msg.angular.x=-dst[0].x;
            vector_msg.angular.y=dst[0].y;
            vector_msg.angular.z=-1;
        }


        // // OCAMCALIB - UNDISTORT POINTS
        // // For range estimation
        // geometry_msgs::Twist vector_msg;
        // cam2world(point3D, point2D, &o);
        // // For USL hex, omnicam is mounted in the down direction.
        // if(front_lens)
        // {
        //     vector_msg.linear.x=point3D[0]; // x -> right
        //     vector_msg.linear.y=-point3D[1]; // y -> up
        //     vector_msg.linear.z=-point3D[2]; // z -> front
        // }
        // else
        // {
        //     vector_msg.linear.x=-point3D[0];
        //     vector_msg.linear.y=point3D[2];
        //     vector_msg.linear.z=-point3D[1];
        // }

    }
    else
        putText(img, "Tracking failure detected", Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);


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
      
      if(target_bbox.x == 0)
      {
        Rect2d tmp(x-10, y-10, 20, 20);
        cout<<"New target is added: "<<x<<" "<<y<<endl;
        tracker->clear();
        tracker = TrackerMIL::create();
        tracker->init(img, tmp);
      }
      else if(target_bbox2.x == 0)
      {
        Rect2d tmp(x-10, y-10, 20, 20);
        cout<<"New target is added: "<<x<<" "<<y<<endl;
        tracker2->clear();
        tracker2 = TrackerMIL::create();
        tracker2->init(img, tmp);
      }
      else
      {
        double distance = sqrt((x-target_bbox.x)*(x-target_bbox.x)+(y-target_bbox.y)*(y-target_bbox.y));
        double distance2 = sqrt((x-target_bbox2.x)*(x-target_bbox2.x)+(y-target_bbox2.y)*(y-target_bbox2.y));

        if(distance < 20)
        {
          target_x = x;
          target_y = y;
          Rect2d tmp(x-10, y-10, 20, 20);
          cout<<target_x<<" "<<target_y<<endl;
          tracker->clear();
          tracker = TrackerMIL::create();
          tracker->init(img, tmp);
        }
        else if(distance2 < 20)
        {
          target_x = x;
          target_y = y;
          Rect2d tmp(x-10, y-10, 20, 20);
          cout<<target_x<<" "<<target_y<<endl;
          tracker2->clear();
          tracker2 = TrackerMIL::create();
          tracker2->init(img, tmp);
        }
        else
          cout<<"More than two targets cannot be added"<<endl;

      }




    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracker_fisheye_usl");
    get_ocam_model(&o, "/home/changkoo/catkin_ws/src/perception/src/Insta360_calib_results.txt");
    ImageConverter ic;
    namedWindow("orig");
    cv::setMouseCallback("orig", onMouse, 0 );

    while(ros::ok())
    {
        ros::spinOnce();

    }

    return 0;
}
