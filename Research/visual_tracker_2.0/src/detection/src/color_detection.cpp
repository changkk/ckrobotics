#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Float64.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
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
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PolygonStamped.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

Mat src; Mat src_gray, HSV;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);
int H_MIN = 0;
int H_MAX = 10;
int S_MIN = 70;
int S_MAX = 255;
int V_MIN = 50;
int V_MAX = 255;
//default capture width and height
const int FRAME_WIDTH = 1280;
const int FRAME_HEIGHT = 960;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 50*50;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;



class Color_detection
{

    public: 

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


        src = cv_ptr->image;

        /// Create Window
        imshow( "source_window", src );

        //createTrackbar( " Threshold:", "Source", &thresh, max_thresh, thresh_callback );
        thresh_callback( 0, 0 );

        int key1 = waitKey(20);


    }

    void thresh_callback(int, void* )
    {
        Mat threshold_output;
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        Mat threshold2;

	    cvtColor(src,HSV,COLOR_BGR2HSV);
    	inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold2);
        Mat erodeElement = getStructuringElement( MORPH_RECT,Size(10,10));
        Mat dilateElement = getStructuringElement( MORPH_RECT,Size(10,10));
        erode(threshold2,threshold2,erodeElement);
        erode(threshold2,threshold2,erodeElement);
        dilate(threshold2,threshold2,dilateElement);
        dilate(threshold2,threshold2,dilateElement);

        /// Find contours
        findContours( threshold2, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
        Mat drawing = Mat::zeros( threshold2.size(), CV_8UC3 );
        Scalar color = Scalar( 0,255,255 );
        imshow("threshold",threshold2);

        /// Approximate contours to polygons + get bounding rects and circles
        vector<vector<Point> > contours_poly( contours.size() );
        vector<Rect> boundRect( contours.size() );
        vector<Point2f>center( contours.size() );
        vector<float>radius( contours.size() );
        double area, max_area=0;
        double max_area_num=-2;


            for( int i = 0; i < contours.size(); i++ )
            { 
                Moments moment = moments((cv::Mat)contours[i]);
                area = moment.m00;
                approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
                if(area>max_area)
                {
                    max_area = area;
                    max_area_num = i;
                }

            }

            if(max_area>MIN_OBJECT_AREA && max_area<MAX_OBJECT_AREA && max_area_num>-1)
            {
                Rect boundRect2 = boundingRect( Mat(contours_poly[max_area_num]) );
                //minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
                Point center_of_rect = (boundRect2.br() + boundRect2.tl())*0.5;
                rectangle( drawing, boundRect2.tl(), boundRect2.br(), Scalar( 255,255,0 ), 2, 8, 0 ); 

                geometry_msgs::Point32 pt_tmp;
                geometry_msgs::PolygonStamped msg;

                msg.polygon.points.reserve(2); // ensure that there will be space for at least 10 points

                pt_tmp.x = center_of_rect.x;
                pt_tmp.y = center_of_rect.y;
                msg.polygon.points.push_back(pt_tmp);

                pt_tmp.x = src.cols;
                pt_tmp.y = src.rows;
                msg.polygon.points.push_back(pt_tmp);

                pt_tmp.x = boundRect2.width;
                pt_tmp.y = boundRect2.height;
                msg.polygon.points.push_back(pt_tmp);

                gimbal_cue.publish(msg);

            }

                    

            /// Draw polygonal contour + bonding rects + circles
            for( int i = 0; i< contours.size(); i++ )
            {
            drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
            //circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
            } 



        /// Show in a window
        //namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
        imshow( "Contours", drawing );

    }

    private:

        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
         ros::Publisher gimbal_cue;


    public:
        Color_detection()
        : it_(nh_)
        {
            image_transport::TransportHints hints("compressed", ros::TransportHints());
            image_sub_ = it_.subscribe("/camera/image_raw", 1, &Color_detection::imageCb, this,hints);
            gimbal_cue = nh_.advertise<geometry_msgs::PolygonStamped>("yolo_detection_box",100);

        }
        ~Color_detection()
        {

        }


};

int main(int argc, char** argv){

	ros::init(argc,argv,"color");
 	Color_detection name;
	ros::NodeHandle nh("/");
	ros::spin();
	return 0;
}
