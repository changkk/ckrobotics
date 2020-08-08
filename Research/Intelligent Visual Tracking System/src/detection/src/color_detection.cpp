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

// Type of the color (red, blue ..)
int H_MIN = 0;
int H_MAX = 12;
// Lower -> more grey, Higher -> more colored
int S_MIN = 70;
int S_MAX = 255;
// Brightness lower -> dark, higher -> white
int V_MIN = 0;
int V_MAX = 255;

//default capture width and height
const int FRAME_WIDTH = 1280;
const int FRAME_HEIGHT = 960;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 80*80;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
Rect boundRect2;
Point center_of_rect;

// >>>> Kalman Filter
int stateSize = 6;
int measSize = 4;
int contrSize = 0;
unsigned int type = CV_32F;
double dT;
cv::KalmanFilter kf(stateSize, measSize, contrSize, type);
double ticks;
bool found;
int notFoundCount;
cv::Mat state;
cv::Mat meas;

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

        unsigned int type = CV_32F;
        state = cv::Mat(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
        meas = cv::Mat(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
        //cv::Mat procNoise(stateSize, 1, type)
        // [E_x,E_y,E_v_x,E_v_y,E_w,E_h]

        // Transition State Matrix A
        // Note: set dT at each processing step!
        // [ 1 0 dT 0  0 0 ]
        // [ 0 1 0  dT 0 0 ]
        // [ 0 0 1  0  0 0 ]
        // [ 0 0 0  1  0 0 ]
        // [ 0 0 0  0  1 0 ]
        // [ 0 0 0  0  0 1 ]
        cv::setIdentity(kf.transitionMatrix);

        // Measure Matrix H
        // [ 1 0 0 0 0 0 ]
        // [ 0 1 0 0 0 0 ]
        // [ 0 0 0 0 1 0 ]
        // [ 0 0 0 0 0 1 ]
        kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
        kf.measurementMatrix.at<float>(0) = 1.0f;
        kf.measurementMatrix.at<float>(7) = 1.0f;
        kf.measurementMatrix.at<float>(16) = 1.0f;
        kf.measurementMatrix.at<float>(23) = 1.0f;

        // Process Noise Covariance Matrix Q
        // [ Ex   0   0     0     0    0  ]
        // [ 0    Ey  0     0     0    0  ]
        // [ 0    0   Ev_x  0     0    0  ]
        // [ 0    0   0     Ev_y  0    0  ]
        // [ 0    0   0     0     Ew   0  ]
        // [ 0    0   0     0     0    Eh ]
        //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
        kf.processNoiseCov.at<float>(0) = 1e-2;
        kf.processNoiseCov.at<float>(7) = 1e-2;
        kf.processNoiseCov.at<float>(14) = 5.0f;
        kf.processNoiseCov.at<float>(21) = 5.0f;
        kf.processNoiseCov.at<float>(28) = 1e-2;
        kf.processNoiseCov.at<float>(35) = 1e-2;

        // Measures Noise Covariance Matrix R
        cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
        // <<<< Kalman Filter
        ticks = 0;
        found = false;


        src = cv_ptr->image;

        double precTick = ticks;
        ticks = (double) cv::getTickCount();

        dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

        //createTrackbar( " Threshold:", "Source", &thresh, max_thresh, thresh_callback );
        thresh_callback( 0, 0 );

        Mat src_resize;
        /// Create Window
        resize(src,src_resize,src.size()/3);
        imshow( "source_window3", src_resize );


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
        Scalar color = Scalar( 0,0,255 );

        // resize(threshold2, threshold2, threshold2.size()/3);
        // imshow("threshold3",threshold2);

        /// Approximate contours to polygons + get bounding rects and circles
        vector<vector<Point> > contours_poly( contours.size() );
        vector<Rect> boundRect( contours.size() );
        vector<Point2f>center( contours.size() );
        vector<float>radius( contours.size() );
        double area, max_area=0;
        double max_area_num=-2;

        if (found)
        {
            // >>>> Matrix A
            kf.transitionMatrix.at<float>(2) = dT;
            kf.transitionMatrix.at<float>(9) = dT;
            // <<<< Matrix A
            state = kf.predict();

            cv::Rect predRect;
            predRect.width = state.at<float>(4);
            predRect.height = state.at<float>(5);
            predRect.x = state.at<float>(0) - predRect.width / 2;
            predRect.y = state.at<float>(1) - predRect.height / 2;

            cv::Point center;
            center.x = state.at<float>(0);
            center.y = state.at<float>(1);

            rectangle( drawing, predRect.tl(), predRect.br(), color, 2, 8, 0 ); 

        }

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

            if(max_area>MIN_OBJECT_AREA && max_area<MAX_OBJECT_AREA && max_area_num>-1)
            {
                boundRect2 = boundingRect( Mat(contours_poly[max_area_num]) );
                //minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
                center_of_rect = (boundRect2.br() + boundRect2.tl())*0.5;
                rectangle( src, boundRect2.tl(), boundRect2.br(), Scalar( 255,0,0 ), 8, 8, 0 ); 

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

        }       
                
    // Draw polygonal contour + bonding rects + circles
    for( int i = 0; i< contours.size(); i++ )
        {
        drawContours( src, contours_poly, i, color, 5, 8, vector<Vec4i>(), 0, Point() );
        //circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
        } 



    //     /// Show in a window
    //     namedWindow( "Contours3", CV_WINDOW_AUTOSIZE );
    //     resize(drawing,drawing,drawing.size()/3);
    //     imshow( "Contours3", drawing );
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
            gimbal_cue = nh_.advertise<geometry_msgs::PolygonStamped>("/yolo_detection_box",100);

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
