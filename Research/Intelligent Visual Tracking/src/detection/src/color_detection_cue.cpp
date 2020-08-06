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
#include "ocam_functions.h"
#include <unordered_set> 

using namespace cv;
using namespace std;

Mat src; Mat src_gray, HSV;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);

// Type of the color (lower -> red, orange, yellow, green, blue .. -> higher)
int H_MIN = 0;
int H_MAX = 10;
// Lower -> more grey, Higher -> more colored
int S_MIN = 70;
int S_MAX = 255;
// Brightness lower -> dark, higher -> white
int V_MIN = 50;
int V_MAX = 255;

//default capture width and height
const int FRAME_WIDTH = 1280;
const int FRAME_HEIGHT = 960;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 40*40;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
vector<vector<int>> target_list;
vector<vector<double>> target_list_vector;


struct ocam_model o; // our ocam_models for the fisheye and catadioptric cameras   
int target_same_thres = 50;

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

        Mat src_resize, src_resize2;
        src = cv_ptr->image;
        src = src(Range::all(), Range(0, 1504));
        resize(src, src, Size(1440,1440));
        flip(src,src,-1);

        // obtain the circle image ROI:
        Mat roi(src);
        Mat mask(roi.size(), roi.type(), Scalar::all(0));
        circle(mask, Point(mask.cols/2,mask.rows/2), mask.rows/2-120, Scalar::all(255), -1);
        src = roi & mask;

        //createTrackbar( " Threshold:", "Source", &thresh, max_thresh, thresh_callback );
        thresh_callback_back( 0, 0 );

        /// Create Window
        resize(src,src_resize,Size(500,500));
        imshow( "source_window", src_resize );

        src = cv_ptr->image;
        src = src(Range::all(), Range(1504, 3008));
        resize(src, src, Size(1440,1440));
        flip(src,src,-1);

        // obtain the circle image ROI:
        Mat roi2(src);
        Mat mask2(roi2.size(), roi2.type(), Scalar::all(0));
        circle(mask2, Point(mask2.cols/2,mask2.rows/2), mask2.rows/2-120, Scalar::all(255), -1);
        src = roi2 & mask2;

        //createTrackbar( " Threshold:", "Source", &thresh, max_thresh, thresh_callback );
        thresh_callback_front( 0, 0 );


        /// Create Window
        resize(src,src_resize2,Size(500,500));
        imshow( "source_window2", src_resize2 );


        // Put the size of the detections at the first array of the message.
        geometry_msgs::Point32 pt_tmp;
        geometry_msgs::PolygonStamped msg2;
        msg2.polygon.points.reserve(10); // ensure that there will be space for at least 10 points
        pt_tmp.x = target_list_vector.size();
        msg2.polygon.points.push_back(pt_tmp);
        cout<<target_list_vector.size()<<endl;

        for(int i=0; i<target_list_vector.size(); i++)
        {
            pt_tmp.x = target_list_vector[i][0];
            pt_tmp.y = target_list_vector[i][1];
            pt_tmp.z = target_list_vector[i][2];
            msg2.polygon.points.push_back(pt_tmp);
        }

        gimbal_cue.publish(msg2);

        int key1 = waitKey(20);


    }

    void thresh_callback_back(int, void* )
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
        resize(threshold2, threshold2, Size(500,500));
        //imshow("threshold",threshold2);

        /// Approximate contours to polygons + get bounding rects and circles
        vector<vector<Point> > contours_poly( contours.size() );
        vector<Rect> boundRect( contours.size() );                      
        vector<Point2f>center( contours.size() );
        vector<float>radius( contours.size() );
        double area, max_area=0;
        double max_area_num=-2;

        // Undistort the detections and publish.
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

            if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>-1)
            {

                Point center_of_rect;
                Rect boundRect2;
                boundRect2 = boundingRect( Mat(contours_poly[i]) );
                //minEnclosingCircle( (Mat)contours_poly[i], center[i],     [i] );
                center_of_rect = (boundRect2.br() + boundRect2.tl())*0.5;
                rectangle( src, boundRect2.tl(), boundRect2.br(), Scalar( 255,0,0 ), 8, 8, 0 ); 
                // cout<<center_of_rect<<endl;
                // Initiate the target_list vector.
                if(target_list.empty())
                {
                    vector<int> tmp_vector = {center_of_rect.x, center_of_rect.y};
                    target_list.push_back(tmp_vector);

                    // Undistort image point to a 3d vector
                    double point3D[3], point2D[2]; // the image point in pixel coordinates  
                    point2D[0] = center_of_rect.x;
                    point2D[1] = center_of_rect.y;
                    cam2world(point3D, point2D, &o);
                    // Change the vector direction in terms of the camera direction
                    vector<double> tmp_vector2 = {-point3D[0], -point3D[1], point3D[2]};
                    target_list_vector.push_back(tmp_vector2);
                }
                else
                {              
                    unordered_set<double> distance_list;
                    bool already_in = false;

                    // Compute the distance between target points already detected and points just detected
                    for(int j=0; j<target_list.size(); j++)
                    {
                        double distance = sqrt((target_list[j][0]-center_of_rect.x)*(target_list[j][0]-center_of_rect.x) + (target_list[j][1]-center_of_rect.y)*(target_list[j][1]-center_of_rect.y));
                        distance_list.insert(distance);
                    }

                    // Check if the new detected points already detected before.
                    for(double x:distance_list)
                    {
                        if(x<target_same_thres)
                        {
                            already_in = true;
                        }
                    }

                    // If not, add the new points in the set.
                    if(!already_in)
                    {
                        vector<int> tmp_vector = {center_of_rect.x, center_of_rect.y};
                        target_list.push_back(tmp_vector);
                        
                        // Undistort image point to a 3d vector
                        double point3D[3], point2D[2]; // the image point in pixel coordinates  
                        point2D[0] = center_of_rect.x;
                        point2D[1] = center_of_rect.y;
                        cam2world(point3D, point2D, &o);

                        // Change the vector direction in terms of the camera direction
                        vector<double> tmp_vector2 = {-point3D[0], -point3D[1], point3D[2]};
                        target_list_vector.push_back(tmp_vector2);
                    }

                }


            }

        }

        /// Draw polygonal contour + bonding rects + circles
        for( int i = 0; i< contours.size(); i++ )
        {
        //drawContours( src, contours_poly, i, color, 5, 8, vector<Vec4i>(), 0, Point() );
        //circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
        } 



        // /// Show in a window
        // namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
        // resize(drawing, drawing, Size(500,500));

        // imshow( "Contours", drawing );

    }

    void thresh_callback_front(int, void* )
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
        resize(threshold2, threshold2, Size(500,500));
        //imshow("threshold2",threshold2);

        /// Approximate contours to polygons + get bounding rects and circles
        vector<vector<Point> > contours_poly( contours.size() );
        vector<Rect> boundRect( contours.size() );                      
        vector<Point2f>center( contours.size() );
        vector<float>radius( contours.size() );
        double area, max_area=0;
        double max_area_num=-2;

        // Undistort the detections and publish.
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

            if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>-1)
            {

                Point center_of_rect;
                Rect boundRect2;
                boundRect2 = boundingRect( Mat(contours_poly[i]) );
                //minEnclosingCircle( (Mat)contours_poly[i], center[i],     [i] );
                center_of_rect = (boundRect2.br() + boundRect2.tl())*0.5;
                rectangle( src, boundRect2.tl(), boundRect2.br(), Scalar( 255,0,0 ), 8, 8, 0 ); 
                // cout<<center_of_rect<<endl;
                // Initiate the target_list vector.
                if(target_list.empty())
                {
                    vector<int> tmp_vector = {center_of_rect.x, center_of_rect.y};
                    target_list.push_back(tmp_vector);

                    // Undistort image point to a 3d vector
                    double point3D[3], point2D[2]; // the image point in pixel coordinates  
                    point2D[0] = center_of_rect.x;
                    point2D[1] = center_of_rect.y;
                    cam2world(point3D, point2D, &o);

                    vector<double> tmp_vector2 = {point3D[0], -point3D[1], -point3D[2]};
                    target_list_vector.push_back(tmp_vector2);
                }
                else
                {              
                    unordered_set<double> distance_list;
                    bool already_in = false;

                    // Compute the distance between target points already detected and points just detected
                    for(int j=0; j<target_list.size(); j++)
                    {
                        double distance = sqrt((target_list[j][0]-center_of_rect.x)*(target_list[j][0]-center_of_rect.x) + (target_list[j][1]-center_of_rect.y)*(target_list[j][1]-center_of_rect.y));
                        distance_list.insert(distance);
                    }

                    // Check if the new detected points already detected before.
                    for(double x:distance_list)
                    {
                        if(x<target_same_thres)
                        {
                            already_in = true;
                        }
                    }

                    // If not, add the new points in the set.
                    if(!already_in)
                    {
                        vector<int> tmp_vector = {center_of_rect.x, center_of_rect.y};
                        target_list.push_back(tmp_vector);

                        // Undistort image point to a 3d vector
                        double point3D[3], point2D[2]; // the image point in pixel coordinates  
                        point2D[0] = center_of_rect.x;
                        point2D[1] = center_of_rect.y;
                        cam2world(point3D, point2D, &o);

                        vector<double> tmp_vector2 = {point3D[0], -point3D[1], -point3D[2]};
                        target_list_vector.push_back(tmp_vector2);
                    }

                }


            }

        }

        /// Draw polygonal contour + bonding rects + circles
        for( int i = 0; i< contours.size(); i++ )
        {
        //drawContours( src, contours_poly, i, color, 5, 8, vector<Vec4i>(), 0, Point() );
        //circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
        } 



        /// Show in a window
        //namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
        //resize(drawing, drawing, Size(500,500));

        //imshow( "Contours2", drawing );

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
            image_sub_ = it_.subscribe("/omnicam/image_raw", 1, &Color_detection::imageCb, this,hints);
            gimbal_cue = nh_.advertise<geometry_msgs::PolygonStamped>("gimbal_cue_periph",100);

        }
        ~Color_detection()
        {

        }


};

int main(int argc, char** argv){

	ros::init(argc,argv,"color_omnicam");
    get_ocam_model(&o, "/home/nvidia/visual_tracker_2.0/src/detection/src/Insta360_calib_results.txt");

 	Color_detection name;
	ros::NodeHandle nh("/");
	ros::spin();
	return 0;
}
