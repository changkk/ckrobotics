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

double accuracy_threshold=0.000005;
int target_x, target_y;
bool front_lens = true;
struct ocam_model o; // our ocam_models for the fisheye and catadioptric cameras   

ros::Publisher target_vector_pub;

std::string target1=ros::package::getPath("perception")+"/src/temp1.png";
std::string target2=ros::package::getPath("perception")+"/src/temp2.png";
std::string target3=ros::package::getPath("perception")+"/src/temp3.png";
// std::string target4=ros::package::getPath("perception")+"/src/template.png";
// std::string target5=ros::package::getPath("perception")+"/src/template.png";
Mat K = (Mat_<double>(3, 3) <<  445.17984003885283, 0.0, 721.2172155119429, 0.0, 445.8816513187479, 721.9473150432297, 0.0, 0.0, 1.0);
Mat D = (Mat_<double>(1, 4) << -0.017496068223756347, -0.003243014339251435, 0.000839223410670477, -0.0004219234265229322);

// List of tracker types in OpenCV 3.4.1
string trackerTypes[8] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};
// vector <string> trackerTypes(types, std::end(types));

// Create a tracker
string trackerType = trackerTypes[1];

Ptr<Tracker> tracker = TrackerMIL::create();
bool initial_track = true;

int k;
Point point1, point2; /* vertical points of the bounding box */
int drag = 0;
Rect rect; /* bounding box */
Mat img, roiImg; /* roiImg - the part of the image in the bounding box */
int select_flag = 0;
bool go_fast = false;
Mat mytemplate1, mytemplate2, mytemplate3,mytemplate4,mytemplate5,mytemplate6, mytemplate7,mytemplate8,mytemplate9;
Mat mytemplate_array[10];
double target_array[10];
int target_detection;
Point target_point, target;
int template_number = 1;

double roll,pitch,yaw;
double zoom;
int zoom_compute;
bool ok;
Rect2d tmp2;


int maximum_length =100;
double minimum_length =2;
int find_number = 200;
int target_determination = 9;

double pi=3.141592;
double focal_length = 17.2*0.001;
double ccd_x=2.3*0.001;
double ccd_y=1.6*0.001;

Point prepoint1,prepoint2,prepoint3,prepoint4,prepoint5,prepoint6,prepoint7;
int minimum_distance = 10;
double average;


class ImageConverter
{

public: 

void chatterCallback(const geometry_msgs::PoseStamped& msg)	
{
        	geometry_msgs::Point coord = msg.pose.position;
		x = coord.x;
		y = coord.y;
		z = coord.z;
	
}


private:
	double x, y, z;
	double g_x;
	double g_y;

Mat T_mxkmGkm = (Mat_<double>(2,1) << 0, 0);
Mat T_SxkmGkm = (Mat_<double>(2,2) << 5, 0, 0, 5);
	double detected_time=0;
	double fps_check=0;
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  //image_transport::Publisher image_pub_;
  //image_transport::Publisher image_pub_2;

  ros::Subscriber pose_sub;
  ros::Publisher p_pub;
  ros::Publisher s_pub;
  ros::Publisher i_pub;
  ros::Subscriber imu_sub;

public:
  ImageConverter()
  : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
        // image_transport::TransportHints hints("compressed", ros::TransportHints());
    image_transport::TransportHints hints("compressed", ros::TransportHints());

    image_sub_ = it_.subscribe("/uslhex/omnicam/image_raw", 1,
    &ImageConverter::imageCb, this,hints);
    //image_pub_ = it_.advertise("/normal_images", 1);
    //image_pub_2 = it_.advertise("/template_crop", 1);

    target_vector_pub = nh_.advertise<geometry_msgs::Twist>("/image_frame_point_fisheye", 10);
    pose_sub = nh_.subscribe("/hexacopter/mavros/local_position/pose", 1000, &ImageConverter::chatterCallback, this);
    p_pub = nh_.advertise<geometry_msgs::Twist>("/rbe_target_point", 10);
    s_pub = nh_.advertise<geometry_msgs::Twist>("/observation_global_point", 10);
    i_pub = nh_.advertise<geometry_msgs::Twist>("/image_frame_point", 10);


    cv::namedWindow(OPENCV_WINDOW);
    x = 0;
    y = 0;
    z = 0;
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


	//fps_check=fps_check+0.195;
	//if(fps_check>20)
	//{
	//	std::cout<<"fps_check: "<<fps_check<<endl;
	//}

    bool trackObjects = true;
    bool useMorphOps = true;
	average=100;

	Mat HSV;
	Mat threshold;
	int x=0, y=0;


    cv::Mat src;

    src = cv_ptr->image;

    if(front_lens)
    {
        img = src(Range::all(), Range(1504, 3008));
    }
    else
    {
        img = src(Range::all(), Range(0, 1504));        
    }
    
    resize(img, img, Size(1440,1440), 0);

    // obtain the circle image ROI:
    // Mat roi(img);
    // Mat mask(roi.size(), roi.type(), Scalar::all(0));
    // circle(mask, Point(mask.cols/2,mask.rows/2), mask.rows/2-120, Scalar::all(255), -1);
    // img = roi & mask; 
    flip(img,img,-1);


    //1280-960 (GigE)
    //Mat orig;
    //resize(orig, orig, Size(1440,1440), 0);

    Mat orig;
    // Undistort
    //undistort(source, img, gige50_K, gige50_D);



    mytemplate1 = imread(target1,1);
    // mytemplate2 = imread(target2,1);
    // mytemplate3 = imread(target3,1);

    mytemplate1.copyTo(mytemplate_array[0]);
    // mytemplate2.copyTo(mytemplate_array[1]);
    // mytemplate3.copyTo(mytemplate_array[2]);


    track();

    //rbe(img);
    ok = tracker->update(img, tmp2);

    if (ok)
    {
        // Tracking success : Draw the tracked object
        rectangle(img, tmp2, Scalar( 255, 0, 0 ), 2, 1 );
        // Undistort image point to a 3d vector
        double point3D[3], point2D[2]; // the image point in pixel coordinates  
        point2D[0] = tmp2.x + tmp2.width/2;
        point2D[1] = tmp2.y + tmp2.height/2;
        circle(img, Point(point2D[0], point2D[1]), 1, Scalar( 255, 0, 0 ), 2, 8, 0 );


        // OPENCV function - undistort points
        Mat_<Point2f> points(1,1);
        points(0) = Point2f(point2D[0],point2D[1]);
        vector<Point2f> dst;// leave empty, opencv will fill it.
        undistortPoints(points, dst, K, D); 
        // For range estimation
        geometry_msgs::Twist vector_msg;
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
        target_vector_pub.publish(vector_msg);




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
        // target_vector_pub.publish(vector_msg);




    }
    else
        putText(img, "Tracking failure detected", Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);

    imshow("orig",img);
    //cout<<"sdafsdaf"<<endl;
    int key1 = waitKey(20);
    //image_pub_.publish(cv_ptr->toImageMsg());


}



///------- template matching -----------------------------------------------------------------------------------------------
Mat TplMatch( Mat &img, Mat &mytemplate )
{
    Mat result;
    // Mat mask(img.rows, img.cols, CV_8UC1, cv::Scalar(0));
    // Mat roi(mask, cv::Rect(0,0,1440,1440));
    // roi = Scalar(255);  
    matchTemplate( img, mytemplate, result, CV_TM_SQDIFF_NORMED);
    //normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );
    return result;
}

///------- Localizing the best match with minMaxLoc ------------------------------------------------------------------------
Point minmax( Mat &result, double &min )
{
    double minVal, maxVal;
    Point minLoc, maxLoc, matchLoc;
    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
    min=minVal;
    matchLoc = minLoc;
    return matchLoc;
}
///------- tracking --------------------------------------------------------------------------------------------------------
void track()
{
	if (select_flag)
	{
	//roiImg.copyTo(mytemplate);
	// select_flag = false;
	go_fast = true;
	}
	go_fast = true;
	// imshow( "mytemplate", mytemplate ); waitKey(0);
	// double target_min=100;
	// int temp_no;
	// double min;


	// Mat result = TplMatch( img, mytemplate1 );
	// Point match = minmax( result, min );
	// 	target_point=match;



    double target_min=100;
	int temp_no;
	double min;
    Rect2d bbox_tmp, bbox;

	for (int i = 0; i < template_number; ++i) 
    {
        Mat result = TplMatch( img, mytemplate_array[i] );
        Point match = minmax( result, min );
        target_array[i]=min;
        if (target_min>min) 
        {
            target_min=min;
            target_point=match;
            temp_no=i;
            Rect2d tmp(target_point.x, target_point.y, mytemplate_array[i].cols, mytemplate_array[i].rows); 
            bbox_tmp = tmp;
            
            // if(min<accuracy_threshold){
            //     goto out; // when target is found, just out
            // }
        }


	}// for
    cout<<" min: "<<target_min<<endl;


	out:;

    g_x=(target_point.x + mytemplate1.cols/2)-img.cols;
    g_y=-(target_point.y+mytemplate1.rows/2)+img.rows;
    if(initial_track && target_min < accuracy_threshold)
    {
        bbox = bbox_tmp;
        tracker->init(img, bbox);
        initial_track = false;
    }
    
    if(target_min < accuracy_threshold)
    {
        bbox = bbox_tmp;
    }

    bool ok = tracker->update(img, bbox);

    if (ok)
    {
    // Tracking success : Draw the tracked object
        rectangle(img, bbox, Scalar( 255, 0, 0 ), 2, 1 );
        // Undistort image point to a 3d vector
        double point3D[3], point2D[2]; // the image point in pixel coordinates  
        point2D[0] = bbox.x + bbox.width/2;
        point2D[1] = bbox.y + bbox.height/2;
        cam2world(point3D, point2D, &o);
        circle(img, Point(point2D[0], point2D[1]), 1, Scalar( 255, 0, 0 ), 2, 8, 0 );

        // For range estimation
        geometry_msgs::Twist vector_msg;
        // For USL hex, omnicam is mounted in the down direction.
        if(front_lens)
        {
            vector_msg.linear.x=point3D[0]; // x -> right
            vector_msg.linear.y=-point3D[1]; // y -> up
            vector_msg.linear.z=-point3D[2]; // z -> front
        }
        else
        {
            vector_msg.linear.x=-point3D[0];
            vector_msg.linear.y=point3D[2];
            vector_msg.linear.z=-point3D[1];
        }
        target_vector_pub.publish(vector_msg);
    }
    else
    {
    // Tracking failure detected.
        putText(img, "Tracking failure detected", Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);
    }
	//std::cout<<target_min<<endl;
	rectangle( img, target_point, Point( target_point.x + mytemplate1.cols , target_point.y + mytemplate1.rows ), CV_RGB(255, 0, 0), 0.5 );
	target.x=target_point.x + mytemplate1.cols;
	target.y=target_point.y + mytemplate1.rows;
	//global_frame(1,2,3);
	// std::cout << "match: " << target << endl;
  
    // imshow( "roiImg", mytemplate_array[temp_no] );


    cv::Rect templateROI(target_point.x, target_point.y,  mytemplate1.cols,  mytemplate1.rows);
    cv::Mat template_crop(img,templateROI);



}


};

static void onMouse( int event, int x, int y, int, void* )
{
    static int count(0);
    if ( event == cv::EVENT_LBUTTONDOWN ) {

        target_x = x;
        target_y = y;
        Rect2d tmp(x-10, y-10, 20, 20);
        cout<<target_x<<" "<<target_y<<endl;
        tracker->clear();
        tracker = TrackerMIL::create();
        tracker->init(img, tmp);
        rectangle( img, Point(x-10,y-10), Point(x+10,y+10), CV_RGB(255, 0, 0), 0.5 );
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter_fisheye");
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
