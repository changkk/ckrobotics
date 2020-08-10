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
#include "mavros_msgs/Mavlink.h"
#include "mavros_msgs/State.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_datatypes.h>
#include <perception/fisheye_corrector.h>



static const std::string OPENCV_WINDOW = "Image window";
static const float HALF_PI = 1.57079632679;
static const unsigned int HORIZONTAL_PIXELS = 320;
static const unsigned int VERTICAL_PIXELS = 240;
using namespace cv;
using namespace std;
using std::vector;

std::string target1=ros::package::getPath("perception")+"/data/target1.png";
std::string target2=ros::package::getPath("perception")+"/data/target2.png";
std::string target3=ros::package::getPath("perception")+"/data/target3.png";
std::string target4=ros::package::getPath("perception")+"/data/target4.png";
std::string target5=ros::package::getPath("perception")+"/data/target1.png";
std::string field_image=ros::package::getPath("perception")+"/data/field.png";
int k;
Point point1, point2; /* vertical points of the bounding box */
int drag = 0;
Rect rect; /* bounding box */
Mat img, roiImg; /* roiImg - the part of the image in the bounding box */
int select_flag = 0;
bool go_fast = false;
Mat mytemplate1, mytemplate2, mytemplate3,mytemplate4,mytemplate5,mytemplate6, mytemplate7,mytemplate8,mytemplate9;
Mat mytemplate_array[10];
double target_array[5];
int target_detection;
//initial min and max HSV filter values.
//these will be changed using trackbars
int Height=5.0;
int Height_MAX = 40.0;
int H_MIN = 0;
int H_MAX = 100;
int S_MIN = 0;
int S_MAX = 100;
int V_MIN = 230;
int V_MAX = 255;
int x=0, y=0, c_x, c_y;
double d_x, d_y, d_z;
double g_x=2000, g_y=2000;
double r_x, r_y;
Point target_point, target;
double map_average_x, map_average_y;
double cur_time, prev_time;
double map_x_1, map_x_2, map_x_3, map_x_4, map_x_5;
double map_y_1, map_y_2, map_y_3, map_y_4, map_y_5;
double dt;
double pid_vel_x, pid_vel_y;
std::vector<float> ranges2;
double lidar;


//////////////////////////////////RBE///////////////////////////////////////
double map_x;
double map_y;
double pixel_x;
double pixel_y;
double drone_x;
double drone_y;
double center_field=0;
int first_detection=0;
Mat T_mxkmGkm = (Mat_<double>(2,1) << 51, 51);
Mat T_SxkmGkm = (Mat_<double>(2,2) << 5, 0, 0, 5);
Mat T_Kk = (Mat_<double>(1,1) << 0);
Mat T_mxkGk = (Mat_<double>(2,1) << 51, 51);
Mat T_SxkGk = (Mat_<double>(2,2) << 5, 0, 0, 5);
Mat T_U = (Mat_<double>(2,1) << 0.01, 0.01);
Mat T_A = (Mat_<double>(2,2) << 1, 0, 0, 1);
Mat T_mxkGkm = (Mat_<double>(2,1) << 51, 51);
Mat T_SxkGkm = (Mat_<double>(2,2) << 5, 0, 0, 5);
double truck_vel=6.2; // This is used for Truck motion model(RBE)
double truck_vel2=3.8; // WE dont use it yet
double truck_height=0.1; // to recover to recover height on this truck height
double lidar_difference_threshold=1.5;
int recover_height=7; // If the drone is on the below the truck height, recover this altitude
int lidar_height=7; // From this height, start to use lidar instead of image
///////////////////////////////////////////////////////////////////////

std::vector<long unsigned int> payload;
long unsigned int distancePayload[4] = {0};
float f;
double t0, t1, t2, t3, t4, ysqr, pitch = 0, roll = 0, yaw = 0, qx, qy, qz, qw, agl;
std_msgs::Float64 msg;


//Mavros State Callback vars
bool armed = false, guided = false;
std::string flightmode;
///////////////////////////////////////////////////////////////////////////////////

int FOV=20;//////////////////// CAMERA FIELD OF VIEW ////////////////////////
double angle=FOV*3.141592/180;// for the global mapping
int FOV2=40;
double angle2=FOV2*3.141592/180; // for the target size
int FOV3=115; // camera fov perception
double angle3=FOV3*3.141592/180; // for the fisheye target size
double accuracy_threshold=0.08;
double fisheye_accuracy_threshold=0.02;
int target_determination;
//default capture width and height
const int FRAME_WIDTH = 320;
const int FRAME_HEIGHT = 240;
const int color=0;
const int track_bar=0;
const int image_on=1;
const int simulation_on=0;
const int double_check_mode=0;
const int new_target=1;
const int field_angle=52*3.141592/180;
int template_number=2;
int vel_threshold=1;
///////////////////////////////////////////////////////////////////////////////

	int counter = 0;
	bool target_detection_buffer = false;

bool lidar_detect_flag = false;
int image_start=0;
int rtk_start=0;
int lidar_start=0;
int start_target=0;
int target_below=0;
int mission_start=0;
int perception_start=0;
int rbe_start=0;
int rbe_start2=0;
int first_descend=0;
double waypoint_z;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 10*10;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
const string trackbarWindowName = "Trackbars";
void on_trackbar( int, void* )
{//This function gets called whenever a
// trackbar position is changed
}
string intToString(int number){
std::stringstream ss;
ss << number;
return ss.str();
}


void createTrackbars(){
//create window for trackbars
namedWindow(trackbarWindowName,0);
//create memory to store trackbar name on window


char TrackbarName[50];
//sprintf( TrackbarName, "H_MIN", H_MIN);
//create trackbars and insert them into window
//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
//the max value the trackbar can move (eg. H_HIGH),
//and the function that is called whenever the trackbar is moved(eg. on_trackbar)// ----> ----> ---->

createTrackbar( "Height", trackbarWindowName, &Height, Height_MAX, on_trackbar );
}


class ImageConverter
{


public: void chatterCallback(const nav_msgs::Odometry& msg)
{
  this->mGlobalPose = msg.pose.pose;
  this->mCurrentYaw = this->computeYaw(this->mGlobalPose);
	geometry_msgs::Point coord = msg.pose.pose.position;
	d_x = coord.x;
	d_y = coord.y;
	d_z = coord.z;
	//r_x=d_x*cos(field_angle)-d_y*sin(field_angle);
	//r_y=d_x*sin(field_angle)+d_y*cos(field_angle);



	if(d_z>9){
	template_number=2;
		if(simulation_on==1){
		accuracy_threshold=0.30;
		}
		else{accuracy_threshold=0.30;}
	}
	
	else if(d_z>8){
		template_number=2;
		accuracy_threshold=0.38;
		}
		
	
	else if(d_z>5){
		template_number=2;
	 	if(simulation_on==1){
	        accuracy_threshold=0.38;
	 	}
	 	else{accuracy_threshold=0.38;}
	    }

	else{
		template_number=2;
	 	if(simulation_on==1){
	 	accuracy_threshold=0.38;
	 	}
	 	else{accuracy_threshold=0.38;}
	    }



	if(d_z>7){
	template_number=2;
		if(simulation_on==1){
		fisheye_accuracy_threshold=0.38;
		}
		else{fisheye_accuracy_threshold=0.38;}
	}
	
	else if(d_z>5){
		template_number=2;
		
		if(simulation_on==1){
		fisheye_accuracy_threshold=0.38;
		}
		else{fisheye_accuracy_threshold=0.38;}
		}
	
		

	else{
		template_number=2;
	 	if(simulation_on==1){
	 	fisheye_accuracy_threshold=0.38;
	 	}
	 	else{fisheye_accuracy_threshold=0.38;}
	}




	if(rtk_start==0){
	rtk_start=1;
	std::cout<<"----------------RTIFIX starts----------------"<<endl;}
	//std::cout<<"<"<<r_x<<" "<<r_y<<">"<<endl;

//
//	double difference_lidar=d_z-agl;
//	cout<<"dfgasdfadsfasdfasdfasdfasdfasdfsadfasdfa"<<difference_lidar<<endl;


//	if(first_detection==1){
//		if(lidar==0){
//		target_below=0;
//		}
//		else{
//			if(difference_lidar>1.5){
//			target_below=1;
//			cout<<"---------TARGET IS JUST BELOW THE DRONE!!-----------"<<endl;
//			geometry_msgs::Pose c;
//			c.position.z=d_z-0.2;
//			point_pub.publish(c);
//			}
//			else{target_below=0;
//			}
//		}
//	}


	//if(d_z<truck_height&&mission_start==1){
	//mission_start=0;
	//first_detection=0;
	//map_x=1000;
	//map_y=1000;
	//geometry_msgs::Pose c;
	//c.position.x=3;
	//c.position.y=3;
	//c.position.z=recover_height;
	//waypoint_z=recover_height;
	//point_pub.publish(c);

    	//ros::Duration(6).sleep();
	//mission_start=1;
	//}
}


void MavrosStateCallback(const mavros_msgs::State msg)
{
	armed = msg.armed;
	guided = msg.guided;
	flightmode = msg.mode;
}


void pid_x_callback(const std_msgs::Float64::ConstPtr& msg)
{
	pid_vel_x=msg->data;
}


void pid_y_callback(const std_msgs::Float64::ConstPtr& msg)
{
	pid_vel_y=msg->data;
}


void start_callback(const std_msgs::Empty& msg)
{
	cout<<"takeoff Start 1 (Top right to bottom left)"<<endl;

	int ini_alt = d_z;
	int count = 0;

		while(d_z < (ini_alt + 2) && count < 5)
		{
			count++;
			if (!armed)
		{
				takeoff_pub.publish(std_msgs::Empty());
			}
			ros::Duration(1).sleep();		
		}

	cout<<"go to start point"<<endl;
	        	geometry_msgs::Pose c;
			c.position.x=3;
			c.position.y=3;
			c.position.z=10;
     		point_pub.publish(c);
			ros::Duration(1).sleep();		

	mission_start=1;
	rbe_start=1;


}

void start2_callback(const std_msgs::Empty& msg)
{
	cout<<"takeoff Start 2 (Top left to bottom right)"<<endl;

	int ini_alt = d_z;
	int count = 0;

		while(d_z < (ini_alt + 2) && count < 5)
		{
			count++;
			if (!armed)
			{
				takeoff_pub.publish(std_msgs::Empty());
			}
			ros::Duration(7).sleep();		
		}

	cout<<"go to start point"<<endl;
	        	geometry_msgs::Pose c;
			c.position.x=3;
			c.position.y=3;
			c.position.z=12;
     		point_pub.publish(c);
			ros::Duration(8).sleep();		

	mission_start=1;
	rbe_start2=1;

}

void estop_callback(const std_msgs::Empty& msg)
{

	mission_start=0;



			geometry_msgs::Pose d;
			d.position.x=0;
			d.position.y=-20;
			d.position.z=10;
     			point_pub.publish(d);


    	ros::Duration(10).sleep();
	landing_pub.publish(std_msgs::Empty());
	cout<<"------------Emergency Landing--------------"<<endl;




}


void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) // For Simulation
{

ranges2=msg->ranges;
auto min_range = min_element(ranges2.begin(), ranges2.end());
double min_range_double=*min_range;
double difference_lidar=d_z-min_range_double;

if(first_detection==1){

		if(difference_lidar>1.5 && difference_lidar < 4){
		target_below=1;
		cout<<"---------TARGET IS JUST BELOW THE DRONE!!-----------"<<endl;
		//geometry_msgs::Pose c;
		// waypoint_z=waypoint_z-0.1;
		lidar_detect_flag = true;
		//c.position.z=waypoint_z;
		//point_pub.publish(c);
		}
		else{target_below=0;
		lidar_detect_flag = false;}
	}


	cout<<"adfadsfadsfasdfsdafsdafsdfsafdsa"<<waypoint_z<<endl;
}


void lidar_callback(const std_msgs::Float64::ConstPtr& msg)
{
	if(lidar_start==0){
	lidar_start=1;
	std::cout<<"----------------LIDAR starts----------------"<<endl;}
	lidar=msg->data;
	double difference_lidar=d_z-lidar;
	//cout<<"dsafasdfasdfasdfsadfasdfasdfasdf"<<difference_lidar<<endl;

if(first_detection==1 && d_z<5){
		if(difference_lidar>lidar_difference_threshold && difference_lidar < 4){
		target_below=1;
		cout<<"---------TARGET IS JUST BELOW THE DRONE!!-----------"<<endl;
		//geometry_msgs::Pose c;
		// waypoint_z=waypoint_z-0.3;
		lidar_detect_flag = true;
		//c.position.z=waypoint_z;
		//point_pub.publish(c);

		}
		else{
		target_below=0;
		lidar_detect_flag = false;}
	}
    
}


//void MavlinkMsgCallback(const mavros_msgs::Mavlink mesg)
//{
//	if(lidar_start==0){
//	lidar_start=1;
//	std::cout<<"----------------LIDAR starts----------------"<<endl;}
//
//	if (mesg.msgid == 173)
//	{
//	payload.clear();
//	payload = mesg.payload64;
//		for (int i = 0; i < 4; i++)
//		{
//		distancePayload[i] = payload[i];
//		}
//	char *pul = (char *)&distancePayload; // ok, char* can alias any type
//	char *pf = (char *)&f; // ok, char* can alias any type
//	memcpy(pf, pul, sizeof(float));
//	}
//}



//void MavrosImuCallback(const sensor_msgs::Imu mesg1)
//{
//	qx = mesg1.orientation.x;
//	qy = mesg1.orientation.y;
//	qz = mesg1.orientation.z;
//	qw = mesg1.orientation.w;
//	ysqr = qy * qy;
//	// roll (x-axis rotation)
//	t0 = +2.0 * (qw * qx + qy * qz);
//	t1 = +1.0 - 2.0 * (qx * qx + ysqr);
//	roll = std::atan2(t0, t1);
//	// pitch (y-axis rotation)
//	t2 = +2.0 * (qw * qy - qz * qx);
//	t2 = t2 > 1.0 ? 1.0 : t2;
//	t2 = t2 < -1.0 ? -1.0 : t2;
//	pitch = std::asin(t2);
//	// yaw (z-axis rotation)
//	t3 = +2.0 * (qw * qz + qx * qy);
//	t4 = +1.0 - 2.0 * (ysqr + qz * qz);
//	yaw = std::atan2(t3, t4);
//	// std::cout << pitch << " " << roll << " " << yaw << " " << std::endl;
//	agl = f * std::cos(roll) * std::cos(pitch);
//	// std::cout << f << " " << agl << std::endl;
//	}


public:

private:

double vel_x_1, vel_x_2, vel_x_3, vel_x_4, vel_x_5;
double vel_y_1, vel_y_2, vel_y_3, vel_y_4, vel_y_5;
ros::NodeHandle nh_;
image_transport::ImageTransport it_;
image_transport::Subscriber image_sub_;
ros::Subscriber pose_sub;
ros::Subscriber pid_x_sub;
ros::Subscriber pid_y_sub;
ros::Subscriber laser_sub;// For Simulation
ros::Subscriber lidar_sub;
ros::Subscriber mavros_state;
//ros::Subscriber sub0;
//ros::Subscriber sub1;
ros::Publisher c_pub;
ros::Publisher point_pub;
ros::Publisher landing_pub;
ros::Publisher takeoff_pub;
ros::Subscriber start_sub;
ros::Subscriber start2_sub;
ros::Subscriber estop_sub;

FisheyeCorrector* mFisheyeCorrector;
cv_bridge::CvImagePtr mFisheyeImagePtr;
image_transport::Subscriber mFisheyeSubscriber;
image_transport::Publisher mFisheyePublisher;
ros::Publisher mPseudoWaypointPublisher;
geometry_msgs::Pose mGlobalPose;
float mCurrentYaw;
float mOffsetYaw;
bool mDoFisheyeCorrection;

public:
ImageConverter(std::string fisheyeCalibrationFile)
: it_(nh_)
{
// Subscrive to input video feed and publish output video feed
//image_sub_ = it_.subscribe("/hexacopter/pan_tilt_camera/image_raw", 1, // SUBSCRIBE VIDEO FEED////////////////////////////
//&ImageConverter::imageCb, this);
//sub0 = nh_.subscribe("mavlink/from", 1, &ImageConverter::MavlinkMsgCallback, this);
//sub1 = nh_.subscribe("mavros/imu/data", 1, &ImageConverter::MavrosImuCallback, this);
pose_sub = nh_.subscribe("/hexacopter/localizer/global_odom", 1, &ImageConverter::chatterCallback, this);
pid_x_sub = nh_.subscribe("/hexacopter/pid/x_position/control_effort", 1, &ImageConverter::pid_x_callback, this);
image_sub_ = it_.subscribe("/hexacopter/perspective_camera/image_raw", 1, &ImageConverter::imageCb, this);


mavros_state = nh_.subscribe("/hexacopter/mavros/state", 1, &ImageConverter::MavrosStateCallback, this);


laser_sub = nh_.subscribe("/rrbot/laser/scan", 1, &ImageConverter::laser_callback, this); // For Simulation
lidar_sub = nh_.subscribe("hexacopter0/rangefinder", 1, &ImageConverter::lidar_callback, this);
pid_y_sub = nh_.subscribe("/hexacopter/pid/y_position/control_effort", 1, &ImageConverter::pid_y_callback, this);
c_pub = nh_.advertise<geometry_msgs::Twist>("/hexacopter/uav_control/velocity2", 10); //////////////////////DRONE VELOCITY COMMAND ///////////////////////////
point_pub = nh_.advertise<geometry_msgs::Pose>("/hexacopter/uav_control/waypoint2", 1); //////////////////////SEND POINT COORDINATE OF THE TARGET ///////////////////////////
this->mFisheyeSubscriber = it_.subscribe("/hexacopter/fisheye_camera/image_raw", 1, &ImageConverter::fisheyeCallback, this);
this->mFisheyePublisher = it_.advertise("/hexacopter/fisheye_camera/image_modified", 1);
this->mPseudoWaypointPublisher = nh_.advertise<geometry_msgs::Pose>("/hexacopter/uav_control/pseudo_waypoint", 1);

landing_pub = nh_.advertise<std_msgs::Empty>("/hexacopter/uav_control/land", 1);
takeoff_pub = nh_.advertise<std_msgs::Empty>("/hexacopter/uav_control/takeoff", 1);
start_sub = nh_.subscribe("/start_challenge1", 1, &ImageConverter::start_callback, this);
start2_sub = nh_.subscribe("/start2_challenge1", 1, &ImageConverter::start_callback, this);
estop_sub = nh_.subscribe("/estop_challenge1", 1, &ImageConverter::estop_callback, this);
this->mCurrentYaw = 0.0;
if (!nh_.getParam("/hexacopter/yaw_offset", this->mOffsetYaw)) {
  this->mOffsetYaw = 0.0;
}
this->mFisheyeCorrector = new FisheyeCorrector(fisheyeCalibrationFile, VERTICAL_PIXELS, HORIZONTAL_PIXELS);
this->mDoFisheyeCorrection = true;

if(image_on==1){
cv::namedWindow(OPENCV_WINDOW);}
}

~ImageConverter()
{
if(image_on==1){
cv::destroyWindow(OPENCV_WINDOW);}
  if (this->mFisheyeCorrector != NULL) {
    delete this->mFisheyeCorrector;
  }
}
// void publish_vel(int x, int y, int z){
// geometry_msgs::Twist c;
//	if(target_point.x>0&&target_point.y>0){
//	double c_x=target_point.x-FRAME_WIDTH/2;
//	double c_y=-target_point.y+FRAME_HEIGHT/2;
//	c.linear.x=(c_x*cos(this->mCurrentYaw)-c_y*sin(this->mCurrentYaw))/100;
//	c.linear.y=(c_x*sin(this->mCurrentYaw)+c_y*cos(this->mCurrentYaw))/100;
//
// c_pub.publish(c);
//	}
//}

void global_frame(int x, int y, int z){



	double c_x=(target.x-FRAME_WIDTH/2);
	//double p_x=c_x/FRAME_WIDTH*2; // normalized vector
	double c_y=(-target.y+FRAME_HEIGHT/2);
	//double p_y=c_y/FRAME_HEIGHT*2; // normalized vector
	//c.linear.x=p_x*cos(this->mCurrentYaw)-p_y*sin(this->mCurrentYaw);
	//c.linear.y=p_x*sin(this->mCurrentYaw)+p_y*cos(this->mCurrentYaw);
	double plus_x=(2*d_z*tan(angle/2))/FRAME_WIDTH*c_x;
	double plus_y=(2*d_z*tan(angle/2))/FRAME_WIDTH*c_y;
	double plus_rotation_x, plus_rotation_y;

	if(simulation_on==1){
	plus_rotation_x=plus_x*cos(this->mCurrentYaw)+plus_y*sin(this->mCurrentYaw);
	plus_rotation_y=-plus_x*sin(this->mCurrentYaw)+plus_y*cos(this->mCurrentYaw);}

	if(simulation_on==0){
	plus_rotation_x=plus_x*cos(this->mCurrentYaw)-plus_y*sin(this->mCurrentYaw);
	plus_rotation_y=plus_x*sin(this->mCurrentYaw)+plus_y*cos(this->mCurrentYaw);}
	g_x=d_x+plus_rotation_x;
	g_y=d_y+plus_rotation_y;

}


void imageCb(const sensor_msgs::ImageConstPtr& msg)
{


	if(mission_start==1){


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


	if(image_start==0){
	image_start=1;
	std::cout<<"----------------TRACKING START----------------"<<endl;}
	cv::Mat src;
	cv::Mat dst, HSV, threshold;

	img = cv_ptr->image;

	cv::Mat field= imread(field_image,1);

	resize(img, img, Size(FRAME_WIDTH, FRAME_HEIGHT));

	double size, half_size, x_size;

	int truck_size;


	if(track_bar==0){
	size=FRAME_WIDTH*1/(2*(d_z)*tan(angle2/2));
	half_size=FRAME_WIDTH*0.5/(2*d_z*tan(angle2/2));
	x_size=FRAME_WIDTH*0.512/(2*d_z*tan(angle2/2));
	target_determination=FRAME_WIDTH*0.5/(2*d_z*tan(angle2/2));}

	if(track_bar==1){
	size=FRAME_WIDTH*1/(2*(Height)*tan(angle2/2));
	half_size=FRAME_WIDTH*0.5/(2*Height*tan(angle2/2));
	x_size=FRAME_WIDTH*0.512/(2*Height*tan(angle2/2));
	target_determination=FRAME_WIDTH*0.5/(2*Height*tan(angle2/2));}

//std::cout<<"Height "<<d_z<<endl;
	if(size>FRAME_HEIGHT||size<0){
	size=FRAME_HEIGHT-10;}
	if(half_size>FRAME_HEIGHT||half_size<0){
	half_size=FRAME_HEIGHT-10;}
	if(new_target==0){
	mytemplate1 = imread(target1,1);
	resize(mytemplate1, mytemplate1, Size(size, size));
	mytemplate1.copyTo(mytemplate_array[0]);

	mytemplate2 = imread(target2,1);
	resize(mytemplate2, mytemplate2, Size(half_size, half_size));
	mytemplate2.copyTo(mytemplate_array[1]);}


	if(new_target==1){
	mytemplate1 = imread(target3,1);
	resize(mytemplate1, mytemplate1, Size(size, size));
	mytemplate1.copyTo(mytemplate_array[0]);

	mytemplate2 = imread(target4,1);
	resize(mytemplate2, mytemplate2, Size(size, size));
	mytemplate2.copyTo(mytemplate_array[1]);}

	//mytemplate3 = imread(target3,1);
	//resize(mytemplate3, mytemplate3, Size(size, size));
	//mytemplate3.copyTo(mytemplate_array[2]);

	//mytemplate4 = imread(target4,1);
	//resize(mytemplate4, mytemplate4, Size(size, size));
	//mytemplate4.copyTo(mytemplate_array[3]);

	//mytemplate5 = imread(target5,1);
	//resize(mytemplate5, mytemplate5, Size(size, size));
	//mytemplate5.copyTo(mytemplate_array[4]);


	track();


	if(g_x>30||g_x<-30||g_y>45||d_y<-45){
	target_detection=0;
	}


	if(rbe_start==1){
	rbe(field,g_x,g_y);}

	if(rbe_start2==1){
	rbe2(field,g_x,g_y);}


	if(image_on==1){
	imshow(OPENCV_WINDOW, img );
	imshow("global",field);
	}


	cv::waitKey(30);

	} // mission start
}


void fisheyeCallback(const sensor_msgs::ImageConstPtr& msg) {


	if(d_z<5){

  try {
    this->mFisheyeImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    if (this->mFisheyeImagePtr != NULL) {
      if (this->mFisheyeCorrector != NULL && this->mDoFisheyeCorrection) {
        this->mFisheyeImagePtr->image = this->mFisheyeCorrector->undistortImage(this->mFisheyeImagePtr->image);
      }
      this->mFisheyePublisher.publish(this->mFisheyeImagePtr->toImageMsg());
    }
    else {
      ROS_ERROR("fisheyeCallback mCvImagePtr is NULL.");
    }
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
	Mat fisheye=mFisheyeImagePtr->image;

	//resize(fisheye, fisheye, Size(FRAME_WIDTH, FRAME_HEIGHT));

	double fisheye_size;


	if(track_bar==0){
	fisheye_size=FRAME_WIDTH*1/(2*(d_z-truck_height)*tan(angle3/2));}

	if(track_bar==1){
	fisheye_size=FRAME_WIDTH*1/(2*(Height-truck_height)*tan(angle3/2));}



	//std::cout<<"Height "<<fisheye_size<<endl;
	if(fisheye_size>FRAME_HEIGHT||fisheye_size<0){
	fisheye_size=FRAME_HEIGHT-10;}
	if(new_target==0){
	mytemplate1 = imread(target1,1);
	resize(mytemplate1, mytemplate1, Size(fisheye_size, fisheye_size));
	mytemplate1.copyTo(mytemplate_array[0]);

	mytemplate2 = imread(target2,1);
	resize(mytemplate2, mytemplate2, Size(fisheye_size, fisheye_size));
	mytemplate2.copyTo(mytemplate_array[1]);}


	if(new_target==1){
	mytemplate1 = imread(target3,1);
	resize(mytemplate1, mytemplate1, Size(fisheye_size, fisheye_size));
	mytemplate1.copyTo(mytemplate_array[0]);

	mytemplate2 = imread(target4,1);
	resize(mytemplate2, mytemplate2, Size(fisheye_size, fisheye_size));
	mytemplate2.copyTo(mytemplate_array[1]);}

	//mytemplate3 = imread(target3,1);
	//resize(mytemplate3, mytemplate3, Size(size, size));
	//mytemplate3.copyTo(mytemplate_array[2]);

	//mytemplate4 = imread(target4,1);
	//resize(mytemplate4, mytemplate4, Size(size, size));
	//mytemplate4.copyTo(mytemplate_array[3]);

	//mytemplate5 = imread(target5,1);
	//resize(mytemplate5, mytemplate5, Size(size, size));
	//mytemplate5.copyTo(mytemplate_array[4]);


	track_fisheye(fisheye);


	if(image_on==1){
	imshow("fisheye",fisheye);
	//cout<<"fisheye!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;	
	}



	} // if d_z<5


}

///------- template matching -----------------------------------------------------------------------------------------------
Mat TplMatch( Mat &img, Mat &mytemplate )
{
Mat result;
matchTemplate( img, mytemplate, result, CV_TM_SQDIFF_NORMED );
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
	double target_min=100;
	int temp_no;
	double min;
	for (int i = 0; i < template_number; ++i) {
	Mat result = TplMatch( img, mytemplate_array[i] );
	Point match = minmax( result, min );
std::cout<<i<<" "<<min<<endl;


	target_array[i]=min;
	if(double_check_mode==0){
		if (target_min>min) {
		target_min=min;
		target_point=match;
		temp_no=i;
			if(min<accuracy_threshold){
				goto out; // when target is found, just out
			}
		}
	}

	}// for


	out:;

	if(double_check_mode==1){
		if(target_array[0]>accuracy_threshold&&target_array[1]>accuracy_threshold){
		target_detection=1;
		first_detection=1;
		}
		else if(target_min==0){
		target_detection=0;
		}
		else{
		target_detection=0;
		}
	}

	if(double_check_mode==0){
		if(target_min>accuracy_threshold){
		target_detection=0;
		}
		else if(target_min==0){
		target_detection=0;
		}
		else{
		target_detection=1;
		first_detection=1;
		}
	}


	if(target_detection==1){
	//std::cout<<target_min<<endl;
	rectangle( img, target_point, Point( target_point.x + mytemplate_array[temp_no].cols , target_point.y + mytemplate_array[temp_no].rows ), CV_RGB(255, 0, 0), 0.5 );
	target.x=target_point.x + mytemplate_array[temp_no].cols;
	target.y=target_point.y + mytemplate_array[temp_no].rows;
	//global_frame(1,2,3);
	std::cout << "match: " << target << endl;
	/// latest match is the new template
	//Rect ROI = cv::Rect( match.x, match.y, mytemplate.cols, mytemplate.rows );
	//roiImg = img( ROI );
	//roiImg.copyTo(mytemplate);



	
		if(image_on==1){
		imshow( "roiImg", mytemplate_array[temp_no] );
		} //waitKey(0);

	}


}



///------- template matching -----------------------------------------------------------------------------------------------
Mat TplMatch_fisheye( Mat &img, Mat &mytemplate )
{
Mat result;
matchTemplate( img, mytemplate, result, CV_TM_SQDIFF_NORMED );
//normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );
return result;
}

///------- Localizing the best match with minMaxLoc ------------------------------------------------------------------------
Point minmax_fisheye( Mat &result, double &min )
{
double minVal, maxVal;
Point minLoc, maxLoc, matchLoc;
minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
min=minVal;
matchLoc = minLoc;
return matchLoc;
}
///------- tracking --------------------------------------------------------------------------------------------------------
void track_fisheye(Mat &fisheye)
{
	if (select_flag)
	{
	//roiImg.copyTo(mytemplate);
	// select_flag = false;
	go_fast = true;
	}
	go_fast = true;
	// imshow( "mytemplate", mytemplate ); waitKey(0);
	double target_min=100;
	int temp_no;
	double min;
	for (int i = 0; i < template_number; ++i) {
	Mat result = TplMatch_fisheye( fisheye, mytemplate_array[i] );
	Point match = minmax_fisheye( result, min );
//std::cout<<i<<" "<<min<<endl;


	target_array[i]=min;
	if(double_check_mode==0){
		if (target_min>min) {
		target_min=min;
		target_point=match;
		temp_no=i;
			if(min<fisheye_accuracy_threshold){
				goto out; // when target is found, just out
			}
		}
	}

	}// for


	out:;
int fisheye_target_detection;

		if(target_min>fisheye_accuracy_threshold){
		int fisheye_target_detection=0;
		}
		else if(target_min==0){
		int fisheye_target_detection=0;
		}
		else{
		int fisheye_target_detection=1;
		}
	


	if(fisheye_target_detection==1){
	//std::cout<<target_min<<endl;
	rectangle( fisheye, target_point, Point( target_point.x + mytemplate_array[temp_no].cols , target_point.y + mytemplate_array[temp_no].rows ), CV_RGB(255, 0, 0), 0.5 );
	target.x=target_point.x + mytemplate_array[temp_no].cols;
	target.y=target_point.y + mytemplate_array[temp_no].rows;
	//global_frame(1,2,3);
	//std::cout << "match: " << target << endl;
	/// latest match is the new template
	//Rect ROI = cv::Rect( match.x, match.y, mytemplate.cols, mytemplate.rows );
	//roiImg = img( ROI );
	//roiImg.copyTo(mytemplate);




		geometry_msgs::Twist fisheye_vel;
		double fisheye_x=(target.x-FRAME_WIDTH/2)/320;
		double fisheye_y=(-target.y+FRAME_HEIGHT/2)/320;

	fisheye_vel.linear.x=fisheye_x*cos(this->mCurrentYaw)+fisheye_y*sin(this->mCurrentYaw);
	fisheye_vel.linear.y=-fisheye_x*sin(this->mCurrentYaw)+fisheye_y*cos(this->mCurrentYaw);

		c_pub.publish(fisheye_vel);


		if(image_on==1){
		imshow( "roiImg_fisheye", mytemplate_array[temp_no] );
		} //waitKey(0);

	}
}


void rbe (Mat &field, double target_x, double target_y){


	if(map_x<3&&map_x>-3&&map_y<3&&map_y>-3)
	{center_field=1;}
	else
	{center_field=0;}


//When both color and circle is detected, RBE is launched using target point(the color position when the conditions are satisfied)

double R = 16;
Mat T_B = (Mat_<double>(2,2) << 1, 0, 0, 1);
Mat T_C = (Mat_<double>(2,2) << 1, 0, 0, 1);
Mat T_A_t = T_A.t();
Mat T_B_t = T_B.t();
Mat T_C_t = T_C.t();
int a=-15;
double a_down=-24.75;
int b=15;
double b_up=24.75;
int f=22;
int g=22;
Mat S_X = (Mat_<double>(2,1) << d_x,d_y);
Mat T_X = (Mat_<double>(2,1) << target_x,target_y); //white color point when the circle is detected at the same time.
Mat T_Swkm = (Mat_<double>(2,2) << 1, 0, 0, 1);
//if(target_x<100)
//{first_detection=true;
//geometry_msgs::Twist f;
//f.linear.x=1; // if detection is started, this is publish
//f.linear.y=sqrt((map_x-map_x2)*(map_x-map_x2)+(map_y-map_y2)*(map_y-map_y2)); // prediction model and real model distance
//f_pub.publish(f);
//}


	if(first_detection==0)
	{
	T_U = (Mat_<double>(2,1) << 0.0000000000000001, 0.000000000000000001);
	//Predict
	T_A = (Mat_<double>(2,2) << 1, 0, 0, 1);
	T_mxkGkm = T_A*T_mxkmGkm + T_B*T_U;
	T_SxkGkm = T_A*T_SxkmGkm*T_A_t + T_Swkm;
	} // first_detection==false

	if(first_detection==1)
	{
	//if(map_x>-1&&map_y<1)
	//{map_x=target_x;map_y=target_y;}
	//map_x=target_x;map_y=target_y;
		if(map_y<b&&map_y>a)
		{
			if(map_x>0)
			{
				if(map_y>a&&map_y<0)
				{T_U = (Mat_<double>(2,1) << -truck_vel/1.414/g, truck_vel/1.414/g);}
				if(map_y>0&&map_y<b)
				{T_U = (Mat_<double>(2,1) << -truck_vel/1.414/g, -truck_vel/1.414/g);}
				if(map_x<3&&map_x>0&&map_y<0&&map_y>-3)// right down -> left up
				{T_mxkGkm.at<double>(0,0)=-1;T_mxkGkm.at<double>(1,0)=1;
				T_U = (Mat_<double>(2,1) << -truck_vel/1.414/g, truck_vel/1.414/g);
				goto out;
				}
				if(map_x<3&&map_x>0&&map_y<3&&map_y>0) // right up -> left down
				{T_mxkGkm.at<double>(0,0)=-1;T_mxkGkm.at<double>(1,0)=-1;
				T_U = (Mat_<double>(2,1) << -truck_vel/1.414/g, -truck_vel/1.414/g);
				goto out;
				}
			}

			if(map_x<0)
			{
				if(map_y>a&&map_y<0)
				{T_U = (Mat_<double>(2,1) << -truck_vel/1.414/g, -truck_vel/1.414/g);}
				if(map_y>0&&map_y<b)
				{T_U = (Mat_<double>(2,1) << -truck_vel/1.414/g, truck_vel/1.414/g);}
			}

	//Predict
		T_A = (Mat_<double>(2,2) << 1, 0, 0, 1);
		T_mxkGkm = T_A*T_mxkmGkm + T_B*T_U;
		T_SxkGkm = T_A*T_SxkmGkm*T_A_t + T_Swkm;
		}
	out:;

	if(map_y>b)
	{
		if(map_y<b_up)
		{
			if(map_x<0)
			{
				T_mxkGkm.at<double>(0,0) = -R*cos(atan((b_up-map_y)/(-map_x))-truck_vel/f/R);
				T_mxkGkm.at<double>(1,0) = b_up-R*sin(atan((b_up-map_y)/(-map_x))-truck_vel/f/R);
			}
			if(map_x>0)
			{
			T_mxkGkm.at<double>(0,0) = R*cos(atan((b_up-map_y)/(map_x))+truck_vel/f/R);
			T_mxkGkm.at<double>(1,0) = b_up-R*sin(atan((b_up-map_y)/(map_x))+truck_vel/f/R);
			}
		}

		if(map_y>b_up)
		{
			if(map_x<0)
			{
			T_mxkGkm.at<double>(0,0) = -R*cos(atan((map_y-b_up)/(-map_x))+truck_vel/f/R);
			T_mxkGkm.at<double>(1,0) = b_up+R*sin(atan((map_y-b_up)/(-map_x))+truck_vel/f/R);
			}
			if(map_x>0)
			{
			T_mxkGkm.at<double>(0,0) = R*cos(atan((map_y-b_up)/(map_x))-truck_vel/f/R);
			T_mxkGkm.at<double>(1,0) = b_up+R*sin(atan((map_y-b_up)/(map_x))-truck_vel/f/R);
			}
		}


		T_A = (Mat_<double>(2,2) << 1, 0, 0, 1);
		T_U = (Mat_<double>(2,1) << 0.00000000000001, 0.0000000000000001);
	}


	if(map_y<a)
	{
	//T_A = (Mat_<double>(2,2) << (R*(a-map_y)*sin(atan((a-map_y)/(-map_x-40))+truck_vel/30/R))/(a*a-2*a*map_y+map_x*map_x+80*map_x	+map_y*map_y+1600), (R*(map_x+40)*sin(atan((-a+map_y)/(map_x+40))+truck_vel/30/R))/(a*a-2*a*map_y+map_x*map_x+80*map_x+map_y*map_y+1600), -	(R*(a-map_y)*cos(atan((a-map_y)/(-map_x-40))+truck_vel/30/R))/(a*a-2*a*map_y+map_x*map_x+80*map_x+map_y*map_y+1600), -(R*(map_x	+40)*cos(atan((-a+map_y)/(map_x+40))+truck_vel/30/R))/(a*a-2*a*map_y+map_x*map_x+80*map_x+map_y*map_y+1600));

		if(map_y>a_down)
		{
			if(map_x<0)
			{
			T_mxkGkm.at<double>(0,0) = -R*cos(atan((map_y-a_down)/(-map_x))-truck_vel/f/R);
			T_mxkGkm.at<double>(1,0) = a_down+R*sin(atan((map_y-a_down)/(-map_x))-truck_vel/f/R);
			}
			if(map_x>0)
			{
			T_mxkGkm.at<double>(0,0) = R*cos(atan((map_y-a_down)/(map_x))+truck_vel/f/R);
			T_mxkGkm.at<double>(1,0) = a_down+R*sin(atan((map_y-a_down)/(map_x))+truck_vel/f/R);
			}
		}
		if(map_y<a_down)
		{
			if(map_x<0)
			{
			T_mxkGkm.at<double>(0,0) = -R*cos(atan((a_down-map_y)/(-map_x))+truck_vel/f/R);
			T_mxkGkm.at<double>(1,0) = a_down-R*sin(atan((a_down-map_y)/(-map_x))+truck_vel/f/R);
			}
			if(map_x>0)
			{
			T_mxkGkm.at<double>(0,0) = R*cos(atan((a_down-map_y)/(map_x))-truck_vel/f/R);
			T_mxkGkm.at<double>(1,0) = a_down-R*sin(atan((a_down-map_y)/(map_x))-truck_vel/f/R);
			}
		}

		T_A = (Mat_<double>(2,2) << 1, 0, 0, 1);
		T_U = (Mat_<double>(2,1) << 0.0000000000001, 0.000000000000000001);
	}

	}//first detection==true


		cout<<target_detection<<endl;
	//std::cout<<"dddddddddddddddddddddddddddddddddddddd"<<" "<<T_mxkGk.at<double>(0,0)<<" "<<T_mxkGk.at<double>(0,1)<<" "<<map_x<<" 	"<<map_y<<target_detection<<std::endl;
	T_X = T_X + T_B*T_U;
	Mat ST_Z = T_X;
	Mat dst = ST_Z-S_X;
	Mat multi=dst.t()*dst;
	double d = sqrt(multi.at<double>(0,0));
	double th = atan(dst.at<double>(1,0)/dst.at<double>(0,0));
	Mat T_Svk = (Mat_<double>(2,2) << cos(th),-sin(th),sin(th),cos(th))*(Mat_<double>(2,2) << d,0, 0,0.2*d);
	Mat eye = (Mat_<double>(2,2) << 1, 0, 0, 1);

	if(target_detection==1){

		if(center_field==0&&d_z>2)
		{
		//Correction
		T_Kk = T_SxkGkm*T_C_t/(T_C*T_SxkGkm*T_C_t + T_Svk);
		T_mxkGk = T_mxkGkm + T_Kk*(ST_Z - T_C*T_mxkGkm);
		T_SxkGk = (eye - T_Kk*T_C)*T_SxkGkm;
		}
		}
		else
		{//when no detection, it does not correct, only predict
		T_Kk = T_Kk;
		T_mxkGk = T_mxkGkm;
		T_SxkGk = T_SxkGkm;
		}

	prev_time=cur_time;
	cur_time =ros::Time::now().toSec();
	dt=cur_time-prev_time;
	//prev_time = cur_time;
	//cur_time = ros::Time::now();
	//double dt = cur_time - prev_time;
	drone_x=200+d_x*20/3; // drone pixel point x
	drone_y=300-d_y*20/3; // drone pixel point y
	map_x=T_mxkGk.at<double>(0,0); // target global point x
	map_y=T_mxkGk.at<double>(1,0); // target global point y

	double dist=sqrt((d_x-map_x)*(d_x-map_x)+(d_y-map_y)*(d_y-map_y));

	map_x_5=map_x_4;
	map_y_5=map_y_4;
	map_x_4=map_x_3;
	map_y_4=map_y_3;
	map_x_3=map_x_2;
	map_y_3=map_y_2;
	map_x_2=map_x_1;
	map_y_2=map_y_1;
	map_x_1=map_x;
	map_y_1=map_y;

	if(map_y_5==0){
	}
	else{
	vel_x_1=(map_x_4-map_x_5)/dt;
	vel_y_1=(map_y_4-map_y_5)/dt;
	vel_x_2=(map_x_3-map_x_4)/dt;
	vel_y_2=(map_y_3-map_y_4)/dt;
	vel_x_3=(map_x_2-map_x_3)/dt;
	vel_y_3=(map_y_2-map_y_3)/dt;
	vel_x_4=(map_x_1-map_x_2)/dt;
	vel_y_4=(map_y_1-map_y_2)/dt;
	vel_x_5=(map_x-map_x_1)/dt;
	vel_y_5=(map_y-map_y_1)/dt;
	map_average_x=(vel_x_1+vel_x_2+vel_x_3+vel_x_4+vel_x_5)/5+pid_vel_x;
	map_average_y=(vel_y_1+vel_y_2+vel_y_3+vel_y_4+vel_y_5)/5+pid_vel_y;
	

	//cout<<"Truck :"<<(vel_x_1+vel_x_2+vel_x_3+vel_x_4+vel_x_5)/5<<" "<<(vel_y_1+vel_y_2+vel_y_3+vel_y_4+vel_y_5)/5<< "	"<<  endl;
	//cout<<"Drone :"<<map_average_x<<" "<<map_average_y<<endl;
	}

// target_detection;

	if(first_detection==1){
	//if(d_z>4){
		
	if(first_descend==0){


		waypoint_z=9;

	first_descend=1;
	}



	if (target_detection == 0)
	{
	counter++;
		if (counter >= 5)
		{
			target_detection_buffer = false;
			counter = 6;

		}
		
	}
	else
	{
			target_detection_buffer = true;
			counter = 0;

	}





	if(d_z>5){

		if(dist<vel_threshold && target_detection_buffer == true)
		{
		geometry_msgs::Twist c;
		c.linear.x=map_average_x;
		c.linear.y=map_average_y;
		
		if(lidar_detect_flag = true)
		{
			c.linear.z=-1.0;
		}
		else
		{
			c.linear.z=-0.5;
		}

		c_pub.publish(c);

		geometry_msgs::Pose d;
		d.position.x=T_mxkGk.at<double>(0,0);
		d.position.y=T_mxkGk.at<double>(1,0);
		d.position.z=waypoint_z; //This will not be used unless we publish it to the z velocity topic ourselves.
		mPseudoWaypointPublisher.publish(d);
		}

		if (dist>vel_threshold && target_detection_buffer == true){
			// geometry_msgs::Twist c;
			// c.linear.x=map_average_x;
			// c.linear.y=map_average_y;
			// c.linear.z=0;
			// c_pub.publish(c);

			// geometry_msgs::Pose d;
			// d.position.x=T_mxkGk.at<double>(0,0);
			// d.position.y=T_mxkGk.at<double>(1,0);
			// d.position.z=waypoint_z; //This will not be used unless we publish it to the z velocity topic ourselves.
			// mPseudoWaypointPublisher.publish(d);

			geometry_msgs::Pose de;
			
			if (T_mxkGk.at<double>(0,0) > 20){
				de.position.x=20;
			}
			else if (T_mxkGk.at<double>(0,0) < -20){
				de.position.x=-20;
			}
			else
			{				
				de.position.x=T_mxkGk.at<double>(0,0);
			}

			
			
			if (T_mxkGk.at<double>(1,0) > 30){
				de.position.y=30;
			}
			else if (T_mxkGk.at<double>(1,0) < -30){
				de.position.y=-30;
			}
			else
			{				
				de.position.y=T_mxkGk.at<double>(1,0);
			}




			if (d_z > 3 && d_z < 4)
			{
				de.position.z= 4;	//This will be used.
			}
			else if (d_z > 4 && d_z < 5)
			{
				de.position.z= 5;	//This will be used.
			}
			else if (d_z > 5 && d_z < 6)
			{
				de.position.z= 6;	//This will be used.
			}
			else if (d_z < 3)
			{
				de.position.z= 7;	//This will be used.
			}
			else
			{
				de.position.z= 7;	//This will be used.
			}
			point_pub.publish(de);

		}

		if (target_detection_buffer == false)
		{
			geometry_msgs::Pose df;
			df.position.x=T_mxkGk.at<double>(0,0);
			df.position.y=T_mxkGk.at<double>(1,0);
			df.position.z= 7;	//This will be used.
			point_pub.publish(df);


		}


	} //if d_z>5






	}

	pixel_x=200+T_mxkGk.at<double>(0,0)*20/3; // target pixel point x
	pixel_y=300-T_mxkGk.at<double>(1,0)*20/3; // target pixel point y
	//cv::Point2f mean(pixel_x, pixel_y);
	//cv::RotatedRect ellipse = getErrorEllipse(2.4477, mean, 100*T_SxkGk);
	// std::cout << "Error "<<T_mxkGk.at<double>(0,0)<<" "<<T_mxkGk.at<double>(1,0)<<" "<<dst<<" "<<dst.at<double>(1,0)<< std::endl;
	//cv::ellipse(field, ellipse, cv::Scalar::all(255), 2);
	cv::circle(field,Point(pixel_x,pixel_y),10,Scalar(0,0,255),1);
	line(field,Point(pixel_x,pixel_y),Point(pixel_x,pixel_y-10),Scalar(0,0,255),1);
	line(field,Point(pixel_x,pixel_y),Point(pixel_x,pixel_y+10),Scalar(0,0,255),1);
	line(field,Point(pixel_x,pixel_y),Point(pixel_x-10,pixel_y),Scalar(0,0,255),1);
	line(field,Point(pixel_x,pixel_y),Point(pixel_x+10,pixel_y),Scalar(0,0,255),1);
	//drone global position on map
	cv::circle(field,Point(drone_x,drone_y),10,Scalar(255,0,0),-1);
	line(field,Point(drone_x,drone_y),Point(pixel_x,pixel_y),Scalar(0,0,255),1);
	T_mxkmGkm = T_mxkGk;
	T_SxkmGkm = T_SxkGk;

} // rbe




void rbe2 (Mat &field, double target_x, double target_y){


	if(map_x<3&&map_x>-3&&map_y<3&&map_y>-3)
	{center_field=1;}
	else
	{center_field=0;}


//When both color and circle is detected, RBE is launched using target point(the color position when the conditions are satisfied)

double R = 16;
Mat T_B = (Mat_<double>(2,2) << 1, 0, 0, 1);
Mat T_C = (Mat_<double>(2,2) << 1, 0, 0, 1);
Mat T_A_t = T_A.t();
Mat T_B_t = T_B.t();
Mat T_C_t = T_C.t();
int a=-15;
double a_down=-24.75;
int b=15;
double b_up=24.75;
int f=22;
int g=22;
Mat S_X = (Mat_<double>(2,1) << d_x,d_y);
Mat T_X = (Mat_<double>(2,1) << target_x,target_y); //white color point when the circle is detected at the same time.
Mat T_Swkm = (Mat_<double>(2,2) << 1, 0, 0, 1);
//if(target_x<100)
//{first_detection=true;
//geometry_msgs::Twist f;
//f.linear.x=1; // if detection is started, this is publish
//f.linear.y=sqrt((map_x-map_x2)*(map_x-map_x2)+(map_y-map_y2)*(map_y-map_y2)); // prediction model and real model distance
//f_pub.publish(f);
//}


	if(first_detection==0)
	{
	T_U = (Mat_<double>(2,1) << 0.0000000000000001, 0.000000000000000001);
	//Predict
	T_A = (Mat_<double>(2,2) << 1, 0, 0, 1);
	T_mxkGkm = T_A*T_mxkmGkm + T_B*T_U;
	T_SxkGkm = T_A*T_SxkmGkm*T_A_t + T_Swkm;
	} // first_detection==false

	if(first_detection==1)
	{
	//if(map_x>-1&&map_y<1)
	//{map_x=target_x;map_y=target_y;}
	//map_x=target_x;map_y=target_y;
		if(map_y<b&&map_y>a)
		{
			if(map_x>0)
			{
				if(map_y>a&&map_y<0) //left up -> right down
				{T_U = (Mat_<double>(2,1) << truck_vel/1.414/g, -truck_vel/1.414/g);}
				if(map_y>0&&map_y<b) // left down -> right up
				{T_U = (Mat_<double>(2,1) << truck_vel/1.414/g, truck_vel/1.414/g);}
				if(map_x>-3&&map_x<0&&map_y<0&&map_y>-3)// left down -> right up
				{T_mxkGkm.at<double>(0,0)=1;T_mxkGkm.at<double>(1,0)=1;
				T_U = (Mat_<double>(2,1) << truck_vel/1.414/g, truck_vel/1.414/g);
				goto out;
				}
				if(map_x>-3&&map_x<0&&map_y<3&&map_y>0) // left up -> right down
				{T_mxkGkm.at<double>(0,0)=1;T_mxkGkm.at<double>(1,0)=-1;
				T_U = (Mat_<double>(2,1) << truck_vel/1.414/g, -truck_vel/1.414/g);
				goto out;
				}
			}
			if(map_x<0)
			{
				if(map_y>a&&map_y<0) // left down -> right up
				{T_U = (Mat_<double>(2,1) << truck_vel/1.414/g, truck_vel/1.414/g);}
				if(map_y>0&&map_y<b) // left up -> right down
				{T_U = (Mat_<double>(2,1) << truck_vel/1.414/g, -truck_vel/1.414/g);}
			}

	//Predict
		T_A = (Mat_<double>(2,2) << 1, 0, 0, 1);
		T_mxkGkm = T_A*T_mxkmGkm + T_B*T_U;
		T_SxkGkm = T_A*T_SxkmGkm*T_A_t + T_Swkm;
		}
	out:;

	if(map_y>b)
	{
		if(map_y<b_up)
		{
			if(map_x<0)
			{
				T_mxkGkm.at<double>(0,0) = -R*cos(atan((b_up-map_y)/(-map_x))+truck_vel/f/R);
				T_mxkGkm.at<double>(1,0) = b_up-R*sin(atan((b_up-map_y)/(-map_x))+truck_vel/f/R);
			}
			if(map_x>0)
			{
			T_mxkGkm.at<double>(0,0) = R*cos(atan((b_up-map_y)/(map_x))-truck_vel/f/R);
			T_mxkGkm.at<double>(1,0) = b_up-R*sin(atan((b_up-map_y)/(map_x))-truck_vel/f/R);
			}
		}

		if(map_y>b_up)
		{
			if(map_x<0)
			{
			T_mxkGkm.at<double>(0,0) = -R*cos(atan((map_y-b_up)/(-map_x))-truck_vel/f/R);
			T_mxkGkm.at<double>(1,0) = b_up+R*sin(atan((map_y-b_up)/(-map_x))-truck_vel/f/R);
			}
			if(map_x>0)
			{
			T_mxkGkm.at<double>(0,0) = R*cos(atan((map_y-b_up)/(map_x))+truck_vel/f/R);
			T_mxkGkm.at<double>(1,0) = b_up+R*sin(atan((map_y-b_up)/(map_x))+truck_vel/f/R);
			}
		}


		T_A = (Mat_<double>(2,2) << 1, 0, 0, 1);
		T_U = (Mat_<double>(2,1) << 0.00000000000001, 0.0000000000000001);
	}


	if(map_y<a)
	{
	//T_A = (Mat_<double>(2,2) << (R*(a-map_y)*sin(atan((a-map_y)/(-map_x-40))+truck_vel/30/R))/(a*a-2*a*map_y+map_x*map_x+80*map_x	+map_y*map_y+1600), (R*(map_x+40)*sin(atan((-a+map_y)/(map_x+40))+truck_vel/30/R))/(a*a-2*a*map_y+map_x*map_x+80*map_x+map_y*map_y+1600), -	(R*(a-map_y)*cos(atan((a-map_y)/(-map_x-40))+truck_vel/30/R))/(a*a-2*a*map_y+map_x*map_x+80*map_x+map_y*map_y+1600), -(R*(map_x	+40)*cos(atan((-a+map_y)/(map_x+40))+truck_vel/30/R))/(a*a-2*a*map_y+map_x*map_x+80*map_x+map_y*map_y+1600));

		if(map_y>a_down)
		{
			if(map_x<0)
			{
			T_mxkGkm.at<double>(0,0) = -R*cos(atan((map_y-a_down)/(-map_x))+truck_vel/f/R);
			T_mxkGkm.at<double>(1,0) = a_down+R*sin(atan((map_y-a_down)/(-map_x))+truck_vel/f/R);
			}
			if(map_x>0)
			{
			T_mxkGkm.at<double>(0,0) = R*cos(atan((map_y-a_down)/(map_x))-truck_vel/f/R);
			T_mxkGkm.at<double>(1,0) = a_down+R*sin(atan((map_y-a_down)/(map_x))-truck_vel/f/R);
			}
		}
		if(map_y<a_down)
		{
			if(map_x<0)
			{
			T_mxkGkm.at<double>(0,0) = -R*cos(atan((a_down-map_y)/(-map_x))-truck_vel/f/R);
			T_mxkGkm.at<double>(1,0) = a_down-R*sin(atan((a_down-map_y)/(-map_x))-truck_vel/f/R);
			}
			if(map_x>0)
			{
			T_mxkGkm.at<double>(0,0) = R*cos(atan((a_down-map_y)/(map_x))+truck_vel/f/R);
			T_mxkGkm.at<double>(1,0) = a_down-R*sin(atan((a_down-map_y)/(map_x))+truck_vel/f/R);
			}
		}

		T_A = (Mat_<double>(2,2) << 1, 0, 0, 1);
		T_U = (Mat_<double>(2,1) << 0.0000000000001, 0.000000000000000001);
	}

	}//first detection==true


	//std::cout<<"dddddddddddddddddddddddddddddddddddddd"<<" "<<T_mxkGk.at<double>(0,0)<<" "<<T_mxkGk.at<double>(0,1)<<" "<<map_x<<" 	"<<map_y<<target_detection<<std::endl;
	T_X = T_X + T_B*T_U;
	Mat ST_Z = T_X;
	Mat dst = ST_Z-S_X;
	Mat multi=dst.t()*dst;
	double d = sqrt(multi.at<double>(0,0));
	double th = atan(dst.at<double>(1,0)/dst.at<double>(0,0));
	Mat T_Svk = (Mat_<double>(2,2) << cos(th),-sin(th),sin(th),cos(th))*(Mat_<double>(2,2) << d,0, 0,0.2*d);
	Mat eye = (Mat_<double>(2,2) << 1, 0, 0, 1);

	if(target_detection==1){

		if(center_field==0&&d_z>2)
		{
		//Correction
		T_Kk = T_SxkGkm*T_C_t/(T_C*T_SxkGkm*T_C_t + T_Svk);
		T_mxkGk = T_mxkGkm + T_Kk*(ST_Z - T_C*T_mxkGkm);
		T_SxkGk = (eye - T_Kk*T_C)*T_SxkGkm;
		}
		}
		else
		{//when no detection, it does not correct, only predict
		T_Kk = T_Kk;
		T_mxkGk = T_mxkGkm;
		T_SxkGk = T_SxkGkm;
		}

	prev_time=cur_time;
	cur_time =ros::Time::now().toSec();
	dt=cur_time-prev_time;
	//prev_time = cur_time;
	//cur_time = ros::Time::now();
	//double dt = cur_time - prev_time;
	drone_x=200+d_x*20/3; // drone pixel point x
	drone_y=300-d_y*20/3; // drone pixel point y
	map_x=T_mxkGk.at<double>(0,0); // target global point x
	map_y=T_mxkGk.at<double>(1,0); // target global point y

	double dist=sqrt((d_x-map_x)*(d_x-map_x)+(d_y-map_y)*(d_y-map_y));

	map_x_5=map_x_4;
	map_y_5=map_y_4;
	map_x_4=map_x_3;
	map_y_4=map_y_3;
	map_x_3=map_x_2;
	map_y_3=map_y_2;
	map_x_2=map_x_1;
	map_y_2=map_y_1;
	map_x_1=map_x;
	map_y_1=map_y;

	if(map_y_5==0){
	}
	else{
	vel_x_1=(map_x_4-map_x_5)/dt;
	vel_y_1=(map_y_4-map_y_5)/dt;
	vel_x_2=(map_x_3-map_x_4)/dt;
	vel_y_2=(map_y_3-map_y_4)/dt;
	vel_x_3=(map_x_2-map_x_3)/dt;
	vel_y_3=(map_y_2-map_y_3)/dt;
	vel_x_4=(map_x_1-map_x_2)/dt;
	vel_y_4=(map_y_1-map_y_2)/dt;
	vel_x_5=(map_x-map_x_1)/dt;
	vel_y_5=(map_y-map_y_1)/dt;
	map_average_x=(vel_x_1+vel_x_2+vel_x_3+vel_x_4+vel_x_5)/5+pid_vel_x;
	map_average_y=(vel_y_1+vel_y_2+vel_y_3+vel_y_4+vel_y_5)/5+pid_vel_y;
	//cout<<map_average_x<<" "<<map_average_y<<endl;
	}


	if(first_detection==1){
	//if(d_z>4){


	
	
		if(dist<vel_threshold){
		geometry_msgs::Twist c;
		c.linear.x=map_average_x;
		c.linear.y=map_average_y;
		c.linear.z=-0.1;
		c_pub.publish(c);
		//geometry_msgs::Pose d;
		//d.position.z=d_z-0.5;
		//point_pub.publish(d);
	
		geometry_msgs::Pose d;
		d.position.x=T_mxkGk.at<double>(0,0);
		d.position.y=T_mxkGk.at<double>(1,0);
		d.position.z=waypoint_z;
		mPseudoWaypointPublisher.publish(d);
		}
		else{
		geometry_msgs::Pose c;
		c.position.x=T_mxkGk.at<double>(0,0);
		c.position.y=T_mxkGk.at<double>(1,0);
		c.position.z=waypoint_z;
		point_pub.publish(c);
		}
	//}
	//else{
	//geometry_msgs::Twist c;
	//c.linear.x=0;
	//c.linear.y=0;
	//c.linear.z=-1;
	//c_pub.publish(c);
	//}
	}
	pixel_x=200+T_mxkGk.at<double>(0,0)*20/3; // target pixel point x
	pixel_y=300-T_mxkGk.at<double>(1,0)*20/3; // target pixel point y
	//cv::Point2f mean(pixel_x, pixel_y);
	//cv::RotatedRect ellipse = getErrorEllipse(2.4477, mean, 100*T_SxkGk);
	// std::cout << "Error "<<T_mxkGk.at<double>(0,0)<<" "<<T_mxkGk.at<double>(1,0)<<" "<<dst<<" "<<dst.at<double>(1,0)<< std::endl;
	//cv::ellipse(field, ellipse, cv::Scalar::all(255), 2);
	cv::circle(field,Point(pixel_x,pixel_y),10,Scalar(0,0,255),1);
	line(field,Point(pixel_x,pixel_y),Point(pixel_x,pixel_y-10),Scalar(0,0,255),1);
	line(field,Point(pixel_x,pixel_y),Point(pixel_x,pixel_y+10),Scalar(0,0,255),1);
	line(field,Point(pixel_x,pixel_y),Point(pixel_x-10,pixel_y),Scalar(0,0,255),1);
	line(field,Point(pixel_x,pixel_y),Point(pixel_x+10,pixel_y),Scalar(0,0,255),1);
	//drone global position on map
	cv::circle(field,Point(drone_x,drone_y),10,Scalar(255,0,0),-1);
	line(field,Point(drone_x,drone_y),Point(pixel_x,pixel_y),Scalar(0,0,255),1);
	T_mxkmGkm = T_mxkGk;
	T_SxkmGkm = T_SxkGk;

} // rbe2

float computeYaw(const geometry_msgs::Pose& pose) const {
  tf::Quaternion quaternion;
  double roll, pitch, yaw;
  tf::quaternionMsgToTF(pose.orientation, quaternion);
  tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  return yaw - HALF_PI + this->mOffsetYaw;
}

}; // image converter

int main(int argc, char** argv)
{

ros::init(argc, argv, "image_converter");
ros::NodeHandle rosNode;
std::string fisheyeCalibrationFile = "/etc/camera_info/fisheye_camera.txt";
rosNode.param(ros::this_node::getName() + "/fisheye_calibration", fisheyeCalibrationFile, fisheyeCalibrationFile);
ImageConverter ic(fisheyeCalibrationFile);

if(track_bar==1){
createTrackbars();}
ros::spin();

return 0;}
