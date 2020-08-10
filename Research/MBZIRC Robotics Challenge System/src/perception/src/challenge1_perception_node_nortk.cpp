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
#include <ros/package.h>

#include <sstream>
#include <string>
#include <nav_msgs/Odometry.h>


#include <opencv2/core/core.hpp>

#include "opencv2/opencv.hpp"

#include <opencv2/objdetect/objdetect.hpp>

#include <sstream>

static const std::string OPENCV_WINDOW = "Image window";
using namespace cv;
using namespace std;
using std::vector;


std::string target1=ros::package::getPath("perception")+"/data/target1.png";
std::string target2=ros::package::getPath("perception")+"/data/target2.png";
std::string target3=ros::package::getPath("perception")+"/data/target3.png";
std::string target4=ros::package::getPath("perception")+"/data/target4.png";
std::string target5=ros::package::getPath("perception")+"/data/target1.png";

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
int target_detection=0;


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
double g_x, g_y;
double r_x, r_y;
Point target_point, target;


///////////////////////////////////////////////////////////////////////////////////

int FOV=65;    //////////////////// CAMERA FIELD OF VIEW ////////////////////////
double angle=FOV*3.141592/180;
double accuracy_threshold=0.08;
int target_determination;
double camera_yaw=90*3.141592/180;

//default capture width and height
const int FRAME_WIDTH = 320;
const int FRAME_HEIGHT = 240;
const int color=0;
const int track_bar=0;
const int image_on=0;
const int double_check_mode=0;
const int new_target=0;
const int field_angle=52*3.141592/180;
int template_number=2;

///////////////////////////////////////////////////////////////////////////////

int image_start=0;
int rtk_start=0;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 10*10;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
//hexacopter0/uav_control/waypoint
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
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
    createTrackbar( "Height", trackbarWindowName, &Height, Height_MAX, on_trackbar );



}
class ImageConverter
{

public: void chatterCallback(const geometry_msgs::PoseStamped& msg)
        {

                geometry_msgs::Point coord = msg.pose.position;
                d_x = coord.x;
                d_y = coord.y;
                d_z = coord.z;
		

		r_x=d_x*cos(field_angle)-d_y*sin(field_angle);
		r_y=d_x*sin(field_angle)+d_y*cos(field_angle);
		

		if(d_z>10){
		template_number=1;
		accuracy_threshold=0.08;}
		else{
		template_number=1;
		accuracy_threshold=0.1;}
		

	if(rtk_start==0){
	rtk_start=1;
	std::cout<<"----------------RTIFIX starts----------------"<<endl;}


	//std::cout<<"<"<<r_x<<" "<<r_y<<">"<<endl;

        }


public: 



private:




  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  ros::Subscriber pose_sub;
  ros::Publisher c_pub;
  //ros::Publisher point_pub;

public:
  ImageConverter()
  : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    //image_sub_ = it_.subscribe("/hexacopter/pan_tilt_camera/image_raw", 1, // SUBSCRIBE VIDEO FEED////////////////////////////
    //&ImageConverter::imageCb, this);

    image_sub_ = it_.subscribe("/hexacopter/perspective_camera/image_raw", 1, &ImageConverter::imageCb, this);



    pose_sub = nh_.subscribe("/hexacopter/mavros/local_position/pose", 1, &ImageConverter::chatterCallback, this);
    c_pub = nh_.advertise<geometry_msgs::Twist>("/hexacopter/uav_control/velocity", 10); //////////////////////DRONE VELOCITY COMMAND ///////////////////////////
    //point_pub = nh_.advertise<geometry_msgs::Pose>("/hexacopter/uav_control/waypoint", 1); //////////////////////SEND POINT COORDINATE OF THE TARGET ///////////////////////////
     
    if(image_on==1){	
    cv::namedWindow(OPENCV_WINDOW);}

  }











  ~ImageConverter()
  {
    if(image_on==1){
    cv::destroyWindow(OPENCV_WINDOW);}
  }



 void publish_vel(int x, int y, int z){

        geometry_msgs::Twist c;



	if(target_point.x>0&&target_point.y>0){
	double c_x=target_point.x-FRAME_WIDTH/2;
	double c_y=-target_point.y+FRAME_HEIGHT/2;
	c.linear.x=(c_x*cos(camera_yaw)-c_y*sin(camera_yaw))/50;
	c.linear.y=(c_x*sin(camera_yaw)+c_y*cos(camera_yaw))/50;

      c_pub.publish(c);
	}


}

 //void publish_target_position(int x, int y, int z){
//
  //      geometry_msgs::Pose c;




	
	//double c_x=(target.x-FRAME_WIDTH/2);
	//double p_x=c_x/FRAME_WIDTH*2; // normalized vector
	//double c_y=(-target.y+FRAME_HEIGHT/2);
	//double p_y=c_y/FRAME_HEIGHT*2; // normalized vector
	//c.linear.x=p_x*cos(camera_yaw)-p_y*sin(camera_yaw);
	//c.linear.y=p_x*sin(camera_yaw)+p_y*cos(camera_yaw);
	

	//double plus_x=(2*d_z*tan(angle/2))/FRAME_WIDTH*c_x;
	//double plus_y=(2*d_z*tan(angle/2))/FRAME_WIDTH*c_y;

	//g_x=d_x+plus_x;
	//g_y=d_y+plus_y;
	
//	if(g_x>24||g_x<-15||g_y>55||d_y<-55){
	//}
//	else{
//	c.position.x=g_x;
//	c.position.y=g_y;
//	c.position.z=d_z;
  //   	point_pub.publish(c);
	//}


//}

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



	if(image_start==0){
	image_start=1;
	std::cout<<"----------------TRACKING START----------------"<<endl;}

    cv::Mat src;
    cv::Mat dst, HSV, threshold;
    img = cv_ptr->image;

    resize(img, img, Size(FRAME_WIDTH, FRAME_HEIGHT));



		

		double size, half_size, x_size;
		int truck_size;
		

		if(track_bar==0){
		size=FRAME_WIDTH*1.5/(2*d_z*tan(angle/2));
		half_size=FRAME_WIDTH*0.75/(2*d_z*tan(angle/2));
		x_size=FRAME_WIDTH*0.512/(2*d_z*tan(angle/2));
		target_determination=FRAME_WIDTH*0.5/(2*d_z*tan(angle/2));}	

		if(track_bar==1){
		size=FRAME_WIDTH*1.5/(2*Height*tan(angle/2));
		half_size=FRAME_WIDTH*0.75/(2*Height*tan(angle/2));
		x_size=FRAME_WIDTH*0.512/(2*Height*tan(angle/2));
		target_determination=FRAME_WIDTH*0.5/(2*Height*tan(angle/2));}



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
	resize(mytemplate2, mytemplate2, Size(x_size, x_size));
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
    	if(image_on==1){
    	imshow(OPENCV_WINDOW, img );}
	
	if(color==1){    
	imshow( "threshold", threshold );}


                cv::waitKey(30);



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
  Point  minLoc, maxLoc, matchLoc;

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
//         select_flag = false;
        go_fast = true;
    }
	        go_fast = true;

//     imshow( "mytemplate", mytemplate ); waitKey(0);

	double target_min=100;
	
	int temp_no;
	double min;


	for (int i = 0; i < template_number; ++i) {
    Mat result  =  TplMatch( img, mytemplate_array[i] );
    Point match =  minmax( result, min ); 


		//std::cout<<i<<"    "<<min<<endl;
	

		target_array[i]=min;

		if(double_check_mode==0){
		if (target_min>min) {
		target_min=min;
		target_point=match;
		temp_no=i;
		
			
			if(min<accuracy_threshold){
			goto out;  // when target is found, just out
			}
		}
		}
	}


	out:;
	
	if(double_check_mode==1){
	if(target_array[0]>accuracy_threshold&&target_array[1]>accuracy_threshold){
		target_detection=1;
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
	}
	}


	if(target_detection==1){
	std::cout<<target_min<<endl;
    rectangle( img, target_point, Point( target_point.x + mytemplate_array[temp_no].cols , target_point.y + mytemplate_array[temp_no].rows ), CV_RGB(255, 0, 0), 0.5 );
	target.x=target_point.x + mytemplate_array[temp_no].cols;
	target.y=target_point.y + mytemplate_array[temp_no].rows;
	
	publish_vel(1,2,3);




    std::cout << "match: " << target_point << endl;
    /// latest match is the new template
    //Rect ROI = cv::Rect( match.x, match.y, mytemplate.cols, mytemplate.rows );
    //roiImg = img( ROI );
    //roiImg.copyTo(mytemplate);
    if(image_on==1){
    imshow( "roiImg", mytemplate_array[temp_no] );} //waitKey(0);
		}
}








};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
	if(track_bar==1){
	createTrackbars();}
  ros::spin();
  return 0;}
