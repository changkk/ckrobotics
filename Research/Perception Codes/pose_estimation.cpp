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
#include "pose_estimation.h"



#include <opencv2/core/core.hpp>

static const std::string OPENCV_WINDOW = "Image window";
using namespace cv;
using std::vector;
using namespace std;
ofstream myfile;

double n_cx;
double n_cy;
double f_cx=721.2172155119429;
double f_cy=721.9473150432297;

//double b=4786.405854; // 5m
double b=1851.086438; // 2m
double gige_sensor_width=4.8;
double gige_sensor_height=3.6;
double gige_pixel_size=0.00375;  

double fisheye_sensor_width=3.2;
double fisheye_sensor_height=3.2;
double fl_1=1;
double fl_2;
double alpha;

double normal_roll, normal_pitch, normal_yaw;
double fisheye_roll, fisheye_pitch, fisheye_yaw;


Mat basler_lo=(Mat_<double>(2, 1) << -2.162, 2.527);
Mat omni_lo=(Mat_<double>(2, 1) << -3.985, 4.145);

double ro_angle=atan(1.618/1.823);
Mat drone_r = (Mat_<double>(2, 2) << cos(ro_angle), -sin(ro_angle), sin(ro_angle), cos(ro_angle));



//GigE zoom0
Mat gige0_K = (Mat_<double>(3, 3) << 1296.6925639253514, 0.0, 642.7956006508207, 0.0, 1305.4672890088902, 475.32342351959164, 0.0, 0.0, 1.0);
Mat gige0_D = (Mat_<double>(1, 4) << -0.289312, 0.359446, -0.000467, 0.000284);

//GigE zoom50
Mat gige50_K = (Mat_<double>(3, 3) << 3251.4617167620245, 0.0, 664.6412636008588, 0.0, 3274.9962244450044, 474.08490776791405, 0.0, 0.0, 1.0);
Mat gige50_D = (Mat_<double>(1, 4) << -0.029953, -1.056533, -0.010504, 0.010500);

Mat rotation_gige, rotation_fisheye;
Mat rotation_gige_inv;

double zoom;
int zoom_compute;

int maximum_length =4;
int minimum_length = 0;
int find_number = 8;
int target_determination = 9;


double normal_x, normal_y, fisheye_x, fisheye_y;
double drone_x, drone_y, drone_z;
double ground_truth;

Point prepoint1,prepoint2,prepoint3,prepoint4,prepoint5,prepoint6,prepoint7;


const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
const int MAX_NUM_OBJECTS=50;
const int MIN_OBJECT_AREA = 10*10;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;

class ImageConverter
{

public: 


// Mat minus=drone_r*basler_lo;

void chatterCallback(const nav_msgs::Odometry& msg)	
{
//   geometry_msgs::Point coord = msg.pose.pose.position;
// 	drone_x = coord.x;
// 	drone_y = coord.y;
// 	drone_z = coord.z;
// 	Mat drone = drone_r * (Mat_<double>(2, 1) << drone_x, drone_y);
//   ground_truth=drone.at<double>(1,0)-minus.at<double>(1,0);
  //cout<<R23<<endl;

}

void normal_point(const geometry_msgs::Twist& msg)	
{
		normal_x = msg.linear.x;
		normal_y = msg.linear.y;

}

void fisheye_point(const geometry_msgs::Twist& msg)	
{
		fisheye_x = msg.linear.x;
		fisheye_y = msg.linear.y;
	
}

void normal_rotation(const geometry_msgs::Twist& msg)	
{
		
        
    normal_roll = msg.linear.x;
		normal_pitch = msg.linear.y;
    normal_yaw = msg.linear.z; 
    	rotation321(normal_roll,normal_pitch,normal_yaw,rotation_gige);
    	//rotation321(0,6.92693*3.14/180,70.9219*3.14/180,rotation_gige);
    	 //rotation321(0*3.14/180,10*3.14/180,72*3.14/180,rotation_gige);


        //cout<<"normal angle: "<<normal_roll*180/3.141592<<" "<<normal_pitch*180/3.141592<<" "<<normal_yaw*180/3.141592<<endl;    

        rotation_gige_inv = rotation_gige.inv();


}

void fisheye_rotation(const geometry_msgs::Twist& msg)	
{


		fisheye_roll = msg.linear.x;
		fisheye_pitch = msg.linear.y;
    fisheye_yaw = msg.linear.z;
        // roll 4 , pitch -2.5 , yaw 110
        //cout<<"fisheye angle: "<<fisheye_roll*180/3.141592<<" "<<fisheye_pitch*180/3.141592<<" "<<fisheye_yaw*180/3.141592<<endl;    
    	rotation321(fisheye_roll,fisheye_pitch,fisheye_yaw,rotation_fisheye);

	
}

void range_callback(const std_msgs::Float64::ConstPtr& msg)
{
	double range=msg->data;
	
	
	// zoom = sensor_size*range*prcnt_coverage/target_size
    double desired_focal = 0.00626*range*0.1/1;

    if(desired_focal <= 0.0048)
	{
		zoom = 0;
	}
	else if(desired_focal >= 0.0576)
    {
		zoom = 100;
	}    
    else
        zoom_compute = floor((1893.93*desired_focal-6.09)/10.0);
		zoom = zoom_compute * 10;

    //zoom = 0;

    ground_truth=range;


    if(zoom==0)
    {
        fl_2=gige0_K.at<double>(0,0) * gige_pixel_size;
        alpha=fl_2/fl_1;
        n_cx=gige0_K.at<double>(0,2);
        n_cy=gige0_K.at<double>(1,2);
    }

    if(zoom==10)
    {
        fl_2=10.08;
        alpha=fl_2/fl_1;
        n_cx=gige0_K.at<double>(0,2);
        n_cy=gige0_K.at<double>(1,2);
    }

    if(zoom==20)
    {
        fl_2=15.36;
        alpha=fl_2/fl_1;
        n_cx=gige0_K.at<double>(0,2);
        n_cy=gige0_K.at<double>(1,2);
    }

    if(zoom==30)
    {
        fl_2=20.64;
        alpha=fl_2/fl_1;
        n_cx=gige0_K.at<double>(0,2);
        n_cy=gige0_K.at<double>(1,2);
    }

    if(zoom==40)
    {
        fl_2=25.92;
        alpha=fl_2/fl_1;
        n_cx=gige0_K.at<double>(0,2);
        n_cy=gige0_K.at<double>(1,2);
    }

	cout<<"range :"<<range<<" zoom :"<<zoom<<" focal length:"<<fl_2<<endl;

}

private:
	double x, y, z;
	double g_x;
	double g_y;

  Mat T_mxkmGkm = (Mat_<double>(2,1) << 0, 0);
  Mat T_SxkmGkm = (Mat_<double>(2,2) << 5, 0, 0, 5);
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber image_sub2_;

  //image_transport::Publisher image_pub_;
  ros::Subscriber pose_sub;
    
  ros::Subscriber normal_sub;
  ros::Subscriber fisheye_sub;
  ros::Subscriber normal_rotation_sub;
  ros::Subscriber fisheye_rotation_sub;
  ros::Subscriber range_sub;


  //ros::Publisher p_pub;
  //ros::Publisher s_pub;
  //ros::Publisher i_pub;

	//Mat K = (Mat_<double>(3, 3) << 445.17984003885283, 0.0, 721.2172155119429, 0.0, 445.8816513187479, 721.9473150432297, 0.0, 0.0, 1.0);
	//Mat D = (Mat_<double>(1, 4) << -0.017496068223756347, -0.003243014339251435, 0.000839223410670477, -0.0004219234265229322);
  //Mat R23 = (Mat_<double>(3,3) << 0.9829706631605084, 0.03155695516204952, 0.1810326875088894, -0.04080955589524314, 0.9980317905089683, 0.04761433902823904, -0.1791738136933624, -0.05419136199029742, 0.9823237962974397);

  //double n_cx=942.8561335296305;
  //double n_cy=523.5090860380981;

public:
  ImageConverter()
  : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/undistort_fisheye", 1,
    &ImageConverter::imageCb2, this);
    image_sub2_ = it_.subscribe("//camera/image_raw", 1,
    &ImageConverter::imageCb, this);
    
    //image_pub_ = it_.advertise("/image_converter/output_video", 1);

    normal_sub = nh_.subscribe("/image_frame_point", 1000, &ImageConverter::normal_point, this);
    fisheye_sub = nh_.subscribe("/image_frame_point_fisheye", 1000, &ImageConverter::fisheye_point, this);

    normal_rotation_sub = nh_.subscribe("/gige_rotation", 1000, &ImageConverter::normal_rotation, this);
    fisheye_rotation_sub = nh_.subscribe("/fisheye_rotation", 1000, &ImageConverter::fisheye_rotation, this);
    pose_sub = nh_.subscribe("/hexacopter/gps/rtkfix", 1000, &ImageConverter::chatterCallback, this);
    range_sub = nh_.subscribe("/range", 1000, &ImageConverter::range_callback, this);


    //p_pub = nh_.advertise<geometry_msgs::Twist>("/rbe_target_point", 10);
    //s_pub = nh_.advertise<geometry_msgs::Twist>("/observation_global_point", 10);
    //i_pub = nh_.advertise<geometry_msgs::Twist>("/image_frame_point_fisheye", 10);

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

    cv::Mat src_normal;

    src_normal = cv_ptr->image;

	currentFrame = src_normal;

	ImageProcessor imageProcessor;
	imageProcessor.imgHeight = currentFrame.rows;
	//cout << "\nimage height" << imageProcessor.imgHeight;
	imageProcessor.imgWidth = currentFrame.cols;

	//cout << "\nimage width" << imageProcessor.imgWidth;
	imageProcessor.processImage(src_normal);


  
	//imshow("normal",src_normal);
	int key1 = waitKey(20);

}

 void imageCb2(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr2;
    try
    {
      cv_ptr2 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat src_fishehye;

    src_fishehye = cv_ptr2->image;
    resize(src_fishehye,src_fishehye,Size(500,500));
    //imshow("fisheye",src_fishehye);
  	int key1 = waitKey(20);

}



void rotation321(double th1, double th2, double th3, Mat &resultmatrix){

	double pi=3.141592;

	//th1=th1*pi/180;
	//th2=th2*pi/180;
	//th3=th3*pi/180;

  double m00=cos(th2)*cos(th3);
  double m01=-cos(th2)*sin(th3);
  double m02=sin(th2);
  double m10=sin(th1)*sin(th2)*cos(th3)+cos(th1)*sin(th3);
  double m11=-sin(th1)*sin(th2)*sin(th3)+cos(th1)*cos(th3);
  double m12=-sin(th1)*cos(th2);
  double m20=-cos(th1)*sin(th2)*cos(th3)+sin(th1)*sin(th3);
  double m21=cos(th1)*sin(th2)*sin(th3)+sin(th1)*cos(th3);
  double m22=cos(th1)*cos(th2);



   resultmatrix = (Mat_<double>(3,3) << m00,m01,m02,m10,m11,m12,m20,m21,m22);

}

void getEulerAngles(Mat &rotCamerMatrix,Vec3d &eulerAngles){

    Mat cameraMatrix,rotMatrix,transVect,rotMatrixX,rotMatrixY,rotMatrixZ;
    double* _r = rotCamerMatrix.ptr<double>();
    double projMatrix[12] = {_r[0],_r[1],_r[2],0,
                          _r[3],_r[4],_r[5],0,
                          _r[6],_r[7],_r[8],0};

    decomposeProjectionMatrix( Mat(3,4,CV_64FC1,projMatrix),
                               cameraMatrix,
                               rotMatrix,
                               transVect,
                               rotMatrixX,
                               rotMatrixY,
                               rotMatrixZ,
                               eulerAngles);
}


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter_range");
  ImageConverter ic;
  myfile.open ("range.txt");
  ros::spin();
  myfile.close();
  return 0;
}
