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
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include "ocam_functions.h"

#include <opencv2/core/core.hpp>

static const std::string OPENCV_WINDOW = "Image window";
using namespace cv;
using std::vector;
using namespace std;

	
	
int H_MIN = 0;
int H_MAX = 10;
int S_MIN = 0;
int S_MAX = 10;
int V_MIN = 200;
int V_MAX = 255;

double omnicam_location_ned[3] = {7.244, -4.593, -0.191};

double roll,pitch,yaw;


    Mat T_mxkmGkm = (Mat_<double>(2,1) << 0, 0);
    Mat T_SxkmGkm = (Mat_<double>(2,2) << 5, 0, 0, 5);

	Mat T_A = (Mat_<double>(2,2) << 1, 0, 0, 1);
	Mat T_B = (Mat_<double>(2,2) << 1, 0, 0, 1);
	Mat T_C = (Mat_<double>(2,2) << 1, 0, 0, 1);
	Mat T_A_t = T_A.t();
	Mat T_B_t = T_B.t();
	Mat T_C_t = T_C.t();
	Mat T_U = (Mat_<double>(2,1) << 1, 1);

	Mat T_Swkm = (Mat_<double>(2,2) << 1, 0, 0, 1);
    Mat T_Svk = (Mat_<double>(2,2) << 1, 0, 0, 1);

    Mat T_mxkGkm;
    Mat T_SxkGkm;
    Mat T_mxkGk;
    Mat T_SxkGk;
    Mat T_Kk;

    Mat eye = (Mat_<double>(2,2) << 1, 0, 0, 1);

    Mat ST_Z;
	Point maxpoint, maxpoint_prev;

	int first_detection=0;
    int detection=0;
    int first_double=0;
	int first_maxpoint=0;
    double v_x, v_y;

	double g_x, g_x_prev;
	double g_y, g_y_prev;
    double g_distance;

	Mat preorig20_orig;


	vector<Point2f> color;

int maximum_length =100;
//double minimum_length =1.5; // long baseline
double minimum_length =4; // short baseline
int find_number = 100;
int target_determination = 9;

double pi=3.141592;
double focal_length = 17.2*0.001;
double ccd_x=2.3*0.001;
double ccd_y=1.6*0.001;

Point prepoint1,prepoint2,prepoint3,prepoint4,prepoint5,prepoint6,prepoint7;
int minimum_distance = 10;
double average;

const int FRAME_WIDTH = 1440;
const int FRAME_HEIGHT = 1440;
const int MAX_NUM_OBJECTS=50;

const int MIN_OBJECT_AREA = 10*10;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;

vector<Point2f> trackedFeatures;
Mat prevGray;
Mat gray; 
vector<Point2f> corners;
vector<uchar> status; vector<float> errors;
bool freshStart=true;
int image_hold = 0;
Mat_<float>     rigidTransform=Mat::eye(3,3,CV_32FC1);
Mat H = Mat::eye(3,3,CV_32FC1);
const double INLIER_THRESHOLD = 50.0; // pixel distance
int opencv_inliers;
double angle_first=0;
int first=0;


class Tracker {

public:

    //Tracker():freshStart(true) {
      //  rigidTransform = Mat::eye(3,3,CV_32FC1); //affine 2x3 in a 3x3 matrix
    //}

    void processImage(Mat& img) {
        cvtColor(img,gray,CV_BGR2GRAY);
    
        if(freshStart==true){
            H = Mat::eye(3,3,CV_32FC1); //affine 2x3 in a 3x3 matrix
        }

        if(trackedFeatures.size() < 200) {
            goodFeaturesToTrack(gray,corners,300,0.01,10);
            //cout << "found " << corners.size() << " features\n";
            for (int i = 0; i < corners.size(); ++i) {
                trackedFeatures.push_back(corners[i]);
            }
        }

        if(!prevGray.empty()) {
            calcOpticalFlowPyrLK(prevGray,gray,trackedFeatures,corners,status,errors,Size(5,5));

            if(first==0)
            {
                first=1;
            }

            if(countNonZero(status) < status.size() * 0.8) {
                cout << "cataclysmic error \n";
                angle_first=0;
                H = Mat::eye(3,3,CV_32FC1);
                trackedFeatures.clear();
                prevGray.release();
                freshStart = true;
                return;
            } else
                freshStart = false; 
                
            vector<uchar> status_homography;
               
                //cout<<freshStart<<endl;
            if(freshStart==false)
            {
                Mat_<float> new_H = findHomography(trackedFeatures, corners, status_homography, CV_RANSAC, INLIER_THRESHOLD);
                opencv_inliers = accumulate(status_homography.begin(), status_homography.end(), 0);
				cout<<"inlinders: "<<opencv_inliers<<endl;
                //Mat_<float> nrt33 = Mat_<float>::eye(3,3);
                //new_H.copyTo(nrt33.rowRange(0,3));
                //H *= nrt33;
                H = new_H * H;
            }

            //Mat_<float> newRigidTransform = estimateRigidTransform(trackedFeatures,corners,false);




            trackedFeatures.clear();
            for (int i = 0; i < status.size(); ++i) {
                if(status[i]) {
                    trackedFeatures.push_back(corners[i]);
                }
            }
        }

        gray.copyTo(prevGray);
    }
};



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

void imuCallback(const sensor_msgs::Imu& msg2)	
{



  static tf2_ros::TransformBroadcaster br;
  
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "rtk_base_station";
  transformStamped.child_frame_id = "omnicam";

  transformStamped.transform.translation.x = omnicam_location_ned[0]; //NED frame
  transformStamped.transform.translation.y = omnicam_location_ned[1]; //NED frame
  transformStamped.transform.translation.z = omnicam_location_ned[2]; //NED frame
  // tf2::Quaternion q;
  // q.setRPY(0, 0, msg->theta);





        	geometry_msgs::Quaternion q = msg2.orientation;
        //q1=imu.x;
       // q1=imu.y;
        //q1=imu.z;
        //q1=imu.w;


	// roll (x-axis rotation)
	double sinr = +2.0 * (q.w * q.x + q.y * q.z);
	double cosr = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
	double roll = atan2(sinr, cosr);
	double pitch;
	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w * q.y - q.z * q.x);
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny = +2.0 * (q.w * q.z + q.x * q.y);
	double cosy = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
	double yaw = atan2(siny, cosy);

///////////////////////////////////////////////////////////////////////////////////////



	//refers to pixhawk frame, pitch, roll, yaw
    // cout<<pitch*180/3.141592<<" "<<roll*180/3.141592<<" "<<yaw*180/3.141592<<endl;


	Mat rotation_fisheye;

	double fisheye_roll = pitch;
	double fisheye_pitch = roll;
	double fisheye_yaw = 55*3.141592/180;


        // Abbreviations for the various angular functions
	double cy = cos(fisheye_yaw * 0.5);
	double sy = sin(fisheye_yaw * 0.5);
	double cr = cos(fisheye_roll * 0.5);
	double sr = sin(fisheye_roll * 0.5);
	double cp = cos(fisheye_pitch * 0.5);
	double sp = sin(fisheye_pitch * 0.5);

	double q_w = cy * cr * cp + sy * sr * sp;
	double q_x = cy * sr * cp - sy * cr * sp;
	double q_y = cy * cr * sp + sy * sr * cp;
	double q_z = sy * cr * cp - cy * sr * sp;


  transformStamped.transform.rotation.x = q_x;  //NED frame
  transformStamped.transform.rotation.y = q_y;  //NED frame
  transformStamped.transform.rotation.z = q_z;  //NED frame
  transformStamped.transform.rotation.w = q_w;  //NED frame

  br.sendTransform(transformStamped);

	//refers to camera frame, rotation321(roll,pitch,yaw)
	rotation321(fisheye_roll,fisheye_pitch,fisheye_yaw,rotation_fisheye);
	// cout<<fisheye_roll*180/3.141592<<" "<<fisheye_pitch*180/3.141592<<" "<<fisheye_yaw*180/3.141592<<endl;

	// Check the angles //
	
	Vec3d eulerAngles_fisheye;
	Mat rotation_fisheye_inv = rotation_fisheye.inv();
	getEulerAngles(rotation_fisheye_inv,eulerAngles_fisheye);
    // cout<<"fisheye frame - roll(CCW)/pitch(Up+)/yaw(CW from north) "<<-eulerAngles_fisheye<<endl;

    geometry_msgs::Twist rotation_f;
    rotation_f.linear.x=fisheye_roll;
    rotation_f.linear.y=fisheye_pitch;
	rotation_f.linear.z=fisheye_yaw;
    rotation_pub.publish(rotation_f);

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
  image_transport::Publisher image_pub_;
  ros::Subscriber pose_sub;
  ros::Publisher p_pub;
  ros::Publisher s_pub;
  ros::Publisher i_pub;
  ros::Publisher rotation_pub;
  ros::Subscriber imu_sub;

	Mat preorig, preorig2, preorig3, preorig4, preorig5, preorig6, preorig7, preorig8, preorig9, preorig10;
	Mat preorig11, preorig12, preorig13, preorig14, preorig15, preorig16, preorig17, preorig18, preorig19, preorig20;
	Mat preorig21, preorig22, preorig23, preorig24, preorig25, preorig26, preorig27, preorig28, preorig29, preorig30;
	Mat preorig31, preorig32, preorig33, preorig34, preorig35, preorig36, preorig37, preorig38, preorig39, preorig40;
	Mat preorig41, preorig42, preorig43, preorig44, preorig45, preorig46, preorig47, preorig48, preorig49, preorig50;
	Mat preorig51, preorig52, preorig53, preorig54, preorig55, preorig56, preorig57, preorig58, preorig59, preorig60;
	Mat K = (Mat_<double>(3, 3) << 445.17984003885283, 0.0, 721.2172155119429, 0.0, 445.8816513187479, 721.9473150432297, 0.0, 0.0, 1.0);
	Mat D = (Mat_<double>(1, 4) << -0.017496068223756347, -0.003243014339251435, 0.000839223410670477, -0.0004219234265229322);

public:
  ImageConverter()
  : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed

        image_transport::TransportHints hints("compressed", ros::TransportHints());
    image_sub_ = it_.subscribe("/omnicam/image_raw", 1,
    &ImageConverter::imageCb, this, hints);
    image_pub_ = it_.advertise("/undistort_fisheye", 1);

    pose_sub = nh_.subscribe("/hexacopter/mavros/local_position/pose", 1000, &ImageConverter::chatterCallback, this);
    p_pub = nh_.advertise<geometry_msgs::Twist>("/rbe_target_point", 10);
    s_pub = nh_.advertise<geometry_msgs::Twist>("/observation_global_point", 10);
    i_pub = nh_.advertise<geometry_msgs::Twist>("/image_frame_point_fisheye", 10);
	rotation_pub = nh_.advertise<geometry_msgs::Twist>("/fisheye_rotation", 10);
    imu_sub = nh_.subscribe("/mavros/imu/data", 1000, &ImageConverter::imuCallback, this);

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

    detection=0;

    bool trackObjects = true;
    bool useMorphOps = true;
	average=100;

	Mat HSV;
	Mat threshold;
	int x=0, y=0;
	int resize_size = 1000;


    cv::Mat src;

    src = cv_ptr->image;

    Mat frame,orig_warped,tmp;

    Tracker tracker;

	Mat flowUmat, flow;
	Point premaxpoint1, premaxpoint2, premaxpoint3, premaxpoint4;
		//pyramid



        cv::Rect myROI(0, 0, 1504, 1504);
        cv::Mat orig(src,myROI);
		//Mat orig;
		Mat out;
		resize(orig, orig, Size(resize_size,resize_size), 0);
		flip(orig,orig,0);

		//fisheye::undistortImage(orig, orig, K, D, K, Size(orig.cols, orig.rows));
		//imshow("original",orig);


		if (preorig40.empty() == false ) {

	    	vector<Point2f> trackedFeatures2;
			Mat_<float>     rigidTransform2;
        	Mat prevGray2;
    	    Mat gray2; Mat gray3; 
			Mat preorig_orig;
        	vector<Point2f> corners2, corners3, graph;
			 //preorig.copyTo(preorig_orig);
			 orig.copyTo(preorig20_orig);
			cvtColor(preorig40,gray2,CV_BGR2GRAY);
			cvtColor(orig,gray3,CV_BGR2GRAY);

			//color detection

			//cvtColor(preorig40,HSV,COLOR_BGR2HSV);
			//inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);
			//if(useMorphOps)
			//	morphOps(threshold);
			//if(trackObjects)1440
			//	trackFilteredO1440
				//imshow(windowName,cameraFeed);


        	if(trackedFeatures2.size() < 5) 
			{
			

				cv::Mat mask = cv::Mat::zeros(gray3.size(), CV_8UC1);  
				double center_size = resize_size/2;
				double circle_radius = resize_size/3.5;


				// Square mask
				//cv::Mat roi(mask, cv::Rect(100,100,900,900));
				//roi = cv::Scalar(255, 255, 255);	

				// Circular mask
				cv::circle(mask,  cv::Point(center_size, center_size), circle_radius, cv::Scalar(255, 255, 255), -1, 8, 0);
            
				goodFeaturesToTrack(gray2,corners2,find_number,0.0001,10,mask);

            	for (int i = 0; i < corners2.size(); ++i) 
				{
                trackedFeatures2.push_back(corners2[i]);
            	}
        	}
			vector<uchar> status_homography;
        	vector<uchar> status2; vector<float> errors2;
			vector<int> outlier;
			outlier.clear();
        	calcOpticalFlowPyrLK(gray2,gray3,trackedFeatures2,corners2,status2,errors2,Size(20,20));
			Mat_<float> new_H = findHomography(trackedFeatures2, corners2, status_homography, CV_RANSAC, INLIER_THRESHOLD);
			for (int i = 0; i < status_homography.size(); ++i)
			{
				if (status_homography[i] == 0)
				{
					outlier.push_back(i);
				}
			}
			opencv_inliers = accumulate(status_homography.begin(), status_homography.end(), 0);
			cout<<"inlinders: "<<opencv_inliers<<endl;

			Point point1, point2, point3, point4;
			Point averagep;
			double maxlen=0;
					
			for (int i=0; i<outlier.size(); i++)
			{
				int array_num = outlier[i];
				circle(preorig20_orig, Point(trackedFeatures2.at(array_num).x,trackedFeatures2.at(array_num).y), 5, Scalar(0, 0, 255), -1);
				circle(preorig20_orig,  Point(corners2.at(array_num).x,corners2.at(array_num).y), 5, Scalar(255, 0, 0), -1);
				line(preorig20_orig, Point(trackedFeatures2.at(array_num).x,trackedFeatures2.at(array_num).y), Point(corners2.at(array_num).x,corners2.at(array_num).y), Scalar(0,255,0), 1);
				
			}

			for (int i = 0; i < status2.size(); ++i) 
			{
				
				circle(preorig20_orig, Point(trackedFeatures2.at(i).x,trackedFeatures2.at(i).y), 2, Scalar(0, 0, 255), -1);
				circle(preorig20_orig,  Point(corners2.at(i).x,corners2.at(i).y), 2, Scalar(255, 0, 0), -1);
				line(preorig20_orig, Point(trackedFeatures2.at(i).x,trackedFeatures2.at(i).y), Point(corners2.at(i).x,corners2.at(i).y), Scalar(0,255,0), 1);
					
				
					// length between points
					double length=sqrt((trackedFeatures2.at(i).x-corners2.at(i).x)*(trackedFeatures2.at(i).x-corners2.at(i).x)+(trackedFeatures2.at(i).y-corners2.at(i).y)*(trackedFeatures2.at(i).y-corners2.at(i).y));
					


					if(length<maximum_length&&length>minimum_length){ // if the length is shorter than my threshold
					if(maxlen<length)
					{maxlen=length;
					maxpoint.x=corners2.at(i).x;
					maxpoint.y=corners2.at(i).y;
					}
					if(point4.x==0){
					if(point3.x==0){
					if(point2.x==0){
					if(point1.x==0){
					point1.x=corners2.at(i).x;
					point1.y=corners2.at(i).y;
					goto out;
					}
					point2.x=corners2.at(i).x;
					point2.y=corners2.at(i).y;
					goto out;
					}
					point3.x=corners2.at(i).x;
					point3.y=corners2.at(i).y;
					goto out;
					}
					point4.x=corners2.at(i).x;
					point4.y=corners2.at(i).y;
					goto out;
					}

					out:;
					}


			}


					
		int targeti=0;
		double minleng=200000;
					
		for(int i=0; i<color.size(); i++)
		{

		double lengg=sqrt((color.at(i).x-maxpoint.x)*(color.at(i).x-maxpoint.x)+(color.at(i).y-maxpoint.y)*(color.at(i).y-maxpoint.y));
		if(minleng>lengg)
		{
		minleng=lengg;
		targeti=i;
		// std::cout<<color.size()<<std::endl;
		}
		}
						
					
		if(color.size()>0)
		{
		//circle(preorig20_orig,Point(color.at(targeti).x,color.at(targeti).y),5,Scalar(255,0,0),-1);
		}
							

		////////find the shortest length

		if(maxlen>minimum_length) // if the length chosen is larger than the threshold
		{
			circle(preorig20_orig,  maxpoint, 15, Scalar(0, 0, 255), 3); // draw
			g_x=maxpoint.x-orig.cols;
			g_y=-maxpoint.y+orig.rows;


			if(maxpoint.y<660){
			detected_time=detected_time+0.195;
			// std::cout<<"detected_time: "<<detected_time<<endl;
			}

			
			if(!premaxpoint4.x==0) // find the distance between consecutive target points and if they are close they are target.
			{
				double lengthpoints=sqrt((premaxpoint1.x-averagep.x)*(premaxpoint1.x-averagep.x)+(premaxpoint1.y-averagep.y)*(premaxpoint1.y-averagep.y));
				double lengthpoints2=sqrt((premaxpoint2.x-premaxpoint1.x)*(premaxpoint2.x-premaxpoint1.x)+(premaxpoint2.y-premaxpoint1.y)*(premaxpoint2.y-premaxpoint1.y));
				double lengthpoints3=sqrt((premaxpoint3.x-premaxpoint2.x)*(premaxpoint3.x-premaxpoint2.x)+(premaxpoint3.y-premaxpoint2.y)*(premaxpoint3.y-premaxpoint2.y));
				double lengthpoints4=sqrt((premaxpoint4.x-premaxpoint3.x)*(premaxpoint4.x-premaxpoint3.x)+(premaxpoint4.y-premaxpoint3.y)*(premaxpoint4.y-premaxpoint3.y));
				double average=(lengthpoints+lengthpoints2+lengthpoints3+lengthpoints4)/4;

				if(average<target_determination)
				{
				//circle(preorig20_orig,  maxpoint, 10, Scalar(0, 0, 255), -1);
				std::cout << maxlen <<  averagep <<" \n";
				}

				premaxpoint4=premaxpoint3;premaxpoint3=premaxpoint2;
				premaxpoint2=premaxpoint1;premaxpoint1=averagep;
			}
			else
			{
				premaxpoint4=premaxpoint3;premaxpoint3=premaxpoint2;
				premaxpoint2=premaxpoint1;premaxpoint1=averagep;
			}	



			
                if(first_detection==0)
                {
                    first_detection=1;
                    g_distance=0;
                }
                else
                {
                    g_x_prev=maxpoint_prev.x-orig.cols;
                    g_y_prev=-maxpoint_prev.y+orig.rows;
                    g_distance=sqrt((g_x-g_x_prev)*(g_x-g_x_prev)+(g_y-g_y_prev)*(g_y-g_y_prev));
                }

				//if(g_distance<1000&&maxpoint.y<650) //long baseline
                if(g_distance<400&&maxpoint.y<720) //short baseline
                {
					v_x=(g_x-g_x_prev)/10;
					v_y=(g_y-g_y_prev)/10;
					T_U = (Mat_<double>(2,1) << v_x, v_y);
                    // cout<<maxpoint<<" "<<maxpoint_prev<<" "<<T_U<<endl;        
                    detection=1;
					maxpoint_prev=maxpoint;
                }
					
				if(first_maxpoint==0)
				{
					maxpoint_prev=maxpoint;
					first_maxpoint=1;						
				}
		// cout<<"gdistance"<<g_distance<<"max"<<maxpoint<<"detection"<<detection<<endl;

		}

		//resize(preorig20_orig, preorig20_orig, Size(FRAME_WIDTH,FRAME_HEIGHT), 0);
		//imshow("thres",threshold);


 		 preorig37.copyTo(preorig36);
		 preorig38.copyTo(preorig37);
		 preorig39.copyTo(preorig38);
		 preorig40.copyTo(preorig39);
		 orig.copyTo(preorig40);

		//imshow("fisheye",preorig20_orig);



        if(first_detection==1)
        {
            rbe(orig);
        }
		imshow("double",preorig20_orig);

		sensor_msgs::ImagePtr msg;
		msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", preorig20_orig).toImageMsg();
      	image_pub_.publish(msg);
	}
	else {

 		 preorig37.copyTo(preorig36);
		 preorig38.copyTo(preorig37);
		 preorig39.copyTo(preorig38);
		 preorig40.copyTo(preorig39);
		 orig.copyTo(preorig40);

	}




	//cv::imshow("src", preorig);    
	//imshow("orig",orig);
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





  void rbe (Mat &field){

	double p_x, p_y;
	struct ocam_model o; // our ocam_models for the fisheye and catadioptric cameras
	get_ocam_model(&o, "Insta360_calib_results.txt");
	
    if(detection==0)
    {
	    //Predict Only
	    T_mxkGkm = T_A*T_mxkmGkm + T_B*T_U;
	    T_SxkGkm = T_A*T_SxkmGkm*T_A_t + T_Swkm;
	    T_mxkGk = T_mxkGkm;
	    T_SxkGk = T_SxkGkm;

		p_x=T_mxkGk.at<double>(0,0) + field.cols;
		p_y=-T_mxkGk.at<double>(1,0) + field.rows;
		putText(preorig20_orig,"Predicting",Point(p_x+10,p_y),2,1,Scalar(0,0,0),2);

    }


    if(detection==1)
    {

	    Mat T_X = (Mat_<double>(2,1) << g_x,g_y);

        //Predict
	    T_mxkGkm = T_A*T_mxkmGkm + T_B*T_U;
	    T_SxkGkm = T_A*T_SxkmGkm*T_A_t + T_Swkm;

	    T_X = T_X + T_B*T_U;
	    ST_Z = T_X;
	
	    //Correction
	    T_Kk = T_SxkGkm*T_C_t/(T_C*T_SxkGkm*T_C_t + T_Svk);
	    T_mxkGk = T_mxkGkm + T_Kk*(ST_Z - T_C*T_mxkGkm);
	    T_SxkGk = (eye - T_Kk*T_C)*T_SxkGkm;
        // cout<<g_distance<<endl;

		p_x=T_mxkGk.at<double>(0,0) + field.cols;
		p_y=-T_mxkGk.at<double>(1,0) + field.rows;
        putText(preorig20_orig,"Correcting",Point(p_x+10,p_y),2,1,Scalar(0,0,0),2);

    }




    //cout<<T_U<<endl;

	cv::circle(preorig20_orig,Point(p_x,p_y),10,Scalar(0,255,0),-1);
  	double point3D[3], point2D[2];                              // the image point in pixel coordinates  
	point2D[0] = p_x;
    point2D[1] = p_y;
  	cam2world(point3D, point2D, &o); 

	vector<Point2f> p_point;
	vector<Point2f> out_point;

	p_point.push_back(Point(p_x,p_y));

	//fisheye::undistortPoints(p_point, out_point, K, D, K);

	//cout<<p_point<<"dadfsdafsdafsdaf"<<out_point<<endl;

    geometry_msgs::Twist i;
    i.linear.x=point3D[0];
    i.linear.y=point3D[1];
	i.linear.z=point3D[2];
	// i.linear.x=out_point[0].x;
    // i.linear.y=out_point[0].y;
    i_pub.publish(i);

	T_mxkmGkm = T_mxkGk;
	T_SxkmGkm = T_SxkGk;



  }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter_normal");
  ImageConverter ic;
  
  ros::spin();
  return 0;
}
