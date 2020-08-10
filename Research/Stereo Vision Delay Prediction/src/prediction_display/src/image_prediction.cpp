#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Float64.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
//To use clock
#include "rosgraph_msgs/Clock.h"
//To introduce delay
#include "message_filters/time_sequencer.h"
#include "message_filters/subscriber.h"

//To use omnicam calib
#include "ocam_functions.h"
//To use OpenCV
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"
#include <sensor_msgs/image_encodings.h>

//#include <opencv2/video/tracking.hpp>

using namespace cv;
using namespace std;
using std::vector;//For class Tracker

Mat K = (Mat_<double>(3, 3) <<  984003885283, 0.0, 721.2172155119429, 0.0, 445.8816513187479, 721.9473150432297, 0.0, 0.0, 1.0);
Mat D = (Mat_<double>(1, 4) << -0.017496068223756347, -0.003243014339251435, 0.000839223410670477, -0.0004219234265229322);
Mat H = Mat::eye(3,3,CV_32FC1);
double yaw_save, pitch_save, roll_save;
int time_stamp=0;
double init_time;
double curr_time;
double prev_time;
double delta_t;
// Test 1 2 3



double half_hfov, half_vfov;
double lt_px, lt_py, rt_px, rt_py, lb_px, lb_py, rb_px, rb_py;

// Parameters to change
double delay = .5; //Specify the constant delay in secs.
double range = 10;
double omni_hfov = 170*3.141592/180;
double omni_vfov = 170*3.141592/180;
double yaw_offset = 10*3.141502/180;

//Parameter for calculating the Homography matrix
bool freshStart = true;
vector<Point2f> trackedFeatures1;
Mat gray1; Mat gray2;
vector<Point2f> corners1;
Mat prevGray1; Mat prevGray2;
vector<uchar> status;
vector<float> errors;
const double INLIER_THRESHOLD = 1.0; // pixel distance
int opencv_inliers;
int first=0;
double angle_first=0;
Mat orig_show_ptz;
Mat cropped_image;
Mat orig_show;
vector<int> outlier;


//Tracker class required for Homography matrix
class Tracker {

public:


    void processImage(Mat& img1, Mat& img2) {
        cout <<"Tracker" << endl;
	cout << orig_show_ptz.size << ", " << cropped_image.size() << endl;
        cvtColor(img1,gray1,CV_BGR2GRAY);
        cvtColor(img2,gray2,CV_BGR2GRAY);
	cout <<"cvtColor" << endl;
        if(freshStart==true){
            H = Mat::eye(3,3,CV_32FC1); //affine 2x3 in a 3x3 matrix
	    
        }

        if(trackedFeatures1.empty()) {
            goodFeaturesToTrack(gray1,corners1,300,0.01,10);
            for (int i = 0; i < corners1.size(); ++i) {
                trackedFeatures1.push_back(corners1[i]);
            }
        }

        if(!prevGray1.empty() && !prevGray2.empty()) {
            calcOpticalFlowPyrLK(prevGray1,prevGray2,trackedFeatures1,corners1,status,errors,Size(25,25));

            if(first==0)
            {
                first=1;
            }

            if(countNonZero(status) < status.size() * 0.6) {
                cout << "cataclysmic error \n";
                angle_first=0;
                H = Mat::eye(3,3,CV_32FC1);
                trackedFeatures1.clear();
                prevGray1.release();
                prevGray2.release();
                freshStart = true;
                return;
            } else
                freshStart = false;

            vector<uchar> status_homography;

            if(freshStart==false)
            {
                Mat_<float> new_H = findHomography(trackedFeatures1, corners1, status_homography, CV_RANSAC, INLIER_THRESHOLD);
                //opencv_inliers = accumulate(status_homography.begin(), status_homography.end(), 0);
				for (int i = 0; i < status_homography.size(); ++i)
				{
					if (status_homography[i] == 1)
					{
						outlier.push_back(i);
					}
				}
                H = new_H * H;
            }


            trackedFeatures1.clear();
            for (int i = 0; i < status.size(); ++i) {
                if(status[i]) {
                    trackedFeatures1.push_back(corners1[i]);
                }
            }
        }

        gray1.copyTo(prevGray1);
        gray2.copyTo(prevGray2);
    }
};

void poseCallback(const geometry_msgs::PoseStamped& msg){


	///////////// Quaternion to Euler ///////////////////////
	double q_x = msg.pose.orientation.x;
	double q_y = msg.pose.orientation.y;
	double q_z = msg.pose.orientation.z;
	double q_w = msg.pose.orientation.w;
	double pitch, roll;

	// pitch (x-axis rotation)
	double sinr_cosp = +2.0 * (q_w * q_x + q_y * q_z);
	double cosr_cosp = +1.0 - 2.0 * (q_x * q_x + q_y * q_y);
	pitch = atan2(sinr_cosp, cosr_cosp);

	// roll (z-axis rotation)
	double sinp = +2.0 * (q_w * q_y - q_z * q_x);
	if (fabs(sinp) >= 1)
		roll = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		roll = asin(sinp);

	// yaw (y-axis rotation)
	double siny_cosp = +2.0 * (q_w * q_z + q_x * q_y);
	double cosy_cosp = +1.0 - 2.0 * (q_y * q_y + q_z * q_z);
	double yaw = atan2(siny_cosp, cosy_cosp)+0.1;

	double pitch_top = pitch + half_vfov;
	double pitch_bot = pitch - half_vfov;
	double yaw_right = yaw - half_hfov;
	double yaw_left = yaw + half_hfov;

	/*if(time_stamp==10*)
	{
		yaw_save = yaw;
	}

	time_stamp++;
	double yaw_rate= (yaw - yaw_save) / 10;*/

	//////////////////////////////////////////////////////////////

	// Rotate -90 deg
	double c_x = range*cos(pitch)*sin(yaw);
	double c_y = -range*cos(pitch)*cos(yaw);
	double c_z = range*sin(pitch);
	double c_r = sqrt(c_x*c_x + c_y*c_y + c_z*c_z);
	double c_yaw = atan2(c_y,c_x)-yaw_offset;
	double c_pitch = asin(c_z/c_r);

	double lt_x = range*cos(pitch_top)*sin(yaw_left);
	double lt_y = -range*cos(pitch_top)*cos(yaw_left);
	double lt_z = range*sin(pitch_top);
	double lt_r = sqrt(lt_x*lt_x + lt_y*lt_y + lt_z*lt_z);
	double lt_yaw = atan2(lt_y,lt_x)-yaw_offset;
	double lt_pitch = asin(lt_z/lt_r);
	lt_px = 1440 - 1440*(lt_yaw/omni_hfov);
	lt_py = 720 - 1440*(lt_pitch/omni_vfov);

	double rt_x = range*cos(pitch_top)*sin(yaw_right);
	double rt_y = -range*cos(pitch_top)*cos(yaw_right);
	double rt_z = range*sin(pitch_top);
	double rt_r = sqrt(rt_x*rt_x + rt_y*rt_y + rt_z*rt_z);
	double rt_yaw = atan2(rt_y,rt_x)-yaw_offset;
	double rt_pitch = asin(rt_z/rt_r);
	rt_px = 1440 - 1440*(rt_yaw/omni_hfov);
	rt_py = 720 - 1440*(rt_pitch/omni_vfov);

	double lb_x = range*cos(pitch_bot)*sin(yaw_left);
	double lb_y = -range*cos(pitch_bot)*cos(yaw_left);
	double lb_z = range*sin(pitch_bot);
	double lb_r = sqrt(lb_x*lb_x + lb_y*lb_y + lb_z*lb_z);
	double lb_yaw = atan2(lb_y,lb_x)-yaw_offset;
	double lb_pitch = asin(lb_z/lb_r);
	lb_px = 1440 - 1440*(lb_yaw/omni_hfov);
	lb_py = 720 - 1440*(lb_pitch/omni_vfov);

	double rb_x = range*cos(pitch_bot)*sin(yaw_right);
	double rb_y = -range*cos(pitch_bot)*cos(yaw_right);
	double rb_z = range*sin(pitch_bot);
	double rb_r = sqrt(rb_x*rb_x + rb_y*rb_y + rb_z*rb_z);
	double rb_yaw = atan2(c_y,rb_x)-yaw_offset;
	double rb_pitch = asin(rb_z/rb_r);
	rb_px = 1440 - 1440*(rb_yaw/omni_hfov);
	rb_py = 720 - 1440*(rb_pitch/omni_vfov);

}

void zoom_callback(const std_msgs::Float64::ConstPtr& msg){

	double zoom=msg->data;
	double focal_len = 4.8 + (57.6-4.8)*zoom/100;

	half_hfov = atan(2.4/focal_len);
	half_vfov = atan(1.8/focal_len);

}

void clock_callback(const rosgraph_msgs::Clock::ConstPtr& msg){

	double begin = ros::Time::now().toSec();
	if (begin != 0){
		if (time_stamp == 0){
			init_time = ros::Time::now().toSec();
			prev_time = init_time;
			time_stamp++;
			std::cout << "Working: " << std::endl;
		}
		else {
			curr_time = ros::Time::now().toSec();
			delta_t = curr_time - prev_time;
			cout << delta_t << endl;
			prev_time = curr_time;
		}
	}
	

}

void imageCb_omni(const sensor_msgs::ImageConstPtr& msg){
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

	Mat src_omni = cv_ptr->image;

    	// Crop image
	Rect myROI = Rect(0, 0, 1504, 1504);
	Mat orig(src_omni,myROI);

	// Resize image
    resize(orig, orig, Size(1440,1440), 0);

	// Undistort image
    fisheye::undistortImage(orig, orig, K, D, K, Size(orig.cols, orig.rows));

	// Flip image
	flip(orig,orig,-1);


	////////////////////// Polygon ROI////////////////////////////////////
	Mat black(orig.rows, orig.cols, orig.type(), cv::Scalar::all(255));
	Mat orig_copy;
	orig.copyTo(orig_copy);
	Mat mask(orig.rows, orig.cols, CV_8UC1, cv::Scalar(0));

	Point p1(lt_px,lt_py);
	Point p2(rt_px,rt_py);
	Point p3(rb_px,rb_py);
	Point p4(lb_px,lb_py);


	vector< vector<Point> >  co_ordinates;
	co_ordinates.push_back(vector<Point>());

	co_ordinates[0].push_back(p1);
	co_ordinates[0].push_back(p2);
	co_ordinates[0].push_back(p3);
	co_ordinates[0].push_back(p4);
	drawContours(orig_copy,co_ordinates,0, Scalar(255), 1, 4 );

	Mat co_ordinates1 = (Mat_<float>(4, 2) <<  lt_px, lt_py, rt_px, rt_py, rb_px, rb_py, lb_px, lb_py);

	Mat crop_pts = (Mat_<float>(4, 2) <<  0.0, 0.0, 500.0, 0.0, 500.0, 500.0, 0.0, 500.0);
    //Cropping the image
    Mat orig_crop;
    orig_copy.copyTo(orig_crop);
    Rect croppedRectangle = Rect(lt_px, lt_py, 500, 500);
    Mat crop_image = orig_crop(croppedRectangle);
    //Reassigning co_ordinates based on the new cropped image [Because (lt_px, lt_py) now becomes (0, 0)]
    co_ordinates1 = (Mat_<float>(4, 2) <<  lt_px-lt_px, lt_py-lt_py, rt_px-lt_px, rt_py-lt_py, rb_px-lt_px, rb_py-lt_py, lb_px-lt_px, lb_py-lt_py);

    //Getting the Perspective Transformation matrix
    Mat Pers_mat = getPerspectiveTransform(co_ordinates1, crop_pts);


    //Doing Perspective Transformation
    
    warpPerspective(crop_image, cropped_image, Pers_mat, crop_image.size());
	cout<<"outlier: "<<outlier.size()<<endl;
	//cout<<"trackpoints: "<<trackedFeatures1[1].x<<endl;
	for (int i=0; i<outlier.size(); i++){
		int array_num = outlier[i];
		circle(cropped_image, Point(trackedFeatures1.at(array_num).x,trackedFeatures1.at(array_num).y), 5, Scalar(0, 0, 255), -1);
		circle(cropped_image,  Point(corners1.at(array_num).x,corners1.at(array_num).y), 5, Scalar(255, 0, 0), -1);
	}
    imshow("Cropped Image", cropped_image);
    cout << cropped_image.size() <<endl;
    cv::waitKey(1);

	orig_copy.copyTo(black);
	Mat black_show;
	resize(black,black_show,Size(500,500));
	imshow("mask",black_show);
	cv::waitKey(1);

	/////////////////////////////////////////////////////////////////////////////////////

	// To show image
	//Mat orig_show;
    resize(orig, orig_show, Size(500,500), 0);

	imshow("image_omni",orig_show);
	cv::waitKey(1);
	//double begin = ros::Time::now().toSec();
	//cout << std::setprecision(15) << begin << endl;
}

void imageCb_ptz(const sensor_msgs::ImageConstPtr& msg){

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

    	Mat src_ptz = cv_ptr->image;
	// To show image
    	resize(src_ptz, orig_show_ptz, Size(500,500), 0);

	imshow("image_ptz",orig_show_ptz);
	cv::waitKey(1);

    // Find corner -> use optical flow -> compute Homography matrix
    Tracker tracker;
    tracker.processImage(cropped_image, orig_show_ptz);

}

/*
void imagePredict_ptz(const sensor_msgs::ImageConstPtr& msg){

	// To show image
	Mat orig_show;
    	resize(src_ptz, orig_show, src_ptz.size()/5, 0);

	imshow("image_ptz",orig_show);
	cv::waitKey(1);
}
*/

void delayCb(const boost::shared_ptr<sensor_msgs::Image> msg){

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

	//cv::Mat src_ptz;
    	Mat src_ptz = cv_ptr->image;

	// To show image
	Mat orig_show;
    	resize(src_ptz, orig_show, src_ptz.size()/5, 0);

	imshow("Delayed image_ptz",orig_show);
	cv::waitKey(1);
}

int main(int argc, char** argv){

	ros::init(argc,argv,"image_prediction");
    	ros::NodeHandle node;

    // Image subscriber should use different node name and subscriber.

	image_transport::ImageTransport it_(node);
	image_transport::TransportHints hints("compressed", ros::TransportHints()); // To interpret compressed video
    	image_transport::Subscriber image_sub_omni = it_.subscribe("/omnicam/image_raw", 1, &imageCb_omni, hints);
    	image_transport::Subscriber image_sub_ptz = it_.subscribe("/camera/image_raw", 1, &imageCb_ptz, hints);
	//image_transport::Subscriber image_predictor = it_.subscribe("/camera/image_raw", 1, &imagePredict_ptz, hints);

	//To add delays
	message_filters::Subscriber<sensor_msgs::Image> sub_delay_ptz(node, "/camera/image_decompressed", 1);
	message_filters::TimeSequencer<sensor_msgs::Image> seq_delay_ptz(sub_delay_ptz, ros::Duration(5), ros::Duration(.01), 1000);
	seq_delay_ptz.registerCallback(delayCb);


	// Normal ROS subscriber
	ros::Subscriber sub = node.subscribe("/gimbal_imu_angles", 10, &poseCallback);
    	ros::Subscriber zoom_sub = node.subscribe("/zoom", 10, &zoom_callback);

	//To get the initial time
	//ros::Subscriber clock_sub = node.subscribe("/clock", 10, &clock_callback);

	while(ros::ok())
	{

		ros::spinOnce();


	}

	return 0;
}
