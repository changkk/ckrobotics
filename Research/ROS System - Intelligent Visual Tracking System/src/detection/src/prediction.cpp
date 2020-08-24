#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Float64.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/video.hpp>
#include <cmath>
#include <opencv2/video/tracking.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
// #include "opencv2/cudaoptflow.hpp"
// #include "opencv2/cudaarithm.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>	
#include <visualization_msgs/Marker.h>
#include <vector>

using namespace cv;
using namespace std;
ros::Publisher marker_pub;
ros::Publisher marker_line_pub;
visualization_msgs::Marker marker, line_strip;

// Create Kalman Filter
KalmanFilter KF(6,6,0);
Mat_<float> measurement(6,1); 

int target_detected = 0;
double target_x, target_y, target_z;
double pre_x=0, pre_y=0, pre_z=0, pre_vx=0, pre_vy=0, pre_vz=0;
vector<double> vx,vy,vz, ax,ay,az;
double Vx=0,Vy=0,Vz=0, Ax=0, Ay=0, Az=0;
Mat estimated;

// parameters
int past_average_time_horizon = 30;
int prediction_time_horizon = 90;

class Conversion
{

	public: 


	void target_cb(const geometry_msgs::Twist& msg)	
	{

		target_detected++;

		double x = msg.linear.x; // E
		double y = msg.linear.y; // N
		double z = msg.linear.z; // U

		measurement(0) = x;
		measurement(1) = y; 
		measurement(2) = z;
 

		// KF correct
		estimated = KF.correct(measurement);
		target_x = estimated.at<float>(0);
		target_y = estimated.at<float>(1);
		target_z = estimated.at<float>(2);

		// Save target velocity history for specific time in the past
		if(pre_x != 0)
		{
			measurement(3) = x- pre_x;
			measurement(4) = y- pre_y; 
			measurement(5) = z- pre_z; 	
			Vx = estimated.at<float>(3); Vy = estimated.at<float>(4); Vz = estimated.at<float>(5);

			// vx.push_back(target_x - pre_x);
			// vy.push_back(target_y - pre_y);
			// vz.push_back(target_x - pre_z);
			
			// // Averaging velocity
			// if(vx.size() > past_average_time_horizon)
			// {
			// 	vx.erase(vx.begin());
			// 	vy.erase(vy.begin());
			// 	vz.erase(vz.begin());

			// 	for(int i=0; i<past_average_time_horizon; i++)
			// 	{
			// 		Vx += vx[i];
			// 		Vy += vy[i];
			// 		Vz += vz[i];
			// 	}
				
			// 	Vx = Vx/vx.size(); Vy = Vy/vy.size(); Vz = Vz/vz.size();
			// }

		}


		// Averaging acceleration
		if(pre_vx != 0)
		{
			ax.push_back(Vx - pre_vx);
			ay.push_back(Vy - pre_vy);
			az.push_back(Vz - pre_vz);	

			if(ax.size() > past_average_time_horizon)
			{
				ax.erase(ax.begin());
				ay.erase(ay.begin());
				az.erase(az.begin());

				for(int i=0; i<past_average_time_horizon; i++)
				{
					Ax += ax[i];
					Ay += ay[i];
					Az += az[i];
				}
				
				Ax = Ax/ax.size(); Ay = Ay/ay.size(); Az = Az/az.size();

			}

		}


		pre_x = x; pre_y = y; pre_z = z;
		pre_vx = Vx; pre_vy = Vy; pre_vz = Vz;


		// Draw marker
		marker.header.frame_id = "observer_aircraft";
		marker.header.stamp = ros::Time::now();
		marker.ns = "basic_shapes";
		marker.id = 6;

		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;

		marker.pose.position.x = target_x;  
		marker.pose.position.y = target_y;  
		marker.pose.position.z = target_z;  
		marker.pose.orientation.w = 1.0;

		marker.scale.x = 1;
		marker.scale.y = 1;
		marker.scale.z = 1;

		marker.color.r = 1.0f;
		marker.color.g = 0.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;

		marker_pub.publish(marker);


	}


	private:

		ros::NodeHandle nh_;
		ros::Subscriber target_sub;


	public:
		Conversion()
		:nh_()
		{
			target_sub = nh_.subscribe("/target_epipolar", 1000, &Conversion::target_cb, this);
			marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker_target2", 1);
			marker_line_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker_line", 1);

		}
		~Conversion()
		{

		}


};

int main(int argc, char** argv){

	ros::init(argc,argv,"quat_to_euler");
 	Conversion convert;
	ros::NodeHandle nh("/");
	using namespace cv;
	ros::Rate loop_rate(30); // 30Hz

	// Kalman Filter
	KF.transitionMatrix = (Mat_<float>(6, 6) << 1,0,0,1,0,0,   0,1,0,0,1,0,  0,0,1,0,0,1,  0,0,0,1,0,0,  0,0,0,0,1,0,  0,0,0,0,0,1);
	KF.statePre.at<float>(0) = 0;
	KF.statePre.at<float>(1) = 0;
	KF.statePre.at<float>(2) = 0;
	KF.statePre.at<float>(3) = 1;
	KF.statePre.at<float>(4) = 1;
	KF.statePre.at<float>(5) = 1;

	// Set KF noise params
	setIdentity(KF.measurementMatrix);
	setIdentity(KF.processNoiseCov, Scalar::all(1e-3));
	setIdentity(KF.measurementNoiseCov, Scalar::all(10));
	setIdentity(KF.errorCovPost, Scalar::all(.005));


	while (ros::ok()) // 30Hz loop
	{

		estimated = KF.predict();
		target_x = estimated.at<float>(0);
		target_y = estimated.at<float>(1);
		target_z = estimated.at<float>(2);	

		Mat X = (Mat_<double>(6, 1) << estimated.at<float>(0), estimated.at<float>(1), estimated.at<float>(2),Vx, Vy, Vz);
		Mat Xdot = (Mat_<double>(6, 1) << Vx, Vy, Vz, Ax, Ay, Az);
		Mat Xnew;

		Xnew.release();
    	
		line_strip.points.clear();

		// Prediction
		for(int i=0; i<prediction_time_horizon; i++)
		{
			Xnew = X + Xdot;
			geometry_msgs::Point p;
			p.x = Xnew.at<double>(0,0);
			p.y = Xnew.at<double>(0,1);
			p.z = Xnew.at<double>(0,2);
			line_strip.points.push_back(p);
			X = Xnew;
			Xdot = (Mat_<double>(6, 1) << Xnew.at<double>(0,3), Xnew.at<double>(0,4), Xnew.at<double>(0,5), Ax, Ay, Az);	
		}

		// line_strip marker
		line_strip.header.frame_id = "observer_aircraft";
		line_strip.header.stamp = ros::Time::now();
		line_strip.ns = "lines";
		line_strip.id = 7;

		line_strip.type = visualization_msgs::Marker::LINE_STRIP;
		line_strip.action = visualization_msgs::Marker::ADD;

		line_strip.pose.orientation.w = 1.0;
		line_strip.scale.x = 0.1;
		line_strip.color.r = 0.0f;
		line_strip.color.g = 0.0f;
		line_strip.color.b = 1.0f;
		line_strip.color.a = 1.0;
		marker_line_pub.publish(line_strip);

		// ROS spin
		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0;
}
