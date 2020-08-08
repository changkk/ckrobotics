#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <string>
#include <vector>
#include "std_msgs/String.h"
#include "governor/target_list.h"
#include "governor/assignment.h"
#include <iostream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include "hungarian.h"
#include <complex>

//{ x pos | y pos | Target type | Served Bool | Points}

double x[3] = {0,0,0}, y[3] = {0,0,0}, z[3] = {0,0,0};
double tx[23], ty[23], tpoints[23];
double model_pose[3], model_velo[3];
int served[23];
std::vector<std::__cxx11::basic_string<char> > model_name[7];
double d_x_prev[3], d_y_prev[3], d_z_prev[3], d_x[3] = {0}, d_y[3] = {0}, d_z[3] = {0}, t_prev;
double t = 0, d_time, los_len[3] = {0}, los_len_diff[3], los_len_old[3], tim_clr_old[3], time_clr[3];
double lim = 4;
stringstream ss (stringstream::in | stringstream::out);
// ros::Time time_prev;
// ros::Time timeN = ros::Time::now();
// ros::Time elapsed_time;

void tlistCallback(const governor::target_list::ConstPtr&  msg)
{
	for(int i = 0; i < 23; i++){
		tx[i] = msg->x[i];
		ty[i] = msg->y[i];
		tpoints[i] = msg->Points[i];
		served[i] = msg->Served[i];
	}
}



void pose0Callback(const nav_msgs::Odometry msg)
{
	//ROS_INFO("Robot at : x = [%f], y = [%f], z = [%f] ", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
	//x = msg.x;
	//y = msg.y;
	x[0] = msg.pose.pose.position.x;
	y[0] = msg.pose.pose.position.y;
	z[0] = msg.pose.pose.position.z;
}
void pose1Callback(const nav_msgs::Odometry msg)
{
	//ROS_INFO("Robot at : x = [%f], y = [%f], z = [%f] ", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
	//x = msg.x;
	//y = msg.y;
	x[1] = msg.pose.pose.position.x;
	y[1] = msg.pose.pose.position.y;
	z[1] = msg.pose.pose.position.z;
}
void pose2Callback(const nav_msgs::Odometry msg)
{
	//ROS_INFO("Robot at : x = [%f], y = [%f], z = [%f] ", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
	//x = msg.x;
	//y = msg.y;
	x[2] = msg.pose.pose.position.x;
	y[2] = msg.pose.pose.position.y;
	z[2] = msg.pose.pose.position.z;
}





int main(int argc, char **argv)
{

	ros::init(argc, argv, "collision_avoidance");

	ros::NodeHandle n;

	ros::Subscriber sub1 = n.subscribe("/governor/tlist", 1, tlistCallback);

	//ros::Subscriber sub2 = n.subscribe("/mavros/battery", 100, batteryCallback);


	ros::Subscriber sub3 = n.subscribe("/hexacopter0/localizer/global_odom", 1, pose0Callback);
	ros::Subscriber sub4 = n.subscribe("/hexacopter1/localizer/global_odom", 1, pose1Callback);
	ros::Subscriber sub5 = n.subscribe("/hexacopter2/localizer/global_odom", 1, pose2Callback);

	// ros::Subscriber sub7 = n.subscribe("/hexacopter0/uav_control/state", 1, uavControlState0Callback);
	// ros::Subscriber sub8 = n.subscribe("/hexacopter1/uav_control/state", 1, uavControlState1Callback);
	// ros::Subscriber sub9 = n.subscribe("/hexacopter2/uav_control/state", 1, uavControlState2Callback);
	
	//SUBSCRIBE TO 
	
	
	
	ros::Publisher pub = n.advertise<governor::assignment>("assignment", 100);
	
	ros::Publisher pubhex0 = n.advertise<geometry_msgs::Pose>("/hexacopter0/uav_control/waypoint", 100);
	ros::Publisher pubhex1 = n.advertise<geometry_msgs::Pose>("/hexacopter1/uav_control/waypoint", 100);
	ros::Publisher pubhex2 = n.advertise<geometry_msgs::Pose>("/hexacopter2/uav_control/waypoint", 100);
	ros::Publisher pubOCS = n.advertise<std_msgs::String>("/ocs/logmessage", 100);
	
	//ros::spin();

	ros::Rate loop_rate(20);

	//Number of Active UAV's
	int activeUAV = 3; 	
	
	//Dropzone Location in global (arena) frame (Reconfirm)
	int dzx = 0, dzy = -25.45;

	std_msgs::String ocsmsg;
	
	geometry_msgs::Twist allStop;
	geometry_msgs::Pose hldpt;
	bool flag;

	while(ros::ok()){

			//Save previous values of line of sight components
			for (int i = 0 ; i < 3 ; i++)
			{

				d_x_prev[i] = d_x[i];
				d_y_prev[i] = d_y[i];
				d_z_prev[i] = d_z[i];
			}


			//Calculate current LOS components for each pair of UAVS, 0-1, 0-2, 1-2
			d_x[0] = std::abs(x[0] - x[1]);
			d_y[0] = std::abs(y[0] - y[1]);
			d_z[0] = std::abs(z[0] - z[1]);

			d_x[1] = std::abs(x[0] - x[2]);
			d_y[1] = std::abs(y[0] - y[2]);
			d_z[1] = std::abs(z[0] - z[2]);

			d_x[2] = std::abs(x[1] - x[2]);
			d_y[2] = std::abs(y[1] - y[2]);
			d_z[2] = std::abs(z[1] - z[2]);

			//Get time elapsed since last loop
			t_prev = t;
			t = ros::Time::now().toSec();
			d_time = t - t_prev;

			// time_prev = timeN;
			// timeN = ros::Time::now();
			// elapsed_time = timeN - time_prev;

			los_len_old[0] = los_len[0];
			los_len_old[1] = los_len[1];
			los_len_old[2] = los_len[2];

			los_len[0] = std::pow((std::pow(d_x[0],2) + std::pow(d_y[0],2) + std::pow(d_z[0],2)),0.5);
			los_len[1] = std::pow((std::pow(d_x[1],2) + std::pow(d_y[1],2) + std::pow(d_z[1],2)),0.5);
			los_len[2] = std::pow((std::pow(d_x[2],2) + std::pow(d_y[2],2) + std::pow(d_z[2],2)),0.5);
			
			
			

			los_len_diff[0] = (los_len[0] - los_len_old[0])/d_time;
			los_len_diff[1] = (los_len[1] - los_len_old[1])/d_time;
			los_len_diff[2] = (los_len[2] - los_len_old[2])/d_time;

			std::cout << los_len[0] << "	" << los_len[1] << "	" << los_len[2] << "		" << los_len_diff[0] << "	" << los_len_diff[1] << "	" << los_len_diff[2] << std::endl;

			for (int i = 0 ; i < 3 ; i++)
			{
				allStop.linear.x = 0;
				allStop.linear.y = 0;
				allStop.linear.z = 0;
				
				if (los_len_diff[i] <= 0)
				{
					tim_clr_old[i] = time_clr[i];
					time_clr[i] = los_len[i]/los_len_diff[i];

					//Add time clearance Logic

					if (d_x[i] <= lim && d_y[i] <= lim && d_z[i] <= lim)
					{
						


						//Check priority
						flag == true;
						
						switch (i)
						{
							case 0: ss.str(std::string());
									ss << "[Collision Detector] Collision imminent b/w Hex 0 and 1! Stopping UAV's...";
									ocsmsg.data = ss.str();
									pubOCS.publish(ocsmsg);
									
									hldpt.position.x = x[0];
									hldpt.position.y = y[0];
									hldpt.position.z = z[0];
									pubhex0.publish(hldpt);
									

									hldpt.position.x = x[1];
									hldpt.position.y = y[1];
									hldpt.position.z = z[1];
									pubhex1.publish(hldpt);

						 	case 1:	ss.str(std::string());
									ss << "[Collision Detector] Collision imminent b/w Hex 0 and 2! Stopping UAV's...";
									ocsmsg.data = ss.str();
									pubOCS.publish(ocsmsg);
							 
							 		hldpt.position.x = x[0];
									hldpt.position.y = y[0];
									hldpt.position.z = z[0];
									pubhex0.publish(hldpt);
									

									hldpt.position.x = x[2];
									hldpt.position.y = y[2];
									hldpt.position.z = z[2];
									pubhex2.publish(hldpt);

						 	case 2: ss.str(std::string());
									ss << "[Collision Detector] Collision imminent b/w Hex 1 and 2! Stopping UAV's...";
									ocsmsg.data = ss.str();
									pubOCS.publish(ocsmsg);
							 
							 		hldpt.position.x = x[1];
									hldpt.position.y = y[1];
									hldpt.position.z = z[1];
									pubhex1.publish(hldpt);
									

									hldpt.position.x = x[2];
									hldpt.position.y = y[2];
									hldpt.position.z = z[2];
									pubhex2.publish(hldpt);
						}

					}

					if (flag == true && (d_x[i] >= lim || d_y[i] >= lim || d_z[i] >= lim))
					{
						flag = false;
					}

				}
				
			}


		// 	//Calculate LOS derivatives (rate of closure between a pair of UAV's')

		// 	for (int i = 0 ; i < 3; i++)
		// 	{
		// 		v_x[i] = (d_x[i] - d_x_prev[i])/d_t;
		// 		v_y[i] = (d_y[i] - d_y_prev[i])/d_t;
		// 		v_z[i] = (d_y[i] - d_z_prev[i])/d_t;
		// 	}

		// 	//Collision Avoidance Logic

		// 	if ( v_x < 0 )

		// }
		
// //------------------------------------------------------------------------------------------------------------------------------------		
// //		Publish Everything
		
		
		
// 		governor::assignment assignMsg;
// 		assignMsg.UAV1.push_back(vassign[0]);
// 		assignMsg.UAV2.push_back(vassign[1]);
// 		assignMsg.UAV3.push_back(vassign[2]);
// 		pub.publish(assignMsg);

// 		geometry_msgs::Pose UAV0set;
// 		UAV0set.position.x = tx[vassign[0]];
// 		UAV0set.position.y = ty[vassign[0]];
// 		UAV0set.position.z = 3;
// 		UAV0set.orientation.w = 1;
// 		pubhex0.publish(UAV0set);
		
// 		geometry_msgs::Pose UAV1set;
// 		UAV1set.position.x = tx[vassign[1]];
// 		UAV1set.position.y = ty[vassign[1]];
// 		UAV1set.position.z = 5;
// 		UAV1set.orientation.w = 1;
// 		pubhex1.publish(UAV1set);
		
// 		geometry_msgs::Pose UAV2set;
// 		UAV2set.position.x = tx[vassign[2]];
// 		UAV2set.position.y = ty[vassign[2]];
// 		UAV2set.position.z = 7;
// 		UAV2set.orientation.w = 1;
// 		pubhex2.publish(UAV2set);
		
		// loopcount++;
		ros::spinOnce();   	    
		loop_rate.sleep();
		

	}


	return 0;
}







