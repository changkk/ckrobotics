#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "gazebo_msgs/ModelStates.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <sstream>
#include "governor/target_list.h"

float x[23] = {0}, y[23] = {0};
//char name[23]

void gazebo_targ_locs(const gazebo_msgs::ModelStates msg)
{
	//ROS_INFO("Robot at : x = [%f], y = [%f], z = [%f] ", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
	//x = msg.x;
	//y = msg.y;
	for(int i = 4; i <= 27; i++){
		x[i-4] = msg.pose[i].position.x;
		y[i-4] = msg.pose[i].position.y;
//		std::cout << "wt";

//		name[i-4] = msg->name[i];
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "target_list");

	ros::NodeHandle n;

	ros::Subscriber sub1 = n.subscribe("/gazebo/model_states", 1, gazebo_targ_locs);
	
	ros::Publisher pub = n.advertise<governor::target_list>("/governor/tlist", 100);

	ros::Rate loop_rate(0.2);

	float tx[10], ty[10], tpoints[10];


	for(int i=0; i<23; i++){ 
//		tx[i] = -rand() % 61;
//		ty[i] = rand() % 101;
		tpoints[i] = rand() % 10 + 1;
		
		std::cout << "what";

	} 




	while (ros::ok())
	{
		governor::target_list msg;

		for (int n=0; n<23; n++){
			msg.Target_Type.push_back(std::string("Static"));
			msg.Served.push_back(0);
//			msg.x.push_back(tx[n]);
//			msg.y.push_back(ty[n]);
			msg.x.push_back(x[n]);
			msg.y.push_back(y[n]);
			msg.Points.push_back(tpoints[n]);
		}

		pub.publish(msg);

		//ROS_INFO("Target Database Refreshed");

		ros::spinOnce();
		loop_rate.sleep();


	}


	return 0;
}
