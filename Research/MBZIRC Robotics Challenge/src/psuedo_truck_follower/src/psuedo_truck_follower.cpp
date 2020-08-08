#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <sstream>
#include <string>
#include "gazebo_msgs/ModelStates.h"

float truck_x = 0, truck_y = 0, truck_z = 0;
bool odom_received = false;
// int c = 20;

// void gazebo_truck_locs(const gazebo_msgs::ModelStates msg)
// {
//     if (c == 20)
// 	{
// 		for (int i = 0 ; i < 8 ; i++)
// 		{
// 			if (msg.name[i] == "truck")
// 			{
// 				c = i;
// 			}
// 		}
// 	}

// 	truck_x = msg.pose[c].position.x;
//     truck_y = msg.pose[c].position.y;
// }

void truckPositionCallback(const nav_msgs::Odometry rtk_odom_msg)
{
    odom_received = true;
    truck_x = rtk_odom_msg.pose.pose.position.x;
    truck_y = rtk_odom_msg.pose.pose.position.y;
    truck_z = rtk_odom_msg.pose.pose.position.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "psuedo_truck_follower");

    ros::NodeHandle n;

    ros::Rate loop_rate(10);

    ros::Publisher pubWay = n.advertise<geometry_msgs::Pose>("uav_control/waypoint", 100);

    ros::Subscriber sub0 = n.subscribe("/hexacopter1/localizer/global_odom", 1, truckPositionCallback);
    // ros::Subscriber sub1 = n.subscribe("/gazebo/model_states", 1, gazebo_truck_locs);

    geometry_msgs::Pose target_waypoint;

    while (ros::ok())
    {
	if (odom_received == true)
	{
	    target_waypoint.position.x = truck_x;
	    target_waypoint.position.y = truck_y;
	    target_waypoint.position.z = truck_z + 10;

            target_waypoint.orientation.w = 1.0;

	    pubWay.publish(target_waypoint);
	}

	ros::spinOnce();

	loop_rate.sleep();
    }

    return 0;
}
