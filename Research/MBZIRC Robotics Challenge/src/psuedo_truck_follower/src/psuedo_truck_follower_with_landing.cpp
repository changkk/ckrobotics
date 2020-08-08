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
#include <mavros_msgs/RCIn.h>


float truck_x = 0, truck_y = 0, truck_z = 0;
bool odom_received = false;
int throttle = 1500;

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

void RCthrottleCallback(const mavros_msgs::RCIn msg)
{
    throttle = msg.channels[2];
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "psuedo_truck_follower");

    ros::NodeHandle n;

    ros::Rate loop_rate(10);

    ros::Publisher pubWay = n.advertise<geometry_msgs::Pose>("uav_control/waypoint", 100);

    ros::Subscriber sub0 = n.subscribe("/hexacopter1/localizer/global_odom", 1, truckPositionCallback);
    ros::Subscriber sub2 = n.subscribe("mavros/rc/in", 1, RCthrottleCallback);
    // ros::Subscriber sub1 = n.subscribe("/gazebo/model_states", 1, gazebo_truck_locs);

    geometry_msgs::Pose target_waypoint;
    float alt = 10;

    while (ros::ok())
    {

        if (odom_received == true)
        {
            if (throttle < 1350 || throttle > 1650)
            {
                alt = alt + ((throttle - 1500) / 600);
            }

            if (alt < 0)
            {
                alt = 0;
            }
            if (alt > 10)
            {
                alt = 10;
            }

            target_waypoint.position.x = truck_x;
            target_waypoint.position.y = truck_y;
            target_waypoint.position.z = truck_z + alt;

            pubWay.publish(target_waypoint);
        }

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}