#include <string.h>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <cmath>






int main(int argc, char **argv)
{
    ros::init(argc, argv, "psuedo_rtk_node");
    
    ros::NodeHandle n;

    ros::Publisher position_pub = n.advertise<nav_msgs::Odometry>("hexacopter/gps/rtkfix", 1);

    ros::Rate loop_rate(5);
    // usleep(delay);

    nav_msgs::Odometry msg;

    double x = 0, y = 0, z = 0;
    int count = 0;

    while(ros::ok())
    {
        // x = 300*sin(0.1/6*count);
        // y = 100*cos(0.1/6*count);
        // z = 10*cos(0.1/6*count)-10;

        // NED frame
        x = 100*sin(0.1/6*count)-150;
        y = 100*cos(0.1/6*count)-150;;
        z = -10*cos(0.1/6*count)-10;

        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = y;
        msg.pose.pose.position.z = z;

        
        // std::cout << pitch * 180 / M_PI << "  " << yaw * 180 / M_PI << "  " << roll * 180 / M_PI << std::endl;

        position_pub.publish(msg);

        count++;
        ros::spinOnce();    
        loop_rate.sleep();

    }

    return(0);
}  