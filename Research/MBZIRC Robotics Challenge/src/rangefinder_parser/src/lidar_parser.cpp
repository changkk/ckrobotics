#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <sstream>
#include <string>
#include <cmath>
#include "mavros_msgs/Mavlink.h"
#include "sensor_msgs/Imu.h"

ros::Publisher pub;
ros::Subscriber sub0;
ros::Subscriber sub1;

std::vector<long unsigned int> payload;
long unsigned int distancePayload[4] = {0};
float f;
double t0, t1, t2, t3, t4, ysqr, pitch = 0, roll = 0, yaw = 0, qx, qy, qz, qw, agl;
std_msgs::Float64 msg;



void MavlinkMsgCallback(const mavros_msgs::Mavlink mesg)
{

	if (mesg.msgid == 173)
    {
	payload.clear();
	payload = mesg.payload64;


	    for (int i = 0; i < 4; i++)
	    {
			distancePayload[i] = payload[i];
	    }

	    
	    char *pul = (char *)&distancePayload; // ok, char* can alias any type
	    char *pf = (char *)&f;		  // ok, char* can alias any type
	    memcpy(pf, pul, sizeof(float));


    }
}


void MavrosImuCallback(const sensor_msgs::Imu mesg1)
{
	qx = mesg1.orientation.x;
	qy = mesg1.orientation.y;
	qz = mesg1.orientation.z;
	qw = mesg1.orientation.w;

	ysqr = qy * qy;

	// roll (x-axis rotation)
	t0 = +2.0 * (qw * qx + qy * qz);
	t1 = +1.0 - 2.0 * (qx * qx + ysqr);
	roll = std::atan2(t0, t1);

	// pitch (y-axis rotation)
	t2 = +2.0 * (qw * qy - qz * qx);
	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	pitch = std::asin(t2);

	// yaw (z-axis rotation)
	t3 = +2.0 * (qw * qz + qx * qy);
	t4 = +1.0 - 2.0 * (ysqr + qz * qz);  
	yaw = std::atan2(t3, t4);
	
	// std::cout << pitch << "	" << roll << "	" << yaw << "	" << std::endl;
	
	agl = f * std::cos(roll) * std::cos(pitch);

	// std::cout << f << "	" << agl << std::endl;

	msg.data = agl;
		
	pub.publish(msg);

}







int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_parser");

    ros::NodeHandle n;

    pub = n.advertise<std_msgs::Float64>("rangefinder", 100);

    ros::Rate loop_rate(30);

    sub0 = n.subscribe("mavlink/from", 1, MavlinkMsgCallback);
	sub1 = n.subscribe("mavros/imu/data", 1, MavrosImuCallback);

	ros::spin();

    return 0;
}