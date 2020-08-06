#include "SBGC.h"
#include "SBGC_cmd_helpers.cpp"
#include <termios.h> /* POSIX terminal control definitions */
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <sys/ioctl.h>
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include <cmath>

double pitch = 0, roll = 0, yaw = 0, qx, qy, qz, qw, q[4];

// Quaternion to Euler angles
void gimbal_angles_cb(const geometry_msgs::PoseStamped mesg1)
{
    // Extract pitch, roll, yaw from quaternion
    qx = mesg1.pose.orientation.x;
	qy = mesg1.pose.orientation.y;
	qz = mesg1.pose.orientation.z;
	qw = mesg1.pose.orientation.w;

	// roll (x-axis rotation)
	double sinr = +2.0 * (qw * qx + qy * qz);
	double cosr = +1.0 - 2.0 * (qx * qx + qy * qy);
	roll = -atan2(sinr, cosr)-M_PI;

	// pitch (y-axis rotation)
	double sinp = +2.0 * (qw * qy - qz * qx);
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny = +2.0 * (qw * qz + qx * qy);
	double cosy = +1.0 - 2.0 * (qy * qy + qz * qz);  
	yaw = atan2(siny, cosy);

    std::cout << "Gimbal IMU Angles:" << std::endl << "Roll: " << roll * 180 / M_PI << "    Pitch: " << pitch * 180 / M_PI << " Yaw: " << yaw * 180 / M_PI << std::endl << std::endl;


}

// Quaternion to Euler angles
void gimbal_angles_cb_1(const geometry_msgs::PoseStamped mesg1)
{
    // Extract pitch, roll, yaw from quaternion
    qx = mesg1.pose.orientation.x;
	qy = mesg1.pose.orientation.y;
	qz = mesg1.pose.orientation.z;
	qw = mesg1.pose.orientation.w;

	// roll (x-axis rotation)
	double sinr = +2.0 * (qw * qx + qy * qz);
	double cosr = +1.0 - 2.0 * (qx * qx + qy * qy);
	yaw = -atan2(sinr, cosr)-M_PI;

	// pitch (y-axis rotation)
	double sinp = +2.0 * (qw * qy - qz * qx);
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny = +2.0 * (qw * qz + qx * qy);
	double cosy = +1.0 - 2.0 * (qy * qy + qz * qz);  
	roll = atan2(siny, cosy);

    std::cout << "Target Angles:" << std::endl << "Roll: " << roll * 180 / M_PI << "    Pitch: " << pitch * 180 / M_PI << " Yaw: " << yaw * 180 / M_PI << std::endl << std::endl;


}

// Quaternion to Euler angles
void gimbal_angles_cb_2(const sensor_msgs::Imu mesg1)
{
    // Extract pitch, roll, yaw from quaternion
    qx = mesg1.orientation.x;
	qy = mesg1.orientation.y;
	qz = mesg1.orientation.z;
	qw = mesg1.orientation.w;

	// roll (x-axis rotation)
	double sinr = +2.0 * (qw * qx + qy * qz);
	double cosr = +1.0 - 2.0 * (qx * qx + qy * qy);
	roll = atan2(sinr, cosr);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (qw * qy - qz * qx);
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny = +2.0 * (qw * qz + qx * qy);
	double cosy = +1.0 - 2.0 * (qy * qy + qz * qz);  
	yaw = atan2(siny, cosy);

    std::cout << "Pixhawk IMU Angles:" << std::endl << "Roll: " << roll * 180 / M_PI << "    Pitch: " << pitch * 180 / M_PI << " Yaw: " << yaw * 180 / M_PI << std::endl << std::endl;


}

// Euler angles to Quaternion
void euler2quat (double pt, double rt, double yt)
{    
    double cy = cos(yt * 0.5);
	double sy = sin(yt * 0.5);
	double cr = cos(rt * 0.5);
	double sr = sin(rt * 0.5);
	double cp = cos(pt * 0.5);
	double sp = sin(pt * 0.5);
        
	q[0] = cy * cr * cp + sy * sr * sp;
	q[1] = cy * sr * cp - sy * cr * sp;
	q[2] = cy * cr * sp + sy * sr * cp;
	q[3] = sy * cr * cp - cy * sr * sp;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gimbal_control_node");
    
    ros::NodeHandle n;

    // ros::Publisher pub_imu_angle = n.advertise<geometry_msgs::PoseStamped>("gimbal_imu_angles", 1);
    ros::Subscriber sub = n.subscribe("gimbal_imu_angles", 1, gimbal_angles_cb);
    // ros::Subscriber sub1 = n.subscribe("gimbal_target_orientation", 1, gimbal_angles_cb_1);
	ros::Subscriber sub2 = n.subscribe("mavros/imu/data", 1, gimbal_angles_cb_2);
    // geometry_msgs::PoseStamped gm_msg;



    // zero_gimbal_IMU();


        //Convert to quaternion to publish, Using -enc_angles[2] for yaw due to drift issues.
        // euler2quat(imu_angles[0],imu_angles[1],-enc_angles[2]*0.02197265625);  //Convert to Quaternion

        // gm_msg.pose.orientation.w = q[0];
        // gm_msg.pose.orientation.x = q[1];
        // gm_msg.pose.orientation.y = q[2];
        // gm_msg.pose.orientation.z = q[3];

        // pub_imu_angle.publish(gm_msg);

        // std::cout << "Target Angles:" << std::endl << "Roll: " << roll * 180 / M_PI << "    Pitch: " << pitch * 180 / M_PI << " Yaw: " << yaw * 180 / M_PI << std::endl << std::endl;
        // std::cout << "Actual IMU Angles:" << std::endl << "Roll: " << imu_angles[0]*0.02197265625 << "  Pitch: " << imu_angles[1]*0.02197265625 << "  Yaw: " << imu_angles[2]*0.02197265625 << std::endl << std::endl;
        // std::cout << "Actual ENC Angles:" << std::endl << "Roll: " << enc_angles[0]*0.02197265625 << "  Pitch: " << enc_angles[1]*0.02197265625 << "  Yaw: " << enc_angles[2]*0.02197265625 << std::endl << std::endl;
        // std::cout <<  "Yaw_drift: " << yaw_drift << "  Target_Yaw_corrected: " << -yaw_drift+yaw * 180 / M_PI << std::endl;
        // std::cout << "------------------" << std::endl;
        
        ros::spin();    


    return(0);
}   