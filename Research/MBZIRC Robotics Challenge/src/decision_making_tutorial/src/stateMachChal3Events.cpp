/*
 * Filename: TurnstileEvents.cpp
 *   Author: Igor Makhtes
 *     Date: Jan 7, 2014
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014 Cogniteam Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <iostream>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/assign/std/vector.hpp>
#include <governor/target_list.h>
#include <governor/assignment.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <algorithm>
#include <cmath>

using namespace std;
using namespace boost::assign;

#define foreach BOOST_FOREACH

float x[3] = {0,0,0}, y[3] = {0,0,0}, z[3] = {0,0,0};
float tx[23], ty[23], tpoints[23];
float model_pose[3], model_velo[3];
int served[23];

void tlistCallback(const governor::target_list::ConstPtr&  msg)
{
	//ROS_INFO("I heard: x = [%f], y = [%f] ", msg.x, msg.y);

	for(int i = 0; i < 23; i++){
		tx[i] = msg->x[i];
		ty[i] = msg->y[i];
		tpoints[i] = msg->Points[i];
		served[i] = msg->Served[i];
	}
}


void batteryCallback(const sensor_msgs::BatteryState msg){
	ROS_INFO("Battery at [%f] volts.", msg.voltage);  
}


void pose0Callback(const geometry_msgs::PoseStamped msg){
	x[0] = msg.pose.position.x;
	y[0] = msg.pose.position.y;
	z[0] = msg.pose.position.z;
}
void pose1Callback(const geometry_msgs::PoseStamped msg){
	x[1] = msg.pose.position.x;
	y[1] = msg.pose.position.y;
	z[1] = msg.pose.position.z;
}
void pose2Callback(const geometry_msgs::PoseStamped msg){
	x[2] = msg.pose.position.x;
	y[2] = msg.pose.position.y;
	z[2] = msg.pose.position.z;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "stateMachChal3_events");

    ros::NodeHandle node;
    
	ros::Subscriber sub1 = node.subscribe("/governor/tlist", 1, tlistCallback);

	ros::Subscriber sub2 = node.subscribe("/mavros/battery", 100, batteryCallback);


	ros::Subscriber sub3 = node.subscribe("/hexacopter0/mavros/local_position/pose", 1, pose0Callback);
	ros::Subscriber sub4 = node.subscribe("/hexacopter1/mavros/local_position/pose", 1, pose1Callback);
	ros::Subscriber sub5 = node.subscribe("/hexacopter2/mavros/local_position/pose", 1, pose2Callback);
	    
    ros::Publisher eventPublisher0 = node.advertise<std_msgs::String>("/decision_making/hexacopter0/stateMachChal3/events", 1, false);
    ros::Publisher eventPublisher1 = node.advertise<std_msgs::String>("/decision_making/hexacopter1/stateMachChal3/events", 1, false);
    ros::Publisher eventPublisher2 = node.advertise<std_msgs::String>("/decision_making/hexacopter2/stateMachChal3/events", 1, false);
    
    int currentEvent = 0;
    vector<string> events;
    events += "targetDelivered",
    		"targetMissed",
			"targetDropped",
			"targetAcquired",
			"allTargetsFound",
//			"allTargetsDelivered",
			"opAltReached",
			"targetInView",
//			"timeExpired",
    		"someTargetsFound",
			"noTargetInList";
    
    ROS_INFO("Starting Challenge 3 event publisher...");

    std_msgs::String event;
    
    
    while (ros::ok()) {

    	
//TAKEOFF EXIT CONDITIONS
    	
//    	if z[0] == 10 && z[1] == 12 && z[3] == 14){
//    		event.data = events
//    		eventPublisher0.publish(event);
//    	}
    	
    	
    	
    	
    	
    	std_msgs::String event;
        event.data = events[currentEvent++];
        currentEvent %= events.size();

        eventPublisher0.publish(event);
        eventPublisher1.publish(event);
        eventPublisher2.publish(event);
        
        
        boost::this_thread::sleep(boost::posix_time::seconds(1));
    }
	return 0;
}









