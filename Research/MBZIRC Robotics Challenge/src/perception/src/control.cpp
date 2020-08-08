/*
 * Filename: Wandering.cpp
 *   Author: Igor Makhtes
 *     Date: Dec 16, 2013
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013 Cogniteam Ltd.
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




#include <stdio.h>
#include <cmath>
#include <vector>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>






#include <iostream>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <random_numbers/random_numbers.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>

using namespace std;
using namespace decision_making;

#define foreach BOOST_FOREACH

/*************************************************************************************************
*** Variables
**************************************************************************************************/

random_numbers::RandomNumberGenerator _randomizer;
ros::Publisher _velocityPublisher;
ros::Publisher c_pub;

  double x, y, z;
  double d_x, d_y, d_z;
  double f_x, f_y, f_z;
  double first_detection;
 double d;

/*************************************************************************************************
*** Final state machine
**************************************************************************************************/

FSM(Wandering)
{
    FSM_STATES
    {
        Takeoff,
        White_Target,
        Target_detection,
	Tracking,
	Stop_correcting,
	Landing
    }
    FSM_START(Takeoff)
    FSM_BGN
    {
        FSM_STATE(Takeoff)
        {
            FSM_CALL_TASK(Takeoff)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/WHITE_TARGET", FSM_NEXT(White_Target))
            }
        }
        FSM_STATE(White_Target)
        {
            FSM_CALL_TASK(White_Target)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/CIRCLE_TARGET", FSM_NEXT(Target_detection))
                FSM_ON_EVENT("/NO_CIRCLE", FSM_NEXT(Takeoff))
            }
        }
        FSM_STATE(Target_detection)
        {
            FSM_CALL_TASK(Target_detection)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/TRACKING", FSM_NEXT(Tracking))
            }
        }
        FSM_STATE(Tracking)
        {
            FSM_CALL_TASK(Tracking)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/STOP_CORRECTING", FSM_NEXT(Stop_correcting))
            }
        }
        FSM_STATE(Stop_correcting)
        {
            FSM_CALL_TASK(Stop_correcting)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/LANDING", FSM_NEXT(Landing))
            }
        }
        FSM_STATE(Landing)
        {
            FSM_CALL_TASK(Landing)

            FSM_TRANSITIONS
            {
               FSM_ON_EVENT("/RESUME", FSM_NEXT(Takeoff))
            }
        }




    }
    FSM_END
}


/*************************************************************************************************
*** ROS Subscriptions
**************************************************************************************************/

void onLaserScanMessage(const sensor_msgs::LaserScan::Ptr laserScanMessage, RosEventQueue* eventQueue) {
    double frontRange = laserScanMessage->ranges[laserScanMessage->ranges.size() / 2];

    if (frontRange < 0.5) {
        //eventQueue->riseEvent("/OBSTACLE");
    }
}



void chatterCallback(const geometry_msgs::Twist msg, RosEventQueue* eventQueue)
        {
                x = msg.linear.x;
                y = msg.linear.y;
                z = msg.linear.z;


	if(d_z>0|d_z<100)
	{
        geometry_msgs::Twist c;
        c.linear.x=(x-d_x)*2;
        c.linear.y=(y-d_y)*2;
		if(d_z>5&&first_detection==1)
		{c.linear.z=-0.4;}
		else
		{
	         c.linear.x=0;
        	 c.linear.y=0;
	 	 //c.linear.z=-d_z/(distance/3);
		c.linear.z=-0.8;		
		}
        c_pub.publish(c);
	}

	else
	{
        geometry_msgs::Twist c;
        c.linear.x=f_x;
        c.linear.y=f_y;
        c.linear.z=-0.4;
        c_pub.publish(c);
	}


        }


        void poseCallback(const geometry_msgs::PoseStamped& msg, RosEventQueue* eventQueue)
        {
                geometry_msgs::Point coord = msg.pose.position;
                d_x = coord.x;
                d_y = coord.y;
                d_z = coord.z;

        }


	void fisheyeCallback(const geometry_msgs::Twist msg, RosEventQueue* eventQueue)
        {
                f_x = msg.linear.x;
                f_y = msg.linear.y;
                f_z = msg.linear.z;

        }

	void detectionCallback(const geometry_msgs::Twist msg, RosEventQueue* eventQueue)
        {
                first_detection = msg.linear.x;
               d = msg.linear.y;
                //f_z = msg.linear.z;


	if(first_detection==1)
	{eventQueue->riseEvent("/WHITE_TARGET");}



        }


/*************************************************************************************************
*** Task implementations
**************************************************************************************************/



decision_making::TaskResult takeoff(string name, const FSMCallContext& context, EventQueue& eventQueue) {


}

decision_making::TaskResult white_Target(string name, const FSMCallContext& context, EventQueue& eventQueue) {



}

decision_making::TaskResult target_detection(string name, const FSMCallContext& context, EventQueue& eventQueue) {


    //return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult tracking(string name, const FSMCallContext& context, EventQueue& eventQueue) {


}

decision_making::TaskResult stop_correcting(string name, const FSMCallContext& context, EventQueue& eventQueue) {



}

decision_making::TaskResult landing(string name, const FSMCallContext& context, EventQueue& eventQueue) {


    return decision_making::TaskResult::SUCCESS();
}

/*************************************************************************************************
*** The Main
**************************************************************************************************/

int main(int argc, char **argv) {
    /**
     * Initialization
     */
    ros::init(argc, argv, "fsm_wandering");
    ros_decision_making_init(argc, argv);
    ros::NodeHandle nodeHandle("~");
    RosEventQueue eventQueue;

    /**
     * Tasks registration
     */  
    LocalTasks::registrate("Takeoff", takeoff);
    LocalTasks::registrate("White_Target", white_Target);
    LocalTasks::registrate("Target_detection", target_detection);
    LocalTasks::registrate("Tracking", tracking);
    LocalTasks::registrate("Stop_correcting", stop_correcting);
    LocalTasks::registrate("Landing", landing);

    /**
     * Subscription for the laser topic and velocity publisher creation
     */

    //ros::Subscriber laserSubscriber = nodeHandle.subscribe<void>("scan_raw", 1, boost::function<void(const sensor_msgs::LaserScan::Ptr)>(boost::bind(onLaserScanMessage, _1, &eventQueue)));

    _velocityPublisher = nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 100, false);


    ros::Subscriber point_sub = nodeHandle.subscribe<void>("/rbe_target_point", 1000, boost::function<void(const geometry_msgs::Twist)>(boost::bind(chatterCallback, _1, &eventQueue)));
    ros::Subscriber pose_sub = nodeHandle.subscribe<void>("/hexacopter/mavros/local_position/pose", 1000, boost::function<void(const geometry_msgs::PoseStamped&)>(boost::bind(poseCallback, _1, &eventQueue)));
    ros::Subscriber fisheye_sub = nodeHandle.subscribe<void>("/fisheye_image_frame_point", 1000, boost::function<void(const geometry_msgs::Twist)>(boost::bind(fisheyeCallback, _1, &eventQueue)));
    ros::Subscriber detection_sub = nodeHandle.subscribe<void>("/first_detection", 1000, boost::function<void(const geometry_msgs::Twist)>(boost::bind(detectionCallback, _1, &eventQueue)));
    c_pub = nodeHandle.advertise<geometry_msgs::Twist>("/hexacopter/uav_control/velocity", 10,false);





    /**
     * ROS Spinner for topic subscriptions
     */
    ros::AsyncSpinner spinner(1);
    spinner.start();

    /**
     * Execution of the FSM
     */
    ROS_INFO("Starting wandering machine...");
    FsmWandering(NULL, &eventQueue);

    /**
     * Cleanup
     */
	return 0;
}
