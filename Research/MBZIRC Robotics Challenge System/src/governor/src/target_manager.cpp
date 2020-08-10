#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "gazebo_msgs/ModelStates.h"
#include "governor/TargetMsg.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <sstream>
#include <string>
#include "governor/target_list.h"

//float32 x_position
//float32 y_position
//string color
//string shape

//int h0_newflag = 0, h1_newflag = 0, h2_newflag = 0;

//List Fields
float x[23] = {0}, y[23] = {0}, x_h0 = 0, y_h0 = 0;
int IDed[23] = {0}, tpoints[23] = {0};
int no_of_targets_in_list = 0;
int listed_flag = 0, ID = 0;

//Seperation threshold between perceived targets necessary to qualify target located at a different location
float sep_thres = 1;

std::string targ_type[23], c_h0;

//char name[23]

void hex0_targs(const governor::TargetMsg msg)
{
    x_h0 = msg.x_position;
    y_h0 = msg.y_position;
    c_h0 = msg.color;

    no_of_targets_in_list = 0;

    for (int i = 0; i < 23; i++)
    {
	no_of_targets_in_list = no_of_targets_in_list + IDed[i];
    }

    listed_flag = 0;

    for (int i = 0; i <= no_of_targets_in_list; i++)
    {

	//If new perceived targets color/type is the same
	if (c_h0 == targ_type[i])
	{

	    //Compare location with existing targets location to see if it is already listed
	    if (abs(x_h0 - x[i]) < sep_thres && abs(y_h0 - y[i]) < sep_thres)
	    {
		listed_flag = 1;
		ID = i;
	    }
	}
    }

    if (listed_flag == 0)
    {
	if (c_h0 == "red")
	{
	    x[no_of_targets_in_list] = (x[no_of_targets_in_list] + x_h0) / 2; //Using a running average on position.
	    y[no_of_targets_in_list] = (y[no_of_targets_in_list] + y_h0) / 2;
	    targ_type[no_of_targets_in_list] = c_h0;
	    tpoints[no_of_targets_in_list] = 1;
	    IDed[no_of_targets_in_list] = 1;
	}
	if (c_h0 == "green")
	{
	    x[no_of_targets_in_list] = (x[no_of_targets_in_list] + x_h0) / 2; //Using a running average on position.
	    y[no_of_targets_in_list] = (y[no_of_targets_in_list] + y_h0) / 2;
	    targ_type[no_of_targets_in_list] = c_h0;
	    tpoints[no_of_targets_in_list] = 2;
	    IDed[no_of_targets_in_list] = 1;
	}
	if (c_h0 == "blue")
	{
	    x[no_of_targets_in_list] = (x[no_of_targets_in_list] + x_h0) / 2; //Using a running average on position.
	    y[no_of_targets_in_list] = (y[no_of_targets_in_list] + y_h0) / 2;
	    targ_type[no_of_targets_in_list] = c_h0;
	    tpoints[no_of_targets_in_list] = 3;
	    IDed[no_of_targets_in_list] = 1;
	}
	if (c_h0 == "orange")
	{
	    x[no_of_targets_in_list] = (x[no_of_targets_in_list] + x_h0) / 2; //Using a running average on position.
	    y[no_of_targets_in_list] = (y[no_of_targets_in_list] + y_h0) / 2;
	    targ_type[no_of_targets_in_list] = c_h0;
	    tpoints[no_of_targets_in_list] = 5;
	    IDed[no_of_targets_in_list] = 1;
	}
	if (c_h0 == "yellow")
	{
	    x[no_of_targets_in_list] = x[no_of_targets_in_list];
	    y[no_of_targets_in_list] = y[no_of_targets_in_list];
	    targ_type[no_of_targets_in_list] = c_h0;
	    tpoints[no_of_targets_in_list] = 5;
	    IDed[no_of_targets_in_list] = 1;
	}
    }

    if (listed_flag == 1)
    {
	if (c_h0 == "red" || c_h0 == "green" || c_h0 == "blue" || c_h0 == "orange")
	{
	    x[ID] = (x[ID] + x_h0) / 2; //Using a running average on position.
	    y[ID] = (y[ID] + y_h0) / 2;
	}
	if (c_h0 == "yellow")
	{
	    x[ID] = x_h0;
	    y[ID] = y_h0;
	}
    }
}

void hex1_targs(const governor::TargetMsg msg)
{
    x_h0 = msg.x_position;
    y_h0 = msg.y_position;
    c_h0 = msg.color;

    no_of_targets_in_list = 0;

    for (int i = 0; i < 23; i++)
    {
	no_of_targets_in_list = no_of_targets_in_list + IDed[i];
    }

    listed_flag = 0;

    for (int i = 0; i <= no_of_targets_in_list; i++)
    {

	//If new perceived targets color/type is the same
	if (c_h0 == targ_type[i])
	{

	    //Compare location with existing targets location to see if it is already listed
	    if (abs(x_h0 - x[i]) < sep_thres && abs(y_h0 - y[i]) < sep_thres)
	    {
		listed_flag = 1;
		ID = i;
	    }
	}
    }

    if (listed_flag == 0)
    {
	if (c_h0 == "red")
	{
	    x[no_of_targets_in_list] = (x[no_of_targets_in_list] + x_h0) / 2; //Using a running average on position.
	    y[no_of_targets_in_list] = (y[no_of_targets_in_list] + y_h0) / 2;
	    targ_type[no_of_targets_in_list] = c_h0;
	    tpoints[no_of_targets_in_list] = 1;
	    IDed[no_of_targets_in_list] = 1;
	}
	if (c_h0 == "green")
	{
	    x[no_of_targets_in_list] = (x[no_of_targets_in_list] + x_h0) / 2; //Using a running average on position.
	    y[no_of_targets_in_list] = (y[no_of_targets_in_list] + y_h0) / 2;
	    targ_type[no_of_targets_in_list] = c_h0;
	    tpoints[no_of_targets_in_list] = 2;
	    IDed[no_of_targets_in_list] = 1;
	}
	if (c_h0 == "blue")
	{
	    x[no_of_targets_in_list] = (x[no_of_targets_in_list] + x_h0) / 2; //Using a running average on position.
	    y[no_of_targets_in_list] = (y[no_of_targets_in_list] + y_h0) / 2;
	    targ_type[no_of_targets_in_list] = c_h0;
	    tpoints[no_of_targets_in_list] = 3;
	    IDed[no_of_targets_in_list] = 1;
	}
	if (c_h0 == "orange")
	{
	    x[no_of_targets_in_list] = (x[no_of_targets_in_list] + x_h0) / 2; //Using a running average on position.
	    y[no_of_targets_in_list] = (y[no_of_targets_in_list] + y_h0) / 2;
	    targ_type[no_of_targets_in_list] = c_h0;
	    tpoints[no_of_targets_in_list] = 5;
	    IDed[no_of_targets_in_list] = 1;
	}
	if (c_h0 == "yellow")
	{
	    x[no_of_targets_in_list] = x[no_of_targets_in_list];
	    y[no_of_targets_in_list] = y[no_of_targets_in_list];
	    targ_type[no_of_targets_in_list] = c_h0;
	    tpoints[no_of_targets_in_list] = 5;
	    IDed[no_of_targets_in_list] = 1;
	}
    }

    if (listed_flag == 1)
    {
	if (c_h0 == "red" || c_h0 == "green" || c_h0 == "blue" || c_h0 == "orange")
	{
	    x[ID] = (x[ID] + x_h0) / 2; //Using a running average on position.
	    y[ID] = (y[ID] + y_h0) / 2;
	}
	if (c_h0 == "yellow")
	{
	    x[ID] = x_h0;
	    y[ID] = y_h0;
	}
    }
}

void hex2_targs(const governor::TargetMsg msg)
{
    x_h0 = msg.x_position;
    y_h0 = msg.y_position;
    c_h0 = msg.color;

    no_of_targets_in_list = 0;

    for (int i = 0; i < 23; i++)
    {
	no_of_targets_in_list = no_of_targets_in_list + IDed[i];
    }

    listed_flag = 0;

    for (int i = 0; i <= no_of_targets_in_list; i++)
    {

	//If new perceived targets color/type is the same
	if (c_h0 == targ_type[i])
	{

	    //Compare location with existing targets location to see if it is already listed
	    if (abs(x_h0 - x[i]) < sep_thres && abs(y_h0 - y[i]) < sep_thres)
	    {
		listed_flag = 1;
		ID = i;
	    }
	}
    }

    if (listed_flag == 0)
    {
	if (c_h0 == "red")
	{
	    x[no_of_targets_in_list] = (x[no_of_targets_in_list] + x_h0) / 2; //Using a running average on position.
	    y[no_of_targets_in_list] = (y[no_of_targets_in_list] + y_h0) / 2;
	    targ_type[no_of_targets_in_list] = c_h0;
	    tpoints[no_of_targets_in_list] = 1;
	    IDed[no_of_targets_in_list] = 1;
	}
	if (c_h0 == "green")
	{
	    x[no_of_targets_in_list] = (x[no_of_targets_in_list] + x_h0) / 2; //Using a running average on position.
	    y[no_of_targets_in_list] = (y[no_of_targets_in_list] + y_h0) / 2;
	    targ_type[no_of_targets_in_list] = c_h0;
	    tpoints[no_of_targets_in_list] = 2;
	    IDed[no_of_targets_in_list] = 1;
	}
	if (c_h0 == "blue")
	{
	    x[no_of_targets_in_list] = (x[no_of_targets_in_list] + x_h0) / 2; //Using a running average on position.
	    y[no_of_targets_in_list] = (y[no_of_targets_in_list] + y_h0) / 2;
	    targ_type[no_of_targets_in_list] = c_h0;
	    tpoints[no_of_targets_in_list] = 3;
	    IDed[no_of_targets_in_list] = 1;
	}
	if (c_h0 == "orange")
	{
	    x[no_of_targets_in_list] = (x[no_of_targets_in_list] + x_h0) / 2; //Using a running average on position.
	    y[no_of_targets_in_list] = (y[no_of_targets_in_list] + y_h0) / 2;
	    targ_type[no_of_targets_in_list] = c_h0;
	    tpoints[no_of_targets_in_list] = 5;
	    IDed[no_of_targets_in_list] = 1;
	}
	if (c_h0 == "yellow")
	{
	    x[no_of_targets_in_list] = x[no_of_targets_in_list];
	    y[no_of_targets_in_list] = y[no_of_targets_in_list];
	    targ_type[no_of_targets_in_list] = c_h0;
	    tpoints[no_of_targets_in_list] = 5;
	    IDed[no_of_targets_in_list] = 1;
	}
    }

    if (listed_flag == 1)
    {
	if (c_h0 == "red" || c_h0 == "green" || c_h0 == "blue" || c_h0 == "orange")
	{
	    x[ID] = (x[ID] + x_h0) / 2; //Using a running average on position.
	    y[ID] = (y[ID] + y_h0) / 2;
	}
	if (c_h0 == "yellow")
	{
	    x[ID] = x_h0;
	    y[ID] = y_h0;
	}
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_manager");

    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<governor::target_list>("/governor/tlist", 100);

    ros::Rate loop_rate(1);

    //Initial Target List Topic before subscribing to UAV perception nodes
    governor::target_list msg;

    for (int n = 0; n < 23; n++)
    {
	msg.IDed.push_back(0);
	msg.Target_Type.push_back(std::string(""));
	msg.Served.push_back(0);
	msg.x.push_back(x[n]);
	msg.y.push_back(y[n]);
	msg.Points.push_back(tpoints[n]);
    }

    pub.publish(msg);

    ros::Subscriber sub0 = n.subscribe("/hexacopter0/perception/targets", 1, hex0_targs);
    ros::Subscriber sub1 = n.subscribe("/hexacopter1/perception/targets", 1, hex1_targs);
    ros::Subscriber sub2 = n.subscribe("/hexacopter2/perception/targets", 1, hex2_targs);

    while (ros::ok())
    {

	governor::target_list msg;

	for (int n = 0; n < 23; n++)
	{
	    msg.IDed.push_back(IDed[n]);
	    msg.Target_Type.push_back(targ_type[n]);
	    //msg.Served.push_back();  This needs to be initialized once in this node then changed by the state machine when it confirms if target has been served
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
