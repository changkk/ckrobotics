
//#include <iostream>
//
//#include <ros/ros.h>

#include <iostream>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/assign/std/vector.hpp>
#include <governor/target_list.h>
#include <governor/assignment.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <string>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <decision_making/SynchCout.h>
#include <decision_making/BT.h>
#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace decision_making;

//PARAMETERS----------------------------------

//Battery voltage required before start
float bat_start_min_volt = 24.90, bat_min_volt = 21.0;
//Altitude for UAV's to reach before moving past takeoff stage
float optAlt[3] = {7.0, 7.0, 7.0};
//Flight mode to be in prior to start and to end with after landing.
std::string start_mode = "Stabilize";
//RTK Covariance Threshold to indicate RTK GPS Fix
float rtk_cov_thres = 100;




//GLOBAL VARIABLES-----------------------------

ros::Publisher pubhex_takeoff;
ros::Publisher pubhex_rcin;
ros::Publisher pubhex_search;
ros::Publisher pubhex_acquire;
ros::Publisher pubhex_rtl;
ros::Publisher pubhex_land;
ros::Publisher pubOCS;
ros::Subscriber sub8; 				// Contact_sensors subscriber, declared globally so can be shutdown after its single use.
ros::Subscriber sub9;				// RTK GPS subscription
ros::Subscriber sub10;				// Perspective Camera subscribtion
ros::Subscriber sub11;				// Fisheye Camera subscribtion
ros::Subscriber sub12;				// All Start subscribtion
ros::Subscriber sub13;				// All RTL topic subscribtion
ros::Subscriber sub14;				// All Land topic subscribtion


//UAV position vars
float x_local[3] = {0, 0, 0}, y_local[3] = {0, 0, 0}, z_local[3] = {0, 0, 0};

//Target Database
float tx[23], ty[23], tpoints[23];

//Gazebo Target Location
float model_pose[3], model_velo[3];

//More Target Database Vars
int ided[23] = {0}, served[23] = {0};
std::vector<std::string> ttype;

//Governor assignment
int assignment[3] = {0, 0, 0};
int hexa = 0;
int unserved_moving_target_flag = 100;


std::string ns = ros::this_node::getNamespace();
float bat_volt = 0, rtk_cov = 100;

//CHANGE THESE TO FALSE FOR ACTUAL SYSTEM
bool system_ready = true, start_flag = true, start_flag_buffer = true;

bool armed = false, guided = false, contact_check = false;
bool bat_check = false, flight_mode_check = false, cam_check = false;
bool rtkfix_check = false, fisheye_cam_check = false, perspective_cam_check = false;
bool all_land = false, all_rtl = false, timeExpired = false, search_complete = false, search_complete_buffer = false, tf_check = false;

std::string flightmode;
stringstream ss (stringstream::in | stringstream::out);
std_msgs::String ocsmsg;
std_msgs::String base_link;
tf::TransformListener tfListener;

//---------------------------------------------

FSM(Challenge3)
{
    FSM_STATES
    {
	PreArmChecks,
	    Standby,
	    Takeoff,
	    Search,
	    Track,
	    Capture,
	    Drop,
	    RTL,
	    Land
    }

    FSM_START(PreArmChecks);
    
	FSM_BGN
    {
	FSM_STATE(PreArmChecks)
	{

	    FSM_CALL_TASK(PreArmChecks);

	    FSM_TRANSITIONS
	    {
		FSM_ON_CONDITION(system_ready, FSM_NEXT(Standby));
	    }
	}
	FSM_STATE(Standby)
	{
	    FSM_CALL_TASK(Standby);

	    FSM_TRANSITIONS
	    {
		FSM_ON_CONDITION(start_flag, FSM_NEXT(Takeoff));
	    }
	}
	FSM_STATE(Takeoff)
	{
	    FSM_CALL_TASK(Takeoff);

	    FSM_TRANSITIONS
	    {
		FSM_ON_CONDITION((z_local[hexa] >= optAlt[hexa]), FSM_NEXT(Search));
		//FSM_ON_CONDITION((z_local[hexa] >= 10), FSM_NEXT(Search));
	    }
	}
	FSM_STATE(Search)
	{
	    FSM_CALL_TASK(Search);

	    FSM_TRANSITIONS
	    {
		// FSM_ON_EVENT("/allTargetsFound", FSM_NEXT(Track));

		FSM_ON_CONDITION(all_rtl, FSM_NEXT(RTL));
		FSM_ON_CONDITION(all_land, FSM_NEXT(Land));
	    }
	}
	FSM_STATE(Track)
	{
	    FSM_CALL_TASK(Track);

	    FSM_TRANSITIONS
	    {
		FSM_ON_EVENT("/targetInView", FSM_NEXT(Capture));
		FSM_ON_EVENT("/noTargetInList", FSM_NEXT(Search));
		//				FSM_ON_EVENT("/timeExpired", FSM_NEXT(RTL));
		//				FSM_ON_EVENT("/allTargetsDelivered", FSM_NEXT(RTL));
	    }
	}
	FSM_STATE(Capture)
	{
	    FSM_TRANSITIONS
	    {
		FSM_ON_EVENT("/targetAcquired", FSM_NEXT(Drop));
		FSM_ON_EVENT("/targetMissed", FSM_NEXT(Capture));
		//				FSM_ON_EVENT("/timeExpired", FSM_NEXT(RTL));
		//				FSM_ON_EVENT("/allTargetsDelivered", FSM_NEXT(RTL));
	    }
	}
	FSM_STATE(Drop)
	{
	    FSM_TRANSITIONS
	    {
		FSM_ON_EVENT("/targetDelivered", FSM_NEXT(Track));
		FSM_ON_EVENT("/targetDropped", FSM_NEXT(Capture));

		//				FSM_ON_EVENT("/timeExpired", FSM_NEXT(RTL));
		//				FSM_ON_EVENT("/allTargetsDelivered", FSM_NEXT(RTL));
	    }
	}
	FSM_STATE(RTL)
	{
	    FSM_CALL_TASK(RTL);

		 FSM_TRANSITIONS
	    {
		FSM_ON_EVENT("/targetDelivered", FSM_NEXT(Track));
		FSM_ON_EVENT("/targetDropped", FSM_NEXT(Capture));

		//				FSM_ON_EVENT("/timeExpired", FSM_NEXT(RTL));
		//				FSM_ON_EVENT("/allTargetsDelivered", FSM_NEXT(RTL));
	    }


	}
	FSM_STATE(Land)
	{
	    FSM_CALL_TASK(Land);

		FSM_TRANSITIONS
	    {
		FSM_ON_EVENT("/targetDelivered", FSM_NEXT(Track));
		FSM_ON_EVENT("/targetDropped", FSM_NEXT(Capture));

		//				FSM_ON_EVENT("/timeExpired", FSM_NEXT(RTL));
		//				FSM_ON_EVENT("/allTargetsDelivered", FSM_NEXT(RTL));
	    }
		
		// FSM_TRANSITIONS
	    // {
	    // }

	}
    }
	FSM_END
}

//PREARMCHECKS----------------------------------------------------------------------------------------------------------------------------------------------------------
decision_making::TaskResult prearmchecks(string name, const FSMCallContext &context, EventQueue &eventQueue)
{

    ROS_INFO("System Checks in progress...");

	std::string ns = ros::this_node::getNamespace();

	if (ns == "//hexacopter0")
	{
		hexa = 0;
		base_link = "hexacopter0_base_link";"
	}
	if (ns == "//hexacopter1")
	{
		hexa = 1;
		base_link = "hexacopter1_base_link";
	}
	if (ns == "//hexacopter2")
	{
		hexa = 2;
		base_link = "hexacopter2_base_link";
	}

	ss.str(std::string());
	ss << "[Hexacopter " << hexa << "] [State Machine] System checks in progress for Hexacopter " << hexa;
	ocsmsg.data = ss.str();
	pubOCS.publish(ocsmsg);

	while(!system_ready)
	{
		//Check Battery voltage
		if (bat_volt > bat_start_min_volt)
		{
			bat_check = true;
			
			ss.str(std::string());
			ss << "[Hexacopter " << hexa << "] [State Machine] Battery is good at: " << bat_volt << " volts";
			ocsmsg.data = ss.str();
			pubOCS.publish(ocsmsg);
		}
		else
		{
			bat_check = false;

			ss.str(std::string());
			ss << "[Hexacopter " << hexa << "] [State Machine] Battery low for start: " << bat_volt << " volts";
			ocsmsg.data = ss.str();
			pubOCS.publish(ocsmsg);
		}
		
		//Check Flight Mode
		if (flightmode == start_mode)
		{
			flight_mode_check = true;

			ss.str(std::string());
			ss << "[Hexacopter " << hexa << "] [State Machine] Flight mode is good, set to stabilize for start.";
			ocsmsg.data = ss.str();
			pubOCS.publish(ocsmsg);
		}
		else
		{
			flight_mode_check = false;
		
			ss.str(std::string());
			ss << "[Hexacopter " << hexa << "] [State Machine] Flight mode not stabilize, please change to stabilize.";
			ocsmsg.data = ss.str();
			pubOCS.publish(ocsmsg);
		}

		//Check RTK Lock
		if (rtk_cov >= rtk_cov_thres)
		{
			rtkfix_check = true;

			ss.str(std::string());
			ss << "[Hexacopter " << hexa << "] [State Machine] RTK fixed with covariance at: " << rtk_cov;
			ocsmsg.data = ss.str();
			pubOCS.publish(ocsmsg);
		}
		else
		{
			rtkfix_check = false;

			ss.str(std::string());
			ss << "[Hexacopter " << hexa << "] [State Machine] No RTK fix. Covariance is "  << rtk_cov;
			ocsmsg.data = ss.str();
			pubOCS.publish(ocsmsg);
		}

		//Check camera topics
		if (perspective_cam_check && fisheye_cam_check)
		{
			cam_check = true;

			ss.str(std::string());
			ss << "[Hexacopter " << hexa << "] [State Machine] Camera image topics are being published to.";
			ocsmsg.data = ss.str();
			pubOCS.publish(ocsmsg);
		}
		if (!perspective_cam_check)
		{
			cam_check = false;

			ss.str(std::string());
			ss << "[Hexacopter " << hexa << "] [State Machine] Check perspective camera.";
			ocsmsg.data = ss.str();
			pubOCS.publish(ocsmsg);
		}
		if (!fisheye_cam_check)
		{
			cam_check = false;

			ss.str(std::string());
			ss << "[Hexacopter " << hexa << "] [State Machine] Check fisheye camera.";
			ocsmsg.data = ss.str();
			pubOCS.publish(ocsmsg);
		}

		//Check contact sensor
		if (contact_check)
		{
			contact_check = true;

			ss.str(std::string());
			ss << "[Hexacopter " << hexa << "] [State Machine] Contact sensor operational.";
			ocsmsg.data = ss.str();
			pubOCS.publish(ocsmsg);
		}
		else
		{
			contact_check = false;

			ss.str(std::string());
			ss << "[Hexacopter " << hexa << "] [State Machine] Check contact sensor operation.";
			ocsmsg.data = ss.str();
			pubOCS.publish(ocsmsg);
		}

		//TF Check - TF Listener - Listen from global map to baselink - if exists, everything in place.
		// Ask J$

		if (tfListener.waitForTransform("global_map", base_link, ros::Time::now(), ros::Duration(1))) 
		{
			tf_check = true;
		}


		if (bat_check && flight_mode_check && rtkfix_check && cam_check && contact_check && tf_check)
		{
			ss.str(std::string());
			ss << "[Hexacopter " << hexa << "] [State Machine] SYSTEM READY. Switching to standby.";
			ocsmsg.data = ss.str();
			pubOCS.publish(ocsmsg);

			system_ready = true;
		}



		ros::Duration(2).sleep();

	}

    return decision_making::TaskResult::SUCCESS();
}

//STANDBY----------------------------------------------------------------------------------------------------------------------------------------------------------
decision_making::TaskResult standby(string name, const FSMCallContext &context, EventQueue &eventQueue)
{

    ss.str(std::string());
	ss << "[Hexacopter " << hexa << "] [State Machine] Challenge 3 Standing By. Waiting for start message.";
	ocsmsg.data = ss.str();
	pubOCS.publish(ocsmsg);


    while(!start_flag_buffer)
	{
		ros::Duration(1).sleep();
	}
	
   
	ss.str(std::string());
	ss << "[Hexacopter " << hexa << "] [State Machine] --- Start message received by Hexacopter "<< hexa << ", commencing Challenge 3 ---";
	ocsmsg.data = ss.str();
	pubOCS.publish(ocsmsg);

	start_flag = true;

    return decision_making::TaskResult::SUCCESS();
}

//TAKEOFF----------------------------------------------------------------------------------------------------------------------------------------------------------
decision_making::TaskResult takeoff(string name, const FSMCallContext &context, EventQueue &eventQueue)
{
    	
	
	ss.str(std::string());
	ss << "[Hexacopter " << hexa << "] [State Machine] --- Start message received by Hexacopter "<< hexa << ", commencing Challenge 3 ---";
	ocsmsg.data = ss.str();
	pubOCS.publish(ocsmsg);

	ros::Duration(5).sleep();
	
    	std_msgs::Empty to;
    
	if (hexa == 0)
	{
		int ini_alt = z_local[0];
		int count = 0;

		while(z_local[0] < (ini_alt + 2) && count < 4)
		{
			count++;
			if (!armed)
			{
				pubhex_takeoff.publish(to);
			}
			ros::Duration(7).sleep();		
		}
	}
	if (hexa == 1)
	{

		ros::Duration(10).sleep();
		int ini_alt = z_local[1];
		int count = 0;

		while(z_local[1] < (ini_alt + 2) && count < 4)
		{
			count++;
			if (!armed)
			{
				pubhex_takeoff.publish(to);
			}

			ros::Duration(7).sleep();
			
		}
	}
	if (hexa == 2)
	{
		ros::Duration(20).sleep();
		int ini_alt = z_local[2];
		int count = 0;

		while(z_local[2] < (ini_alt + 2) && count < 4)
		{
			count++;
			if (!armed)
			{
				pubhex_takeoff.publish(to);				
			}
			ros::Duration(7).sleep();
		}
	}


	// Give a few seconds to complete takeoff and reach prescribed altitudes.
    ros::Duration(5).sleep();
    
    return decision_making::TaskResult::SUCCESS();
}
//SEARCH----------------------------------------------------------------------------------------------------------------------------------------------------------
decision_making::TaskResult search(string name, const FSMCallContext &context, EventQueue &eventQueue)
{
    ss.str(std::string());
	ss << "[Hexacopter " << hexa << "] [State Machine] Searching for targets...";
	ocsmsg.data = ss.str();
	pubOCS.publish(ocsmsg);

    std_msgs::Empty srch;
    pubhex_search.publish(srch);

	// while(searching)
	// {

	// }	

	while(!search_complete_buffer)
		ros::Duration(2).sleep();

	

	ss.str(std::string());
	ss << "[Hexacopter " << hexa << "] [State Machine] Search pattern complete.";
	ocsmsg.data = ss.str();
	pubOCS.publish(ocsmsg);
    
	search_complete_buffer = false;
	search_complete = true;

    return decision_making::TaskResult::SUCCESS();
}

//TRACK-----------------------------------------------------------------------------------------------------------------------------------------------------------------
decision_making::TaskResult track(string name, const FSMCallContext &context, EventQueue &eventQueue)
{
	search_complete = false;

    ss.str(std::string());
	ss << "[Hexacopter " << hexa << "] [State Machine] Tracking to target at (x, y) = (" << tx[assignment[hexa]] << ", " << ty[assignment[hexa]] << ")";
	ocsmsg.data = ss.str();
	pubOCS.publish(ocsmsg);

    geometry_msgs::Pose trck;
    //
    trck.position.x = tx[assignment[hexa]];
    trck.position.y = ty[assignment[hexa]];
    trck.position.z = optAlt[hexa];
	
	ss.str(std::string());
	ss << "[Hexacopter " << hexa << "] [State Machine] ===" << assignment[hexa] << "	"<< tx[assignment[hexa]] << "   " << ty[assignment[hexa]] << "===";
	ocsmsg.data = ss.str();
	pubOCS.publish(ocsmsg);

	 
    pubhex_acquire.publish(trck);
	
	while( std::abs(x_local[hexa] - tx[assignment[hexa]]) > 1 &&  std::abs(y_local[hexa] - ty[assignment[hexa]]) > 1 )




    return decision_making::TaskResult::SUCCESS();
}

//LAND---------------------------------------------------------------------------------------------------------------------------------------------------------------
decision_making::TaskResult land(string name, const FSMCallContext &context, EventQueue &eventQueue)
{
	   
	   return decision_making::TaskResult::SUCCESS();
}

//CALLBACKS----------------------------------------------------------------------------------------------------------------------------------------------------------
void tlistCallback(const governor::target_list::ConstPtr &msg)
{
    //ROS_INFO("I heard: x = [%f], y = [%f] ", msg.x, msg.y);

    ttype.clear();

    for (int i = 0; i < 23; i++)
    {
	tx[i] = msg->x[i];
	ty[i] = msg->y[i];
	ttype.push_back(msg->Target_Type[i]);
	ided[i] = msg->IDed[i];
	served[i] = msg->Served[i];
	tpoints[i] = msg->Points[i];

	if (ided[i] == 1 && ttype[i] == "yellow" && served[i] == 0)
	{
	    unserved_moving_target_flag = i;
	}
	else
	{
	    unserved_moving_target_flag = 100; //This variable is either set to the target index or 100, 100 implying the flag is down.
	}
    }
}

void batteryCallback(const sensor_msgs::BatteryState msg)
{
	bat_volt = (bat_volt + msg.voltage)/2;
}
void pose0Callback(const nav_msgs::Odometry& msg)
{
    x_local[0] = msg.pose.pose.position.x;
    y_local[0] = msg.pose.pose.position.y;
    z_local[0] = msg.pose.pose.position.z;
}
void pose1Callback(const nav_msgs::Odometry& msg)
{
    x_local[1] = msg.pose.pose.position.x;
    y_local[1] = msg.pose.pose.position.y;
    z_local[1] = msg.pose.pose.position.z;
}
void pose2Callback(const nav_msgs::Odometry& msg)
{
    x_local[2] = msg.pose.pose.position.x;
    y_local[2] = msg.pose.pose.position.y;
    z_local[2] = msg.pose.pose.position.z;
}

void AssignmentCallback(const governor::assignment msg)
{
    assignment[0] = msg.UAV1;
    assignment[1] = msg.UAV2;
    assignment[2] = msg.UAV3;
}

void MavrosStateCallback(const mavros_msgs::State msg)
{
	armed = msg.armed;
	guided = msg.guided;
	flightmode = msg.mode;
}

void ContactSensorCallback(const std_msgs::Bool msg)
{ 
	if (msg.data)
	{
		contact_check = true;	
	}
}

void GPSCallback(const nav_msgs::Odometry msg)
{ 
	rtk_cov = msg.pose.covariance[0];
	sub9.shutdown();	
}

void PerspectiveCamCallback(const sensor_msgs::Image msg)
{ 
	perspective_cam_check = true;
	sub10.shutdown();	
}

void FisheyeCamCallback(const sensor_msgs::Image msg)
{ 
	fisheye_cam_check = true;	
	sub11.shutdown();
}

void StartCallback(const std_msgs::Empty msg)
{
	start_flag_buffer = true;
}

void AllRTLCallback(const std_msgs::Empty msg)
{
	all_rtl = true;
}

void AllLandCallback(const std_msgs::Empty msg)
{
	all_land = true;
}
void SearchCompleteCallback(const std_msgs::Empty msg)
{
	search_complete_buffer = true;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "stateMachChal3");
    ros_decision_making_init(argc, argv);

    LocalTasks::registrate("PreArmChecks", prearmchecks);
    LocalTasks::registrate("Standby", standby);
    LocalTasks::registrate("Takeoff", takeoff);
    LocalTasks::registrate("Search", search);
    LocalTasks::registrate("Track", track);
	LocalTasks::registrate("Land", land);
    
    ros::NodeHandle node;

    //    ros::Publisher eventPublisher0 = node.advertise<std_msgs::String>("", 1, false);

    ros::Publisher eventPublisher0 = node.advertise<std_msgs::String>("stateMachChal3/events", 100, false);

    pubhex_takeoff = node.advertise<std_msgs::Empty>("uav_control/takeoff", 100, false);
    pubhex_acquire = node.advertise<geometry_msgs::Pose>("uav_control/acquire", 100, false);
    pubhex_search = node.advertise<std_msgs::Empty>("uav_control/target/search", 100, false);
	pubhex_land = node.advertise<std_msgs::Empty>("uav_control/land", 100, false);
	pubhex_rtl = node.advertise<std_msgs::Empty>("uav_control/rtl", 100, false);

    pubhex_rcin = node.advertise<mavros_msgs::RCIn>("mavros/rc/in", 100, false);



	pubOCS = node.advertise<std_msgs::String>("/ocs/log_message", 100, false);

    ros::Subscriber sub1 = node.subscribe("/governor/tlist", 1, tlistCallback);

    ros::Subscriber sub2 = node.subscribe("mavros/battery", 1, batteryCallback);

    ros::Subscriber sub3 = node.subscribe("/hexacopter0/localizer/global_odom", 1, pose0Callback);
    ros::Subscriber sub4 = node.subscribe("/hexacopter1/localizer/global_odom", 1, pose1Callback);
    ros::Subscriber sub5 = node.subscribe("/hexacopter2/localizer/global_odom", 1, pose2Callback);

    ros::Subscriber sub6 = node.subscribe("/governor/assignment", 1, AssignmentCallback);

	ros::Subscriber sub7 = node.subscribe("mavros/state", 1, MavrosStateCallback);

	sub8 = node.subscribe("contact_sensors", 1, ContactSensorCallback);

	sub9 = node.subscribe("gps/rtkfix", 1, GPSCallback);

	sub10 = node.subscribe("perspective_cam", 1, PerspectiveCamCallback);
	sub11 = node.subscribe("fisheye_cam", 1, FisheyeCamCallback);

	sub12 = node.subscribe("/start", 1, StartCallback);
	sub13 = node.subscribe("/allrtl", 1, AllRTLCallback);
	sub14 = node.subscribe("/allland", 1, AllLandCallback);
	
	sub15 = node.subscribe("uav_control/target/search/complete", 1, SearchCompleteCallback);


    ros::AsyncSpinner spinner(2);
    spinner.start();

    //	ROS_INFO("Starting turnstile...");
    FsmChallenge3(NULL, new RosEventQueue());

    //	ros::spin();

    spinner.stop();

    return 0;
}
