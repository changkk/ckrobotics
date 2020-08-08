#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <string>
#include <vector>
#include "std_msgs/String.h"
#include "governor/target_list.h"
#include "governor/assignment.h"
#include <iostream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include "hungarian.h"



std::vector< std::vector<int> > array_to_matrix(int* m, int rows, int cols) {
	int i,j;
	std::vector< std::vector<int> > r;
	r.resize(rows, std::vector<int>(cols, 0));

	for(i=0;i<rows;i++)
	{
		for(j=0;j<cols;j++)
			r[i][j] = m[i*cols+j];
	}
	return r;
}

//Number of Targets (get from rostopic)
//	int no_targets = length(tx);
	int no_targets = 23;



//{ x pos | y pos | Target type | Served Bool | Points}

float x[3] = {0,0,0}, y[3] = {0,0,0}, z[3] = {0,0,0};
float tx[23], ty[23], tpoints[23];
float model_pose[3], model_velo[3];
int served[23];
std::vector<std::__cxx11::basic_string<char> > model_name[7];


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


void batteryCallback(const sensor_msgs::BatteryState msg)
{
	ROS_INFO("Battery at [%f] volts.", msg.voltage);  
	//x = msg.x;
	//y = msg.y;
}


void pose0Callback(const geometry_msgs::PoseStamped msg)
{
	//ROS_INFO("Robot at : x = [%f], y = [%f], z = [%f] ", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
	//x = msg.x;
	//y = msg.y;
	x[0] = msg.pose.position.x;
	y[0] = msg.pose.position.y;
	z[0] = msg.pose.position.z;
}
void pose1Callback(const geometry_msgs::PoseStamped msg)
{
	//ROS_INFO("Robot at : x = [%f], y = [%f], z = [%f] ", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
	//x = msg.x;
	//y = msg.y;
	x[1] = msg.pose.position.x;
	y[1] = msg.pose.position.y;
	z[1] = msg.pose.position.z;
}
void pose2Callback(const geometry_msgs::PoseStamped msg)
{
	//ROS_INFO("Robot at : x = [%f], y = [%f], z = [%f] ", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
	//x = msg.x;
	//y = msg.y;
	x[2] = msg.pose.position.x;
	y[2] = msg.pose.position.y;
	z[2] = msg.pose.position.z;
}





int main(int argc, char **argv)
{

	ros::init(argc, argv, "governor");

	ros::NodeHandle n;

	ros::Subscriber sub1 = n.subscribe("/governor/tlist", 1, tlistCallback);

	//ros::Subscriber sub2 = n.subscribe("/mavros/battery", 100, batteryCallback);


	ros::Subscriber sub3 = n.subscribe("/hexacopter0/mavros/local_position/pose", 1, pose0Callback);
	ros::Subscriber sub4 = n.subscribe("/hexacopter1/mavros/local_position/pose", 1, pose1Callback);
	ros::Subscriber sub5 = n.subscribe("/hexacopter2/mavros/local_position/pose", 1, pose2Callback);
	
	ros::Publisher pub = n.advertise<governor::assignment>("assignment", 100);
	
	ros::Publisher pubhex0 = n.advertise<geometry_msgs::Pose>("/hexacopter0/uav_control/waypoint", 100);
	ros::Publisher pubhex1 = n.advertise<geometry_msgs::Pose>("/hexacopter1/uav_control/waypoint", 100);
	ros::Publisher pubhex2 = n.advertise<geometry_msgs::Pose>("/hexacopter2/uav_control/waypoint", 100);
	ros::Publisher pubTarg = n.advertise<governor::target_list>("/governor/tlist", 100);
	
	
	//ros::spin();

	ros::Rate loop_rate(4);

	//Number of Active UAV's
	int activeUAV = 3; 

	
	
	//Dropzone Location in local frame
	int dzx = -35, dzy = 55;

	//Initialize Cost matrix
	double cost_matrix[3][23] = {0}; 
	int cm[3*23] = {0};

	//Assign weights to each cost matrix term.
	double w_bat = 1, w_distance = 0.04, w_type = 1, w_points = 100, w_served = -100, max_dist = 250*2;

//	max_dist = pow(pow((x[i]-tx[j]),2) + pow((y[i]-ty[j]),2),0.5)
		
	//UAV Positions
//	float ux[3] = {0}, uy[3] = {0}, uz[3] = {0}, 
	double distance = 0;
	int cost = 0, loopcount = 0;
	const vector<vector<int> > assign;
	int vassign[3] = {0, 0, 0};
	float txLocal[23], tyLocal[23], tpointsLocal[23];
	int servedLocal[23] = {0}, runOnce = 0;
	
//	ROS_INFO("Cost Matrix B");

	
	
	
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------	
	//Target Allocation Calculations

	while(ros::ok()){

		if (runOnce == 0){
			for (int j = 0; j < 23; j++){
				txLocal[j] = tx[j];
				tyLocal[j] = ty[j];
				tpointsLocal[j] = tpoints[j];
//				servedLocal[j] = served[j];
			}
//			runOnce = 1;					
		}
		
//		ux = x;		
//		uy = y;
//		uz = z;
				
//		for (int i = 0; i < activeUAV ; i++){
//			std::cout << x[i] << " " << y[i] << " "<< z[i] << std::endl;
//		}
		
//		std::cout << std::endl;

//------------------------------------------------------------------------------------------------------------------------------------		
//		Check if target Reached		
		
		for (int i = 0; i < activeUAV ; i++){
			if (( abs(x[i] - tx[vassign[i]]) <= 1) && ( abs(y[i] - ty[vassign[i]]) <= 1 )){
//				txLocal[vassign[i]] = 5;
//				tyLocal[vassign[i]] = 0;
				
//				governor::target_list msgTarg;
//				msgTarg.Served[vassign[i]] = 1;
//				pubTarg.publish(msgTarg);
//				
				
				servedLocal[vassign[i]] = 1;
//				std::cout << "served? = " << servedLocal[vassign[i]] << std::endl;

			}
		}
		
		
		
		int k = 0;
		
		for (int i = 0; i < activeUAV ; i++){
			for (int j = 0; j < 23; j++){

				//Distance from UAV location to target to dropzone				
				distance = pow(pow((x[i]-txLocal[j]),2) + pow((y[i]-tyLocal[j]),2),0.5) + 2*abs(z[i]) + pow(pow((dzx-txLocal[j]),2) + pow((dzy-tyLocal[j]),2),0.5) + 2*abs(z[i]-4);

//				cost_matrix[i][j] = floor(w_distance*(-distance*distance + pow(max_dist,2)))-servedLocal[j]*floor(w_distance*(-distance*distance + pow(max_dist,2)));//
//				cost_matrix[i][j] = i*j - i*j*servedLocal[j]+1;
//				cost_matrix[i][j] = 10*i*j - 10*i*j*servedLocal[j]+i+1;
								
				if (runOnce == 0){
					cost_matrix[i][j] = j*distance/14;
				}
				
				cm[k] = cost_matrix[i][j];
				k++;
				std::cout << cost_matrix[i][j] << "  ";
			}
			std::cout << std::endl;
			
		}
		runOnce = 1;
		std::cout << std::endl;

	
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		std::vector< std::vector<int> > m = array_to_matrix(cm,3,23);

		/* initialize the hungarian_problem using the cost matrix*/
		Hungarian hungarian(m, activeUAV,no_targets, HUNGARIAN_MODE_MAXIMIZE_UTIL) ;
//		Hungarian hungarian(m, activeUAV,no_targets, HUNGARIAN_MODE_MINIMIZE_COST) ;

		/* some output */
//		fprintf(stderr, "cost-matrix:");
//		hungarian.print_cost();

		/* solve the assignement problem */
		hungarian.solve();

		/* some output */
//		fprintf(stderr, "assignment:");
//		hungarian.print_assignment();

		
		vector<vector<int> > assign;
		
		assign = hungarian.assignment();
		
		
		cost = hungarian.cost();
		
//		fprintf(stderr, "cost: ");
//		std::cout << std::endl;
//		fprintf(stderr, "%5d ",cost);
//		std::cout << std::endl;
//		std::cout << std::endl;
//		fprintf(stderr, "Assignment: ");
//		std::cout << std::endl;
		
		
		int p = 0, q = 0;
		std::vector<int> temp;
		std::vector<int>::iterator it;
		int needle1[1] = {1};
		
		for (std::vector<std::vector<int> >::const_iterator iter1 = assign.begin(); iter1 != assign.end(); ++iter1) {
			for (std::vector<int>::const_iterator iter2 = iter1->begin(); iter2 != iter1->end(); ++iter2) {
//				std::cout << *iter2 << "	";
				temp.push_back(*iter2);
				p++;
			}
//			std::cout << std::endl;
			p = 0;
			it = std::search (temp.begin(), temp.end(), needle1, needle1+1);
			if (q < 3){
				vassign[q] = (it-temp.begin());
				//std::cout << (it-temp.begin());
			}
			temp.clear();
			q++;
		}
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		std::cout << std::endl << "Current Assignment (Target Number): ";
		
		for (int i = 0; i < activeUAV ; i++){
			std::cout << vassign[i] << "	";
		}
	
		std::cout << std::endl << "Served:" << std::endl;
		
		for (int j = 0; j < 23; j++){
			 std::cout << servedLocal[j] << "	";
		}
		std::cout << std::endl;
		
//		for (int j = 0; j < 3; j++){
//			std::cout << assign[j];
		
//	    std::copy(assign.begin(), assign.end(), std::ostream_iterator<char>(std::cout, " "));

//		for( assign::size_type i=0; i<assign.size(); ++i)
//		  std::cout << assign[i] << ' ';
		
//		std::cout << std::endl;
//		std::cout << "------------------------------------------";
//		std::cout << std::endl;
//		hungarian.print_status();
		

		
		
		
		
//------------------------------------------------------------------------------------------------------------------------------------		
//		Publish Everything
		
		
		
		governor::assignment assignMsg;
//		assignMsg.UAV1.push_back(vassign[0]);
//		assignMsg.UAV2.push_back(vassign[1]);
//		assignMsg.UAV3.push_back(vassign[2]);
		assignMsg.UAV1 = vassign[0];
		assignMsg.UAV2 = vassign[1];
		assignMsg.UAV3 = vassign[2];
		pub.publish(assignMsg);

		geometry_msgs::Pose UAV0set;
		UAV0set.position.x = tx[vassign[0]];
		UAV0set.position.y = ty[vassign[0]];
		UAV0set.position.z = 3;
		UAV0set.orientation.w = 1;
		pubhex0.publish(UAV0set);
		
		geometry_msgs::Pose UAV1set;
		UAV1set.position.x = tx[vassign[1]];
		UAV1set.position.y = ty[vassign[1]];
		UAV1set.position.z = 5;
		UAV1set.orientation.w = 1;
		pubhex1.publish(UAV1set);
		
		geometry_msgs::Pose UAV2set;
		UAV2set.position.x = tx[vassign[2]];
		UAV2set.position.y = ty[vassign[2]];
		UAV2set.position.z = 7;
		UAV2set.orientation.w = 1;
		pubhex2.publish(UAV2set);
		
		
		
		loopcount++;
		ros::spinOnce();   	    
		loop_rate.sleep();
		
		

	}


	return 0;
}







