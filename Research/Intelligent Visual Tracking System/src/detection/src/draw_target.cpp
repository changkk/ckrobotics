#include <ros/ros.h>
// #include <tf2_ros,h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
//#include <piksi_rtk_msgs/BaselineNed.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <iostream>
#include <fstream>

ros::Publisher marker_pub;
visualization_msgs::Marker marker;
std::ofstream myfile;

double target_ground_truth_n;
double target_ground_truth_e;
double target_ground_truth_d;
double uncertainty_x, uncertainty_y, uncertainty_z;

double gimbal_location_ned[3] = {0, 0, 0};

//void poseCallback(const piksi_rtk_msgs::BaselineNed& msg){
  //static tf2_ros::TransformBroadcaster br;  
  
  // target_ground_truth_n=msg.n/1000;
  // target_ground_truth_e=msg.e/1000;
  // target_ground_truth_d=msg.d/1000;

  // geometry_msgs::TransformStamped transformStamped;
  
  // transformStamped.header.stamp = ros::Time::now();
  // transformStamped.header.frame_id = "rtk_base_station";
  // transformStamped.child_frame_id = "aircraft";

  // transformStamped.transform.translation.x = msg.n/1000;   //NED frame
  // transformStamped.transform.translation.y = msg.e/1000;  //NED frame
  // transformStamped.transform.translation.z = msg.d/1000;  //NED frame
  // // tf2::Quaternion q;
  // // q.setRPY(0, 0, msg->theta);
  // transformStamped.transform.rotation.x = 0;
  // transformStamped.transform.rotation.y = 0;
  // transformStamped.transform.rotation.z = 0;
  // transformStamped.transform.rotation.w = 1;

  //   // Set the scale of the marker -- 1x1x1 here means 1m on a side
  //   // transformStamped.scale.x = 1;
  //   // transformStamped.scale.y = 1;
  //   // transformStamped.scale.z = 1;


  // br.sendTransform(transformStamped);
//}

void target_cb(const geometry_msgs::PolygonStamped& msg)	
{

  if(!msg.polygon.points[0].x == 0)
  {
    // The x component of the first message is the size of the detections.
    for(int i=1; i<msg.polygon.points[0].x+1; i++)
    {
      double x = msg.polygon.points[i].x; // E
      double y = msg.polygon.points[i].y; // N
      double z = msg.polygon.points[i].z; // U

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.header.frame_id = "observer_aircraft";
      marker.header.stamp = ros::Time::now();

      marker.ns = "basic_shapes";
      marker.id = i+4;

      // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      marker.type = visualization_msgs::Marker::SPHERE;

      // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
      marker.action = visualization_msgs::Marker::ADD;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose.position.x = x;  //E of ENU frame
      marker.pose.position.y = y;  //N of ENU frame
      marker.pose.position.z = z;  //U of ENU frame
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;

      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;

      marker_pub.publish(marker);

      // double error_n = target_ground_truth_n - x;
      // double percent_error_n = (target_ground_truth_n - x)/target_ground_truth_n * 100;
      
      // double error_e = target_ground_truth_e - y;
      // double percent_error_e = (target_ground_truth_e - y)/target_ground_truth_e * 100;

      // double error_d = target_ground_truth_d - z;
      // double percent_error_d = (target_ground_truth_d - z)/target_ground_truth_d * 100;

      // double estimated_range = sqrt(x*x+y*y+z*z);
      // double actual_range = sqrt(target_ground_truth_n*target_ground_truth_n+target_ground_truth_e*target_ground_truth_e+target_ground_truth_d*target_ground_truth_d);
      
      // myfile << estimated_range<<" "<<actual_range<<" "<<error_n <<" "<<percent_error_n<<" "<< error_e<<" "<<percent_error_e<<" "<<error_d<<" "<<percent_error_d<<"\n";

    }
  }

    
	
}


void uncertainty_cb(const geometry_msgs::Twist& msg)	
{


		uncertainty_x = msg.linear.x; // E
		uncertainty_y = msg.linear.y; // N
    uncertainty_z = msg.linear.z; // U

    	
}

int main(int argc, char** argv){
  ros::init(argc, argv, "aircraft_tf2_broadcaster");

  ros::NodeHandle private_node("~");
  ros::NodeHandle node;
  
  myfile.open ("range.txt");

  //ros::Subscriber sub = node.subscribe("piksi/baseline_ned", 10, &poseCallback);
  ros::Subscriber target_sub = node.subscribe("/target_list_tf", 1000, &target_cb); // From epipolar_range_estimation.cpp
  ros::Subscriber uncertainty_sub = node.subscribe("/target_uncertainty", 1000, &uncertainty_cb); // From epipolar_range_estimation.cpp

  marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker_target", 1);

  ros::spin();
  myfile.close();
  return 0;
};
