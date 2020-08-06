/*
This code takes in the Pixhawk pose through the mavros/global_position/local topic and publishes the transfrom from the arming location orgin.
*/


#include <ros/ros.h>
// #include <tf2_ros,h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <piksi_rtk_msgs/BaselineNed.h>

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg){
  static tf2_ros::TransformBroadcaster br;  
  
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "arm_location";
  transformStamped.child_frame_id = "observer_aircraft";

  transformStamped.transform.translation.x = msg.pose.pose.position.x;
  transformStamped.transform.translation.y = msg.pose.pose.position.y;
  transformStamped.transform.translation.z = msg.pose.pose.position.z;
  
  transformStamped.transform.rotation.x = msg.pose.pose.orientation.x;
  transformStamped.transform.rotation.y = msg.pose.pose.orientation.y;
  transformStamped.transform.rotation.z = msg.pose.pose.orientation.z;
  transformStamped.transform.rotation.w = msg.pose.pose.orientation.w;

  br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "observer_aircraft_tf2_broadcaster");

  ros::NodeHandle private_node("~");
  ros::NodeHandle node;

  ros::Subscriber sub = node.subscribe("mavros/global_position/local", 10, &poseCallback);

  ros::spin();
  return 0;
};
