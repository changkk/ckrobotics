#include <ros/ros.h>
// #include <tf2_ros,h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <vector>

double gimbal_location_enu[3] = {0, 0, 0};

void poseCallback(const geometry_msgs::PoseStamped& msg){
  static tf2_ros::TransformBroadcaster br;
  
  

  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "rtk_base_station";
  // transformStamped.header.frame_id = "observer_aircraft"; 
  transformStamped.child_frame_id = "gimbal_camera";

  
  transformStamped.transform.translation.x = gimbal_location_enu[0];
  transformStamped.transform.translation.y = gimbal_location_enu[1];
  transformStamped.transform.translation.z = gimbal_location_enu[2];

  transformStamped.transform.rotation.x = msg.pose.orientation.x;
  transformStamped.transform.rotation.y = msg.pose.orientation.y;
  transformStamped.transform.rotation.z = msg.pose.orientation.z;
  transformStamped.transform.rotation.w = msg.pose.orientation.w;

  br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "gimbal_tf2_broadcaster");

  ros::NodeHandle node;
  
  std::vector<double> gimbal_location_list;
  std::string key;

  if (node.searchParam("/gimbal_location_enu", key))
  {
      node.getParam(key, gimbal_location_list);
  }
  for( unsigned i = 0; i < gimbal_location_list.size(); i++)
  {
    gimbal_location_enu[i] = gimbal_location_list[i];
  } 

  ros::Subscriber sub = node.subscribe("gimbal_imu_angles", 10, &poseCallback);

  ros::spin();
  return 0;
};