#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>


double omnicam_location_ned[3] = {0, 0, 0};

void poseCallback(const sensor_msgs::Imu& msg){
  static tf2_ros::TransformBroadcaster br;
  
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "rtk_base_station";
  transformStamped.child_frame_id = "omnicam";

  // NED to ENU conversion handled here - To go from Piksi standard to ROS standard
  transformStamped.transform.translation.x = omnicam_location_ned[1];
  transformStamped.transform.translation.y = omnicam_location_ned[0];
  transformStamped.transform.translation.z = -omnicam_location_ned[2];
  // tf2::Quaternion q;
  // q.setRPY(0, 0, msg->theta);
  transformStamped.transform.rotation.x = msg.orientation.x;
  transformStamped.transform.rotation.y = msg.orientation.y;
  transformStamped.transform.rotation.z = msg.orientation.z;
  transformStamped.transform.rotation.w = msg.orientation.w;

  br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "omnicam_tf2_broadcaster");


  ros::NodeHandle node;
  
  std::vector<double> omnicam_location_list;
  std::string key;

  if (node.searchParam("/omnicam_location_ned", key))
  {
      node.getParam(key, omnicam_location_list);
  }
  for( unsigned i = 0; i < omnicam_location_list.size(); i++)
  {
    omnicam_location_ned[i] = omnicam_location_list[i];
  } 

  ros::Subscriber sub = node.subscribe("mavros/imu/data", 10, &poseCallback);
  

  ros::spin();
  return 0;
};