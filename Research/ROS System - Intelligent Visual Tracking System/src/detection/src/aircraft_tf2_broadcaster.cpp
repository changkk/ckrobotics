#include <ros/ros.h>
// #include <tf2_ros,h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <piksi_rtk_msgs/BaselineNed.h>


double gimbal_location_ned[3] = {0, 0, 0};

void poseCallback(const piksi_rtk_msgs::BaselineNed& msg){
  static tf2_ros::TransformBroadcaster br;  
  

  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "rtk_base_station";
  transformStamped.child_frame_id = "aircraft";

  // NED to ENU conversion handled here - To go from Piksi standard to ROS standard
  transformStamped.transform.translation.x = msg.e/1000;
  transformStamped.transform.translation.y = msg.n/1000;
  transformStamped.transform.translation.z = -msg.d/1000;
  // tf2::Quaternion q;
  // q.setRPY(0, 0, msg->theta);
  transformStamped.transform.rotation.x = 0;
  transformStamped.transform.rotation.y = 0;
  transformStamped.transform.rotation.z = 0;
  transformStamped.transform.rotation.w = 1;

  br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "aircraft_tf2_broadcaster");

  ros::NodeHandle private_node("~");
  ros::NodeHandle node;
  
  // std::vector<double> gimbal_location_list;
  // ros::param::get("/gimbal_location_ned", gimbal_location_list);

  // for( int i = 0; i < 3; i++)
  // {
  //   gimbal_location_ned[i] = gimbal_location_list[i];
  // } 

  ros::Subscriber sub = node.subscribe("piksi/baseline_ned", 10, &poseCallback);

  ros::spin();
  return 0;
};
