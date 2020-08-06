#include <ros/ros.h>
// #include <tf2_ros,h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <vector>

double gimbal_location_enu[3] = {-0.2, 0.082, -0.397};

void poseCallback(const geometry_msgs::PoseStamped& msg){
  static tf2_ros::TransformBroadcaster br;
  
  

  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "observer_aircraft";
  // transformStamped.header.frame_id = "observer_aircraft"; 
  transformStamped.child_frame_id = "gimbal_camera";

  double q_x = msg.pose.orientation.x;
  double q_y = msg.pose.orientation.y;
  double q_z = msg.pose.orientation.z;
  double q_w = msg.pose.orientation.w;
  double pitch;

    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q_w * q_x + q_y * q_z);
    double cosr_cosp = +1.0 - 2.0 * (q_x * q_x + q_y * q_y);
    double roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q_w * q_y - q_z * q_x);
    if (fabs(sinp) >= 1)
      double pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
      double pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q_w * q_z + q_x * q_y);
    double cosy_cosp = +1.0 - 2.0 * (q_y * q_y + q_z * q_z);  
    double yaw = atan2(siny_cosp, cosy_cosp) - 0.2;

      // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    q_w = cy * cp * cr + sy * sp * sr;
    q_x = cy * cp * sr - sy * sp * cr;
    q_y = sy * cp * sr + cy * sp * cr;
    q_z = sy * cp * cr - cy * sp * sr;

  
  transformStamped.transform.translation.x = gimbal_location_enu[0];
  transformStamped.transform.translation.y = gimbal_location_enu[1];
  transformStamped.transform.translation.z = gimbal_location_enu[2];

  transformStamped.transform.rotation.x = q_x;
  transformStamped.transform.rotation.y = q_y;
  transformStamped.transform.rotation.z = q_z;
  transformStamped.transform.rotation.w = q_w;

  br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "gimbal_tf2_broadcaster");

  ros::NodeHandle node;
  
  std::vector<double> gimbal_location_list;
  std::string key;

  // if (node.searchParam("/gimbal_location_enu", key))
  // {
  //     node.getParam(key, gimbal_location_list);
  // }
  // for( unsigned i = 0; i < gimbal_location_list.size(); i++)
  // {
  //   gimbal_location_enu[i] = gimbal_location_list[i];
  // } 

  ros::Subscriber sub = node.subscribe("/gimbal_imu_angles", 10, &poseCallback);

  ros::spin();
  return 0;
};