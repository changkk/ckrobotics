#include <ros/ros.h>
// #include <tf2_ros,h>
// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>

double gimbal_location_enu[3] = {0, 0, 0};

void callback( const geometry_msgs::PoseStamped& msg){
  static tf2_ros::TransformBroadcaster br;  
  

  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "rtk_base_station";
  transformStamped.child_frame_id = "target";

  // NED to ENU conversion has already been handled by mavros! Do not repeat here! Scaling from mm to meters done here.
  transformStamped.transform.translation.x = msg.pose.position.x/1000;
  transformStamped.transform.translation.y = msg.pose.position.y/1000;
  transformStamped.transform.translation.z = msg.pose.position.z/1000;


  transformStamped.transform.rotation.x = msg.pose.orientation.x;
  transformStamped.transform.rotation.y = msg.pose.orientation.y;
  transformStamped.transform.rotation.z = msg.pose.orientation.z;
  transformStamped.transform.rotation.w = msg.pose.orientation.w;

  br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "aircraft_tf2_broadcaster");

  ros::NodeHandle private_node("~");
  ros::NodeHandle node;
  
  std::vector<double> gimbal_location_list;
  ros::param::get("/gimbal_location_enu", gimbal_location_list);

  for( int i = 0; i < 3; i++)
  {
    gimbal_location_enu[i] = gimbal_location_list[i];
  } 

  ros::Subscriber sub = node.subscribe("mavros/local_position/pose", 10, &callback);
  
  // message_filters::Subscriber<sensor_msgs::Imu> orient_sub(nh, "mavros/imu/data", 1);
  // message_filters::Subscriber<geometry_msgs::PoseStamped> baseline_sub(nh, "mavros/local_position/pose", 1);
  // message_filters::TimeSynchronizer<sensor_msgs::Imu, geometry_msgs::PoseStamped> sync(orient_sub, baseline_sub, 10);
  // sync.registerCallback(boost::bind(&callback, _1, _2));
  



  ros::spin();
  return 0;
};
