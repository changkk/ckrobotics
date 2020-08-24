/*
  This is a quick solution to publish base_link odom wrt to global map
*/
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>


int main(int argc, char** argv) {
  ros::init(argc, argv, argv[1]);
  ros::NodeHandle nh;

  std::string baselinkId, globalMapId, pubTopic;
  std::string paramBaseLinkId = "/baselink_frame_id";
  std::string paramGlobalMapId = "/global_map_id";
  std::string paramGlobalOdomTopic = "/global_odom_topic";

  nh.param<std::string>(argv[1] + paramBaseLinkId, baselinkId, "base_link");
  nh.param<std::string>(argv[1] + paramGlobalMapId, globalMapId, "global_map");
  nh.param<std::string>(argv[1] + paramGlobalOdomTopic, pubTopic, "localizer/global_odom");

  std::cout << "Params set : " << baselinkId << " " << globalMapId << " " << pubTopic << std::endl;

  ros::Publisher odomPub = nh.advertise<nav_msgs::Odometry>(pubTopic, 100);
  tf::TransformListener tfListener;
  ros::Rate loopRate(20);
  
  while(true) {
    ros::Time now = ros::Time::now();
    tf::StampedTransform transform;
    nav_msgs::Odometry odom;

    if(tfListener.waitForTransform(globalMapId, baselinkId, ros::Time(0), ros::Duration(1))) {
      try {
        tfListener.lookupTransform(globalMapId, baselinkId, ros::Time(0), transform);

        odom.header.stamp = now;
        odom.header.frame_id = globalMapId;

        odom.pose.pose.position.x = transform.getOrigin().getX();
        odom.pose.pose.position.y = transform.getOrigin().getY();
        odom.pose.pose.position.z = transform.getOrigin().getZ();
        

        tf::quaternionTFToMsg(transform.getRotation(), odom.pose.pose.orientation);

        odomPub.publish(odom);

      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(0.05).sleep();
        continue;
      }
    } else {
      ROS_WARN("%s -> %s no transformation", baselinkId.c_str(), globalMapId.c_str());
    }
    loopRate.sleep();
  }
}
