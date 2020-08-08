#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>

void cb(const geometry_msgs::Pose::ConstPtr&);

int main(int argc, char** argv) {
  ros::init(argc, argv, "wp_control");
  ros::NodeHandle nh;
 
  ros::Subscriber sub = nh.subscribe("/wp_test", 100, cb);

  ros::spin();

  return 0;
}


void cb(const geometry_msgs::Pose::ConstPtr& msg) {
/*
    1. Try to Transform the waypoint into arm_origin frame
    2. On success : Execute waypoint
       On failure : Ignore **for now!!!**
  */
  std::cout << "Got message" << std::endl;
  tf::TransformListener tfListener;
  ros::Time startTime = ros::Time::now();
  ros::Duration tenSecs(10.0);
  ros::Time now = startTime;

  bool success = false;
  std_msgs::String errorMsg;

  geometry_msgs::PoseStamped poseStampedGlobal;
  geometry_msgs::PoseStamped poseStampedArm;
  poseStampedGlobal.header.frame_id = "global_map";
  poseStampedGlobal.pose = *msg;

  std::cout << "Starting loop" << std::endl;
  while (now - startTime < tenSecs) {
    try {
        // 1. Transform the waypoint into arm_origin frame
      if (tfListener.waitForTransform("global_map", "base_link", ros::Time(0), ros::Duration(0.5))) {
        tfListener.transformPose("arm_origin", poseStampedGlobal, poseStampedArm);

        std::cout << "x:" << poseStampedArm.pose.position.x << "y: " << poseStampedArm.pose.position.y << " z: " << poseStampedArm.pose.position.z << std::endl;

        // 2. Execute waypoint
        // this->mWaypoint = poseStampedArm.pose;
        // this->mMavrosAdapter.executeMoveToWaypoint(this->mWaypoint);
        // this->mEnRouteToWaypoint = true;
        std::cout << "All good" << std::endl;
        success = true;
        break;
      }
    } catch (tf::TransformException ex) {
      ROS_ERROR("TFException - %s", ex.what());
      success = false;
      errorMsg.data = "Error " + std::string(ex.what()) + " transforming from " + "global_origin" + " -> " + "arm_origin" + "at uav_control "; 
      break;
    }
    now = ros::Time::now();
  }

  if (!success) {
    // this->mLoggerPublisher.publish(errorMsg);
    std::cout << errorMsg << std::endl;
  }
  std::cout << "Done" << std::endl;
}
