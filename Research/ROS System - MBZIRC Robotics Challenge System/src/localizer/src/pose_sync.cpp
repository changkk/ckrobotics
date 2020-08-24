/*
  Pose Sync implementation
*/
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// Project dependencies
#include "localizer/pose_sync.hpp"

// C++ dependencies
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>

// Initialize static constants
const int PoseSync::SUB_IMU_QUEUE_SIZE = 100;
const int PoseSync::SUB_RTKGPS_QUEUE_SIZE = 100;
const int PoseSync::SYNC_QUEUE_SIZE = 100;
const int PoseSync::PUB_ODOM_QUEUE_SIZE = 100;

PoseSync::PoseSync(std::string subImuTopic, std::string subRtkgpsTopic, std::string pubOdomTopic, std::string rtkBaseFrameId, std::string rtkRecieverFrameId, std::string armFrameId, std::string baseLinkFrameId, std::string tfRtkRecieverBaselink) {
  this->subImuTopic = subImuTopic;
  this->subRtkgpsTopic = subRtkgpsTopic;
  this->pubOdomTopic = pubOdomTopic;
  this->rtkBaseFrameId = rtkBaseFrameId;
  this->rtkRecieverFrameId = rtkRecieverFrameId;
  this->armFrameId = armFrameId;
  this->baseLinkFrameId = baseLinkFrameId;

  this->setTFStructFromStr(tfRtkRecieverBaselink);

  std::cout << "PoseSync Initialized" << std::endl;
}

PoseSync::~PoseSync() {
  std::cout << "Destructor on PoseSync called..." << std::endl;

  this->tfThread->interrupt();
  delete this->tfThread;
}

void PoseSync::start(ros::NodeHandle nh) {
  this->imuSubscriber = new message_filters::Subscriber<sensor_msgs::Imu>(nh, this->subImuTopic, this->SUB_IMU_QUEUE_SIZE);
  
  this->rtkgpsSubscriber = new message_filters::Subscriber<nav_msgs::Odometry>(nh, this->subRtkgpsTopic, this->SUB_RTKGPS_QUEUE_SIZE);

  this->odomSync = new message_filters::Synchronizer<odomSyncPolicy>(odomSyncPolicy(this->SYNC_QUEUE_SIZE),*(this->imuSubscriber), *(this->rtkgpsSubscriber));

  this->odomSync->registerCallback(boost::bind(&PoseSync::odomSubscriberCb, this, _1, _2));

  this->odomPublisher = nh.advertise<nav_msgs::Odometry>(this->pubOdomTopic, this->PUB_ODOM_QUEUE_SIZE);

  this->tfThread = new boost::thread(boost::bind(&PoseSync::tfProcess, this));
}

void PoseSync::odomSubscriberCb(const sensor_msgs::Imu::ConstPtr& imuMsg, const nav_msgs::Odometry::ConstPtr& rtkgpsMsg) {
  geometry_msgs::TransformStamped odomTf;
  nav_msgs::Odometry gpsImuOdom;
  ros::Time currentTime = ros::Time::now();

  odomTf.header.stamp = currentTime;
  odomTf.header.frame_id = this->rtkBaseFrameId;
  odomTf.child_frame_id = this->rtkRecieverFrameId;
  odomTf.transform.translation.x = rtkgpsMsg->pose.pose.position.x;
  odomTf.transform.translation.y = rtkgpsMsg->pose.pose.position.y;
  odomTf.transform.translation.z = rtkgpsMsg->pose.pose.position.z;
  odomTf.transform.rotation = imuMsg->orientation;

  odomBroadcaster.sendTransform(odomTf);  

  gpsImuOdom.header.stamp = currentTime;
  gpsImuOdom.header.frame_id = this->rtkBaseFrameId;
  gpsImuOdom.child_frame_id = this->rtkRecieverFrameId;
  gpsImuOdom.pose.pose.position = rtkgpsMsg->pose.pose.position;
  gpsImuOdom.pose.pose.orientation = imuMsg->orientation;
  gpsImuOdom.pose.covariance = rtkgpsMsg->pose.covariance;

  this->odomPublisher.publish(gpsImuOdom);
}

void PoseSync::setTFStructFromStr(std::string tfRtkRecieverBaselink) {
  std::vector<std::string> tokens;
  boost::split(tokens, tfRtkRecieverBaselink, boost::is_any_of(" "));
  this->tfRtkRecieverBaselink.x = boost::lexical_cast<float>(tokens[0]);
  this->tfRtkRecieverBaselink.y = boost::lexical_cast<float>(tokens[1]);
  this->tfRtkRecieverBaselink.z = boost::lexical_cast<float>(tokens[2]);
  this->tfRtkRecieverBaselink.yaw = boost::lexical_cast<float>(tokens[3]);
  this->tfRtkRecieverBaselink.pitch = boost::lexical_cast<float>(tokens[4]);
  this->tfRtkRecieverBaselink.roll = boost::lexical_cast<float>(tokens[5]);
}

/*  Continuously do
    1. Get fixed base_link -> rtk_reciever tf
    2. Calculate rtk_reciever -> arm
    3. Broadcast rtk_reciever -> arm
*
void PoseSync::tfProcess() {
  std::cout << "Starting tf process" << std::endl;
  tf::TransformListener tfListener;
  tf::TransformBroadcaster tfBroadcaster;

  tf::Transform tfRtkRecieverBaselink(
    tf::createQuaternionFromRPY(
      this->tfRtkRecieverBaselink.roll,
      this->tfRtkRecieverBaselink.pitch,
      this->tfRtkRecieverBaselink.yaw),
    tf::Vector3(
      this->tfRtkRecieverBaselink.x,
      this->tfRtkRecieverBaselink.y,
      this->tfRtkRecieverBaselink.z
    ));

  tf::Stamped<tf::Pose> tfBaseLinkRtkRecieverPoseStamped;
  tf::Stamped<tf::Pose> tfArmRtkRecieverPoseStamped;
  tf::Transform tfArmRtkReciever;
  tf::StampedTransform tfRtkRecieverArmStamped;

  ros::Time now;

  ros::Rate rate(10.0);

  while(true) {
    boost::this_thread::interruption_point();

    try {
      now = ros::Time::now();

      // 1. Calculate base_link -> rtk_reciever
      tfBaseLinkRtkRecieverPoseStamped = tf::Stamped<tf::Pose>(tfRtkRecieverBaselink.inverse(), now, this->baseLinkFrameId);

      if (tfListener.waitForTransform(this->armFrameId, this->baseLinkFrameId,now, ros::Duration(1))) {
        // 2. Calculate rtk_reciever -> arm
        tfListener.transformPose(this->armFrameId, tfBaseLinkRtkRecieverPoseStamped, tfArmRtkRecieverPoseStamped);

        tfArmRtkReciever = tf::Transform(
          tf::Quaternion(tfArmRtkRecieverPoseStamped.getRotation()),
          tf::Point(tfArmRtkRecieverPoseStamped.getOrigin()));

        tfRtkRecieverArmStamped = tf::StampedTransform(tfArmRtkReciever.inverse(), now, this->rtkRecieverFrameId, this->armFrameId);

        // 3. Broadcast rtk_reciever -> arm
        tfBroadcaster.sendTransform(tfRtkRecieverArmStamped);
      } else {
        ROS_WARN("Cannot find transformation from frame %s to %s", this->baseLinkFrameId.c_str(), this->armFrameId.c_str());
      }
    } catch (tf::TransformException ex) {
      ROS_ERROR("TFException - %s", ex.what());
    }

    rate.sleep();
  }
}*/

/*  Continuously do
    1. Get fixed base_link -> rtk_reciever tf
    2. Calculate rtk_reciever -> arm
    3. Calculate gps -> arm
    4. Broadcast gps -> arm if available
    5. Broadcast old gps -> arm if not available
*/
void PoseSync::tfProcess() {
  std::cout << "Starting tf process" << std::endl;
  tf::TransformListener tfListener;
  tf::TransformBroadcaster tfBroadcaster;

  tf::Transform tfRtkRecieverBaselink(
    tf::createQuaternionFromRPY(
      this->tfRtkRecieverBaselink.roll,
      this->tfRtkRecieverBaselink.pitch,
      this->tfRtkRecieverBaselink.yaw),
    tf::Vector3(
      this->tfRtkRecieverBaselink.x,
      this->tfRtkRecieverBaselink.y,
      this->tfRtkRecieverBaselink.z
    ));

  tf::Stamped<tf::Pose> tfBaseLinkRtkRecieverPoseStamped;
  tf::Stamped<tf::Pose> tfArmRtkRecieverPoseStamped;
  tf::Stamped<tf::Pose> tfRtkRecieverArmPoseStamped;
  tf::Stamped<tf::Pose> tfGpsArmPoseStamped;
  tf::Transform tfArmRtkReciever;
  tf::Transform tfGpsArm;
  tf::StampedTransform tfRtkRecieverArmStamped;
  tf::StampedTransform tfGpsArmStamped;

  ros::Time now;
  ros::Rate rate(10.0);

  bool hasTransform = false;

  while(true) {
    boost::this_thread::interruption_point();

    try {
      now = ros::Time::now();

      // 1. Calculate base_link -> rtk_reciever
      tfBaseLinkRtkRecieverPoseStamped = tf::Stamped<tf::Pose>(tfRtkRecieverBaselink.inverse(), now, this->baseLinkFrameId);

      if (tfListener.waitForTransform(this->armFrameId, this->baseLinkFrameId,now, ros::Duration(1))) {
        // 2. Calculate rtk_reciever -> arm
        tfListener.transformPose(this->armFrameId, tfBaseLinkRtkRecieverPoseStamped, tfArmRtkRecieverPoseStamped);

        tfArmRtkReciever = tf::Transform(
          tf::Quaternion(tfArmRtkRecieverPoseStamped.getRotation()),
          tf::Point(tfArmRtkRecieverPoseStamped.getOrigin()));

        tfRtkRecieverArmPoseStamped = tf::Stamped<tf::Pose>(tfArmRtkReciever.inverse(), now, this->rtkRecieverFrameId);

        if(tfListener.waitForTransform(this->rtkBaseFrameId, this->rtkRecieverFrameId, now, ros::Duration(0.2))) {
          // 3. Calculate gps -> arm
          tfListener.transformPose(this->rtkBaseFrameId, tfRtkRecieverArmPoseStamped, tfGpsArmPoseStamped);

          tfGpsArm = tf::Transform(
            tf::Quaternion(tfGpsArmPoseStamped.getRotation()),
            tf::Point(tfGpsArmPoseStamped.getOrigin()));

          tfGpsArmStamped = tf::StampedTransform(tfGpsArm, now, this->rtkBaseFrameId, this->armFrameId);

          // 4. Broadcast gps -> arm
          tfBroadcaster.sendTransform(tfGpsArmStamped);
          hasTransform = true;
        } else if (hasTransform) {
          // Broadcast old gps -> arm
          tfBroadcaster.sendTransform(tfGpsArmStamped);
        } else {
          ROS_WARN("Cannot find transformation from gps to rtk_reciever and no previous transformations exist!");
        }
      } else {
        ROS_WARN("Cannot find transformation from frame %s to %s", this->baseLinkFrameId.c_str(), this->armFrameId.c_str());
      }
    } catch (tf::TransformException ex) {
      ROS_ERROR("TFException - %s", ex.what());
    }

    rate.sleep();
  }
}