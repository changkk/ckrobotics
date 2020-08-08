/*
  Pose Sync class header file
*/
#ifndef LOCALIZER_POSE_SYNC
#define LOCALIZER_POSE_SYNC

// ROS dependencies
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

// C++ dependencies
#include <boost/thread/thread.hpp>

class PoseSync {
  private :
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, nav_msgs::Odometry> odomSyncPolicy;

    static const int SUB_IMU_QUEUE_SIZE;
    static const int SUB_RTKGPS_QUEUE_SIZE;
    static const int SYNC_QUEUE_SIZE;
    static const int PUB_ODOM_QUEUE_SIZE;

    struct tfStruct{
      float x;
      float y;
      float z;
      float roll;
      float pitch;
      float yaw;
    } tfRtkRecieverBaselink;

    std::string subImuTopic;
    std::string subRtkgpsTopic;
    std::string pubOdomTopic;
    std::string rtkBaseFrameId;
    std::string rtkRecieverFrameId;
    std::string armFrameId;
    std::string baseLinkFrameId;

    boost::thread *tfThread;

    message_filters::Subscriber<sensor_msgs::Imu> *imuSubscriber;
    message_filters::Subscriber<nav_msgs::Odometry> *rtkgpsSubscriber;
    message_filters::Synchronizer<odomSyncPolicy> *odomSync;

    ros::Publisher odomPublisher;

    tf::TransformBroadcaster odomBroadcaster;

    void odomSubscriberCb(const sensor_msgs::Imu::ConstPtr&, const nav_msgs::Odometry::ConstPtr&);

    void setTFStructFromStr(std::string);

    void tfProcess();

  public :
    PoseSync(std::string, std::string, std::string, std::string, std::string, std::string, std::string, std::string);

    void start(ros::NodeHandle);

    virtual ~PoseSync();
};

#endif
