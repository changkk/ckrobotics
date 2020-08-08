/*
  RTK-GPS simulator class header file
*/
#ifndef LOCALIZER_RTK_SIMULATOR_H
#define LOCALIZER_RTK_SIMULATOR_H

// ROS dependencies
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>

class RTKSimulator {
  private:
    static const int SUB_QUEUE_SIZE;
    static const int PUB_QUEUE_SIZE;

    std::string subUavPoseTopic;
    std::string pubUavPoseTopic;
    std::string pubUavOdomTopic;
    std::string uavModelName;

    double errorVar;
    double errorSD;

    ros::Subscriber poseSubscriber;
    ros::Publisher posePublisher;
    ros::Publisher odomPublisher;

    void poseSubscriberCb(const gazebo_msgs::ModelStates::ConstPtr&);
    void processUavPose(geometry_msgs::Pose);

  public:
    RTKSimulator(std::string, std::string,std::string, std::string, double);
    void start(ros::NodeHandle);
};

#endif
