/*
  RTK-GPS simulator implementation
*/
// ROS dependencies
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

// Project dependencies
#include <localizer/rtk_simulator.h>

// C++ dependencies
#include <math.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

// Initialize static constants
const int RTKSimulator::SUB_QUEUE_SIZE = 100;
const int RTKSimulator::PUB_QUEUE_SIZE = 100;

RTKSimulator::RTKSimulator(std::string subUavPoseTopic, std::string pubUavPoseTopic, std::string pubUavOdomTopic, std::string uavModelName, double errorVar) {
  this->subUavPoseTopic = subUavPoseTopic;
  this->pubUavPoseTopic = pubUavPoseTopic;
  this->pubUavOdomTopic = pubUavOdomTopic;
  this->uavModelName = uavModelName;
  this->errorVar = errorVar;
  this->errorSD = sqrt(errorVar);
}

void RTKSimulator::start(ros::NodeHandle nh) {
  /* 
    1. Subscribe to UAV pose topic
    2. Calculate pose with gaussian noise
    3. Publish UAV pose with noise

    Subscriber takes care of 1,2,3
    Publisher will be called within subscriber CB
  */
  this->poseSubscriber = nh.subscribe(
    this->subUavPoseTopic,
    this->SUB_QUEUE_SIZE,
    &RTKSimulator::poseSubscriberCb,
    this
  );

  this->posePublisher = nh.advertise<geometry_msgs::Pose>(
    this->pubUavPoseTopic,
    this->PUB_QUEUE_SIZE
  );

  this->odomPublisher = nh.advertise<nav_msgs::Odometry>(
    this->pubUavOdomTopic,
    this->PUB_QUEUE_SIZE
  );
}

void RTKSimulator::poseSubscriberCb(const gazebo_msgs::ModelStates::ConstPtr& msg) {
  /*
    1. Identify the hexacoptor index
    2. Extract data and call method to process
  */
  int hexIdx = -1;

  for (int i=0; i<msg->name.size(); i++) {
    if (this->uavModelName == msg->name[i]) {
      hexIdx = i;
      break;
    }
  }

  if (hexIdx == -1) {
    ROS_WARN("RTK Simulator: Hexacoptor model not detected!");
  } else {
    this->processUavPose(msg->pose[hexIdx]);
  }
}

void RTKSimulator::processUavPose(geometry_msgs::Pose uavPose) {
  boost::mt19937 *randNumGen = new boost::mt19937();

  randNumGen->seed(time(NULL));

  boost::normal_distribution<> distribution(0, this->errorSD);
  boost::variate_generator< boost::mt19937, boost::normal_distribution<> > dist(*randNumGen, distribution);

  geometry_msgs::Pose poseWithError; 
  nav_msgs::Odometry odomWithError;
  poseWithError.position.x = uavPose.position.x + dist();
  poseWithError.position.y = uavPose.position.y + dist();
  poseWithError.position.z = uavPose.position.z + dist();
  poseWithError.orientation.x = uavPose.orientation.x;// + dist();
  poseWithError.orientation.y = uavPose.orientation.y;// + dist();
  poseWithError.orientation.z = uavPose.orientation.z;// + dist();
  poseWithError.orientation.w = uavPose.orientation.w;// + dist();

  odomWithError.header.stamp = ros::Time::now();
  odomWithError.pose.pose = poseWithError;
  
  for (int i=0; i<6; i++) {
    for (int j=0; j<6; j++) {
      if (i==j) {
        odomWithError.pose.covariance[6*i+j] = this->errorVar;
      } else {
        odomWithError.pose.covariance[6*i+j] = 0.0;
      }
    }
  }

  this->posePublisher.publish(poseWithError);
  this->odomPublisher.publish(odomWithError);

  delete(randNumGen); 
}
