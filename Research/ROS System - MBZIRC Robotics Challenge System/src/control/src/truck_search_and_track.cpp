#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <string>
#include <iostream>
#include <cmath>

static const std::string TOPIC_PERCEPTION_CAMERA_CENTER_OFFSET = "perception/camera_center_offset";
static const std::string TOPIC_PERCEPTION_POSITION_ESTIMATE_TRUCK = "truck_perception/truck_position";
static const std::string TOPIC_MAVROS_POSITION_LOCAL = "mavros/local_position/pose";
static const std::string TOPIC_MAVROS_VELOCITY = "mavros/local_position/velocity";
static const std::string TOPIC_UAV_CONTROLLER_TAKEOFF = "/hexacopter/uav_control/takeoff";
static const std::string TOPIC_UAV_CONTROLLER_LAND = "uav_control/land";
static const std::string TOPIC_UAV_CONTROLLER_WAYPOINT = "/hexacopter/uav_control/waypoint";
static const std::string TOPIC_UAV_CONTROLLER_VELOCITY = "uav_control/velocity";
static const std::string TOPIC_UAV_LOCALIZED_POSITION = "localizer/rtk_pose";
static const int UAV_START_HEIGHT = 11;
static const int UAV_POSITION_OFFSET_X = -42;
static const int UAV_POSITION_OFFSET_Y = 38;
int once=0;

class TruckSearchAndTrack
{
private:

  enum State
  {
    NONE,
    SEARCH_IDLE,
    SEARCH_WAYPOINT,
    TRACK
  };

  ros::Subscriber camera_center_offset_subscriber;
  ros::Subscriber perception_position_estimate_subscriber;
  ros::Subscriber uav_velocity_subscriber;
  ros::Subscriber uav_localized_position_subscriber;
  ros::Publisher takeoff_publisher;
  ros::Publisher land_publisher;
  ros::Publisher waypoint_publisher;
  ros::Publisher velocity_publisher;
  State state;
  geometry_msgs::Pose uav_position;
  geometry_msgs::Twist uav_velocity;
  geometry_msgs::Pose waypoint;
  geometry_msgs::Pose truck_position;

public:

  TruckSearchAndTrack(ros::NodeHandle &ros_node)
  {
    this->camera_center_offset_subscriber = ros_node.subscribe<geometry_msgs::Twist>(TOPIC_PERCEPTION_CAMERA_CENTER_OFFSET, 10, &TruckSearchAndTrack::camera_center_offset_callback, this);
    this->perception_position_estimate_subscriber = ros_node.subscribe<geometry_msgs::Twist>(TOPIC_PERCEPTION_POSITION_ESTIMATE_TRUCK, 10, &TruckSearchAndTrack::perception_position_estimate_callback, this);
    this->uav_velocity_subscriber = ros_node.subscribe<geometry_msgs::TwistStamped>(TOPIC_MAVROS_VELOCITY, 10, &TruckSearchAndTrack::uav_velocity_callback, this);
    this->uav_localized_position_subscriber = ros_node.subscribe<geometry_msgs::Pose>(TOPIC_UAV_LOCALIZED_POSITION, 10, &TruckSearchAndTrack::uav_position_callback, this);
    this->takeoff_publisher = ros_node.advertise<std_msgs::Empty>(TOPIC_UAV_CONTROLLER_TAKEOFF, 10);
    this->land_publisher = ros_node.advertise<std_msgs::Empty>(TOPIC_UAV_CONTROLLER_LAND, 10);
    this->waypoint_publisher = ros_node.advertise<geometry_msgs::Pose>(TOPIC_UAV_CONTROLLER_WAYPOINT, 10);
    this->velocity_publisher = ros_node.advertise<geometry_msgs::Twist>(TOPIC_UAV_CONTROLLER_VELOCITY, 10);
    this->state = State::NONE;
    ros::Duration(1).sleep();
    // TODO: This is temporary.
    this->truck_position.position.x = UAV_POSITION_OFFSET_X;
    this->truck_position.position.y = UAV_POSITION_OFFSET_Y;
    this->truck_position.position.z = UAV_START_HEIGHT;
  }

  void execute()
  {
    switch (this->state)
    {
    case State::NONE:
      this->takeoff();
      break;
    case State::SEARCH_IDLE:
	if(once==0){
      std::cout << "go_to_the_origin..." << std::endl;}
      this->waypoint = this->generate_waypoint();
      this->go_to_waypoint();
      //this->state = State::SEARCH_WAYPOINT;
      break;
    case State::TRACK:
      // this->waypoint.position.x = this->truck_position.position.x;
      // this->waypoint.position.y = this->truck_position.position.y;
      // this->waypoint.position.z -= 0.02;
      // this->go_to_waypoint();
      this->set_velocity();
      break;
    default:
      break;
    }
  }

private:

  void camera_center_offset_callback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    //this->state = State::TRACK;
    if (this->uav_position.position.z > 3.0)
    {
      this->uav_velocity.linear.y = this->camera_offset_to_speed(msg->linear.x);
      this->uav_velocity.linear.x = this->camera_offset_to_speed(msg->linear.y);
      this->uav_velocity.linear.z = -0.08;
    }
    else
    {
      // this->uav_velocity.linear.z = -0.3;
      this->land_publisher.publish(std_msgs::Empty());
    }
    // this->uav_velocity.linear.z = this->uav_position.position.z > UAV_START_HEIGHT ? -0.01 : 0.05;
  }

  double camera_offset_to_speed(double camera_offset)
  {
    const double MAX_SPEED = 0.5;
    const double MAX_OFFSET = 200;
    const double OVERTAKE_COEFFECIENT = 1.4;
    if (camera_offset > MAX_OFFSET)
    {
      return MAX_SPEED;
    }
    else if (camera_offset < -MAX_OFFSET)
    {
      return -MAX_SPEED;
    }
    return OVERTAKE_COEFFECIENT * MAX_SPEED * (camera_offset / MAX_OFFSET);
  }

  void perception_position_estimate_callback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    // this->state = State::TRACK;
    // if (!std::isnan(msg->linear.x) && !std::isnan(msg->linear.y))
    // {
    //   // this->truck_position.position.x = UAV_POSITION_OFFSET_X + msg->linear.x;
    //   // this->truck_position.position.y = UAV_POSITION_OFFSET_Y + msg->linear.y;
    //   this->uav_velocity.linear.x = this->cap_speed((msg->linear.x - this->uav_position.position.x) / 5.0);
    //   this->uav_velocity.linear.y = this->cap_speed((msg->linear.y - this->uav_position.position.y) / 5.0);
    //   // this->uav_velocity.linear.x = this->cap_speed((msg->linear.x - this->uav_position.position.x) / 5.0);
    //   // this->uav_velocity.linear.y = this->cap_speed((msg->linear.y - this->uav_position.position.y) / 5.0);
    //   // this->uav_velocity.linear.z = this->uav_position.position.z > UAV_START_HEIGHT ? -0.01 : 0.05;
    //   std::cout << "UAV: (" << this->uav_position.position.x << ", " << this->uav_position.position.y << "), Truck: (";
    //   std::cout << msg->linear.x << ", " << msg->linear.y << "), Velociy (" << this->uav_velocity.linear.x;
    //   std::cout << ", " << this->uav_velocity.linear.y << ")" << std::endl;
    // }
  }

  double cap_speed(double speed)
  {
    double MAX_SPEED = 0.5;
    if (speed > MAX_SPEED)
    {
      return MAX_SPEED;
    }
    else if (speed < (-1.0 * MAX_SPEED))
    {
      return -1.0 * MAX_SPEED;
    }
    return speed;
  }

  void uav_position_callback(const geometry_msgs::Pose::ConstPtr& msg)
  {
    this->uav_position = *msg;
  }

  void uav_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
  {
    // this->uav_velocity = msg->twist;
    // std::cout << "Velocity (" << this->uav_velocity.linear.x << ", " << this->uav_velocity.linear.y << ", " << this->uav_velocity.linear.z << ")" << std::endl;
  }

  void takeoff()
  {
    this->takeoff_publisher.publish(std_msgs::Empty());
    this->state = State::SEARCH_IDLE;
    ros::Duration(10).sleep();
  }

  void search()
  {
    std::cout << "Searching..." << std::endl;
  }

  void go_to_waypoint()
  {

	if(once==0){
    this->waypoint_publisher.publish(this->waypoint);
	once=1;}

  }

  void set_velocity()
  {
    this->velocity_publisher.publish(this->uav_velocity);
  }

  geometry_msgs::Pose generate_waypoint()
  {
    geometry_msgs::Pose waypoint;
    //waypoint.position.x = UAV_POSITION_OFFSET_X;
    //waypoint.position.y = UAV_POSITION_OFFSET_Y;
    //waypoint.position.z = UAV_START_HEIGHT;

    waypoint.position.x = 0;
    waypoint.position.y = 0;
    waypoint.position.z = 15;
    return waypoint;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "truck_search_and_track");

  ros::NodeHandle ros_node;
  int rate = 10;
  ros_node.param(ros::this_node::getName() + "/rate", rate, rate);
  ros::Rate ros_rate(rate);

  TruckSearchAndTrack truck_search_and_track(ros_node);

  while (ros_node.ok())
  {
    truck_search_and_track.execute();
    ros::spinOnce();
    ros_rate.sleep();
  }

  return 0;
}
