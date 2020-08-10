/*
This code takes in the Pixhawk pose through the mavros/global_position/local topic and publishes the transfrom from the arming location orgin.
*/


#include <ros/ros.h>
// #include <tf2_ros,h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>
//#include <piksi_rtk_msgs/BaselineNed.h>
#include <nav_msgs/Odometry.h>

double earth_R = 6378100;
double pi = 3.141592;
double nsl_x, nsl_y, nsl_z, nsl_rx, nsl_ry, nsl_rz, nsl_rw;
double usl_x, usl_y, usl_z, usl_rx, usl_ry, usl_rz, usl_rw;
double threat1_x, threat1_y, threat1_z, threat2_x, threat2_y, threat2_z;
double gimbal_x, gimbal_y, gimbal_z;


double ref_lat = 37.196917*pi/180;
double ref_long = -80.578573*pi/180; 
double gimbal_location_nsl[3] = {0.13, 0.0, -0.20}; // x -> front y->left z-> up (RVIZ)

//double gimbal_location_usl[3] = {-0.2, 0.082, -0.397};

double omnicam_location_nsl[3] = {0.0, -0.06, 0.13}; // x -> front y->left z-> up (RVIZ)

double omnicam_location_usl[3] = {0.034, -1.8, -0.1445}; // x -> front y->left z-> up (RVIZ)

tf::Quaternion q, q_omni_nsl, q_usl;
tfScalar yaw_nsl, pitch_nsl, roll_nsl;
tfScalar yaw_usl, pitch_usl, roll_usl;


using namespace std;

void global_CB_nsl(const sensor_msgs::NavSatFix msg){

  double nsl_lat = msg.latitude * pi/180;
  double nsl_long = msg.longitude * pi/180;

  double dlat = nsl_lat - ref_lat;
  double dlong = nsl_long - ref_long;

  double x = cos(nsl_lat)*sin(dlong);
  double y = cos(ref_lat)*sin(nsl_lat)-sin(ref_lat)*cos(nsl_lat)*cos(dlong);

  double d = acos(cos(pi/2-ref_lat)*cos(pi/2-nsl_lat)+sin(pi/2-ref_lat)*sin(pi/2-nsl_lat)*cos(-dlong))*earth_R;
  double th = -atan2(-y,x);

  nsl_x = d*cos(th);
  nsl_y = d*sin(th);

  cout<<"nsl "<<nsl_x<<" "<<nsl_y<<" "<<th*180/pi<<endl;

}

void global_CB_usl(const sensor_msgs::NavSatFix msg){

  double usl_lat = msg.latitude * pi/180;
  double usl_long = msg.longitude * pi/180;

  double dlat = usl_lat - ref_lat;
  double dlong = usl_long - ref_long;

  double x = cos(usl_lat)*sin(dlong);
  double y = cos(ref_lat)*sin(usl_lat)-sin(ref_lat)*cos(usl_lat)*cos(dlong);

  double d = acos(cos(pi/2-ref_lat)*cos(pi/2-usl_lat)+sin(pi/2-ref_lat)*sin(pi/2-usl_lat)*cos(-dlong))*earth_R;
  double th = -atan2(-y,x);

  usl_x = d*cos(th);
  usl_y = d*sin(th);

  cout<<"usl "<<usl_x<<" "<<usl_y<<" "<<th*180/pi<<endl;

}

void threat1_CB(const sensor_msgs::NavSatFix msg){

  double threat_lat = msg.latitude * pi/180;
  double threat_long = msg.longitude * pi/180;

  double dlat = threat_lat - ref_lat;
  double dlong = threat_long - ref_long;

  double x = cos(threat_lat)*sin(dlong);
  double y = cos(ref_lat)*sin(threat_lat)-sin(ref_lat)*cos(threat_lat)*cos(dlong);

  double d = acos(cos(pi/2-ref_lat)*cos(pi/2-threat_lat)+sin(pi/2-ref_lat)*sin(pi/2-threat_lat)*cos(-dlong))*earth_R;
  double th = -atan2(-y,x);

  threat1_x = d*cos(th);
  threat1_y = d*sin(th);
  threat1_z = msg.altitude - 523;

}

void threat2_CB(const sensor_msgs::NavSatFix msg){

  double threat_lat = msg.latitude * pi/180;
  double threat_long = msg.longitude * pi/180;

  double dlat = threat_lat - ref_lat;
  double dlong = threat_long - ref_long;

  double x = cos(threat_lat)*sin(dlong);
  double y = cos(ref_lat)*sin(threat_lat)-sin(ref_lat)*cos(threat_lat)*cos(dlong);

  double d = acos(cos(pi/2-ref_lat)*cos(pi/2-threat_lat)+sin(pi/2-ref_lat)*sin(pi/2-threat_lat)*cos(-dlong))*earth_R;
  double th = -atan2(-y,x);

  threat2_x = d*cos(th);
  threat2_y = d*sin(th);
  threat2_z = msg.altitude - 523;

}
void local_CB_nsl(const nav_msgs::Odometry msg){  

  nsl_z = msg.pose.pose.position.z;

  nsl_rx = msg.pose.pose.orientation.x;
  nsl_ry = msg.pose.pose.orientation.y;
  nsl_rz = msg.pose.pose.orientation.z;
  nsl_rw = msg.pose.pose.orientation.w;

  tf::Quaternion q_nsl;

  q_nsl.setX(msg.pose.pose.orientation.x);
  q_nsl.setY(msg.pose.pose.orientation.y);
  q_nsl.setZ(msg.pose.pose.orientation.z);
  q_nsl.setW(msg.pose.pose.orientation.w);

  tf::Matrix3x3 mat(q_nsl);
  mat.getEulerYPR(yaw_nsl, roll_nsl, pitch_nsl);

}

void local_CB_usl(const nav_msgs::Odometry msg){  

  usl_z = msg.pose.pose.position.z;

  usl_rx = msg.pose.pose.orientation.x;
  usl_ry = msg.pose.pose.orientation.y;
  usl_rz = msg.pose.pose.orientation.z;
  usl_rw = msg.pose.pose.orientation.w;

  q_usl.setX(msg.pose.pose.orientation.x);
  q_usl.setY(msg.pose.pose.orientation.y);
  q_usl.setZ(msg.pose.pose.orientation.z);
  q_usl.setW(msg.pose.pose.orientation.w);

  tf::Matrix3x3 mat(q_usl);
  mat.getEulerYPR(yaw_usl, roll_usl, pitch_usl); 

  q_usl.setRPY(roll_usl, pitch_usl, yaw_usl - 0*pi/180); // Camera yaw 0 is looking at left
}

void gimbal_CB_nsl(const geometry_msgs::PoseStamped& msg){
  
  gimbal_x = gimbal_location_nsl[0];
  gimbal_y = gimbal_location_nsl[1];
  gimbal_z = gimbal_location_nsl[2];

  q.setX(msg.pose.orientation.x);
  q.setY(msg.pose.orientation.y);
  q.setZ(msg.pose.orientation.z);
  q.setW(msg.pose.orientation.w);

  tfScalar yaw, pitch, roll;
  tf::Matrix3x3 mat(q);
  mat.getEulerYPR(yaw, roll, pitch);

  // Gimbal pitch and roll angles are in inertial frame
  // Gimbal yaw angle is in encorder frame
  // Therefore, gimbal pitch and roll angles are subtracted by aircraft orientation
  // To consider aircraft orientation.

  //For dataset 1 and 2
  q.setRPY(roll - roll_nsl, pitch + pitch_nsl, -yaw + pi/2 - 20*pi/180); // Camera yaw 0 is looking at left
  
  // For dataset 3
  //q.setRPY(roll - roll_nsl, pitch + pitch_nsl -25*pi/180, -yaw - pi/2 - 20*pi/180); // Camera yaw 0 is looking at left


}

int main(int argc, char** argv){
  ros::init(argc, argv, "observer_aircraft_tf2_broadcaster");

  ros::NodeHandle private_node("~");
  ros::NodeHandle node;

  ros::Subscriber global_sub_nsl = node.subscribe("nslhex/mavros/global_position/global", 10, &global_CB_nsl);
  ros::Subscriber global_sub_usl = node.subscribe("uslhex/mavros/global_position/global", 10, &global_CB_usl);
  ros::Subscriber local_sub_nsl = node.subscribe("nslhex/mavros/global_position/local", 10, &local_CB_nsl);
  ros::Subscriber local_sub_usl = node.subscribe("uslhex/mavros/global_position/local", 10, &local_CB_usl);
  ros::Subscriber threat_sub1_1 = node.subscribe("flight1_threat1", 10, &threat1_CB);
  ros::Subscriber threat_sub2_1 = node.subscribe("flight2_threat1", 10, &threat1_CB);
  ros::Subscriber threat_sub2_2 = node.subscribe("flight2_threat2", 10, &threat2_CB);
  ros::Subscriber threat_sub3_1 = node.subscribe("flight3_threat1", 10, &threat1_CB);
  ros::Subscriber threat_sub3_2 = node.subscribe("flight3_threat2", 10, &threat2_CB);

  ros::Subscriber sub = node.subscribe("nslhex/gimbal_imu_angles", 10, &gimbal_CB_nsl);

  
  while(ros::ok())
	{
		ros::spinOnce();
    
    static tf2_ros::TransformBroadcaster br; 

    geometry_msgs::TransformStamped tf_nslhex;
    tf_nslhex.header.stamp = ros::Time::now();
    tf_nslhex.header.frame_id = "reference";
    tf_nslhex.child_frame_id = "nslhex";
    tf_nslhex.transform.translation.x = nsl_x;
    tf_nslhex.transform.translation.y = nsl_y;
    tf_nslhex.transform.translation.z = nsl_z;

    tf_nslhex.transform.rotation.x = nsl_rx;
    tf_nslhex.transform.rotation.y = nsl_ry;
    tf_nslhex.transform.rotation.z = nsl_rz;
    tf_nslhex.transform.rotation.w = nsl_rw;

    br.sendTransform(tf_nslhex);

    geometry_msgs::TransformStamped tf_uslhex;
    tf_uslhex.header.stamp = ros::Time::now();
    tf_uslhex.header.frame_id = "reference";
    tf_uslhex.child_frame_id = "uslhex";
    tf_uslhex.transform.translation.x = usl_x;
    tf_uslhex.transform.translation.y = usl_y;
    tf_uslhex.transform.translation.z = usl_z;

    tf_uslhex.transform.rotation.x = q_usl.getX();
    tf_uslhex.transform.rotation.y = q_usl.getY();
    tf_uslhex.transform.rotation.z = q_usl.getZ();
    tf_uslhex.transform.rotation.w = q_usl.getW();

    br.sendTransform(tf_uslhex);

    geometry_msgs::TransformStamped tf_gimbal_nsl;
    tf_gimbal_nsl.header.stamp = ros::Time::now();
    tf_gimbal_nsl.header.frame_id = "nslhex";
    tf_gimbal_nsl.child_frame_id = "gimbal_nsl";
    tf_gimbal_nsl.transform.translation.x = gimbal_x;
    tf_gimbal_nsl.transform.translation.y = gimbal_y;
    tf_gimbal_nsl.transform.translation.z = gimbal_z;

    tf_gimbal_nsl.transform.rotation.x = q.getX();
    tf_gimbal_nsl.transform.rotation.y = q.getY();
    tf_gimbal_nsl.transform.rotation.z = q.getZ();
    tf_gimbal_nsl.transform.rotation.w = q.getW();

    br.sendTransform(tf_gimbal_nsl);

    geometry_msgs::TransformStamped tf_omnicam_usl;
    tf_omnicam_usl.header.stamp = ros::Time::now();
    tf_omnicam_usl.header.frame_id = "uslhex";
    tf_omnicam_usl.child_frame_id = "omnicam_usl";
    tf_omnicam_usl.transform.translation.x = omnicam_location_usl[0];
    tf_omnicam_usl.transform.translation.y = omnicam_location_usl[1];
    tf_omnicam_usl.transform.translation.z = omnicam_location_usl[2];

    tf_omnicam_usl.transform.rotation.x = 0;
    tf_omnicam_usl.transform.rotation.y = 0;
    tf_omnicam_usl.transform.rotation.z = 0;
    tf_omnicam_usl.transform.rotation.w = 1;

    br.sendTransform(tf_omnicam_usl);


    geometry_msgs::TransformStamped tf_omnicam_nsl;
    tf_omnicam_nsl.header.stamp = ros::Time::now();
    tf_omnicam_nsl.header.frame_id = "nslhex";
    tf_omnicam_nsl.child_frame_id = "omnicam_nsl";
    tf_omnicam_nsl.transform.translation.x = omnicam_location_nsl[0];
    tf_omnicam_nsl.transform.translation.y = omnicam_location_nsl[1];
    tf_omnicam_nsl.transform.translation.z = omnicam_location_nsl[2];

    q_omni_nsl.setRPY(0, 0, 0*pi/180); // Camera yaw 0 is looking at left
    tf_omnicam_nsl.transform.rotation.x = q_omni_nsl.getX();
    tf_omnicam_nsl.transform.rotation.y = q_omni_nsl.getY();
    tf_omnicam_nsl.transform.rotation.z = q_omni_nsl.getZ();
    tf_omnicam_nsl.transform.rotation.w = q_omni_nsl.getW();

    br.sendTransform(tf_omnicam_nsl);


    geometry_msgs::TransformStamped tf_threat1;
    tf_threat1.header.stamp = ros::Time::now();
    tf_threat1.header.frame_id = "reference";
    tf_threat1.child_frame_id = "threat1";
    tf_threat1.transform.translation.x = threat1_x;
    tf_threat1.transform.translation.y = threat1_y;
    tf_threat1.transform.translation.z = threat1_z;

    tf_threat1.transform.rotation.x = 0;
    tf_threat1.transform.rotation.y = 0;
    tf_threat1.transform.rotation.z = 0;
    tf_threat1.transform.rotation.w = 1;

    br.sendTransform(tf_threat1);


    geometry_msgs::TransformStamped tf_threat2;
    tf_threat2.header.stamp = ros::Time::now();
    tf_threat2.header.frame_id = "reference";
    tf_threat2.child_frame_id = "threat2";
    tf_threat2.transform.translation.x = threat2_x;
    tf_threat2.transform.translation.y = threat2_y;
    tf_threat2.transform.translation.z = threat2_z;

    tf_threat2.transform.rotation.x = 0;
    tf_threat2.transform.rotation.y = 0;
    tf_threat2.transform.rotation.z = 0;
    tf_threat2.transform.rotation.w = 1;

    br.sendTransform(tf_threat2);
	}
  return 0;
};
