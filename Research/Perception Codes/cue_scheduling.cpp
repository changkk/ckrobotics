#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <unordered_set> 
#include <algorithm>
#include <functional>
#include <iostream>
#include <utility>      // std::pair
#include<bits/stdc++.h> 
#include <queue> 

using namespace std;
using namespace tf2;

tf::Matrix3x3 rotation;
int pseudo_range = 2;
double observed_thres = 0.01;

ros::Publisher ang_pub;
ros::Publisher gimbal_cue;
ros::Publisher target_list_pub;
ros::Publisher target_vector_pub;
ros::Publisher target_acquired_cue;

vector<int> observed_list;
queue<int> cur_target;
double actual_target[3];
vector<pair<bool,vector<double>>> actual_target_list;
vector<pair<double,double>> azi_ele_list;
geometry_msgs::PolygonStamped target_list_msgs;
int wait_time = 10;
int cost_param = 20;
bool central_detection = false;
bool central_detection_prev = false;

int wait;
bool first_in = true;
double pitch, yaw;

void poseCallback(const geometry_msgs::PoseStamped& msg){
  
    double q_x = msg.pose.orientation.x;
    double q_y = msg.pose.orientation.y;
    double q_z = msg.pose.orientation.z;
    double q_w = msg.pose.orientation.w;
    double roll;

    // pitch (x-axis rotation)
    double sinr_cosp = +2.0 * (q_w * q_x + q_y * q_z);
    double cosr_cosp = +1.0 - 2.0 * (q_x * q_x + q_y * q_y);
    pitch = atan2(sinr_cosp, cosr_cosp);

    // roll (y-axis rotation)
    double sinp = +2.0 * (q_w * q_y - q_z * q_x);
    if (fabs(sinp) >= 1)
        double roll = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        double roll = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q_w * q_z + q_x * q_y);
    double cosy_cosp = +1.0 - 2.0 * (q_y * q_y + q_z * q_z);  
//    double yaw = atan2(siny_cosp, cosy_cosp) - 0.2;
    yaw = atan2(siny_cosp, cosy_cosp);

    // // Abbreviations for the various angular functions
    // double cy = cos(yaw * 0.5);
    // double sy = sin(yaw * 0.5);
    // double cp = cos(roll * 0.5);
    // double sp = sin(roll * 0.5);
    // double cr = cos(pitch * 0.5);
    // double sr = sin(pitch * 0.5);

    // q_w = cy * cp * cr + sy * sp * sr;
    // q_x = cy * cp * sr - sy * sp * cr;
    // q_y = sy * cp * sr + cy * sp * cr;
    // q_z = sy * cp * cr - cy * sp * sr;

    // REMEMBER //
    // A rotation matirx and translation vector is about "A VECTOR" not a frame.
    // Therefore, in order to know the coordinates of a point in 'another frame',
    // You need to translate the point in the oppsite direction first,
    // and rotate the vector in the opposite direction.
    // e.g.) If the gimbal frame is rotated about 45 deg (clockwise of course)
    //       The point vector should be rotated by -045 deg to know the coordintate pof the point
    //       in the gimbal frame.

    tf::Matrix3x3 rot_x(1,0,0, 0,cos(-pitch),-sin(-pitch),   0,sin(-pitch),cos(-pitch) );
    tf::Matrix3x3 rot_y(cos(-roll),0,sin(-roll),  0,1,0,   -sin(-roll),0,cos(-roll) );
    tf::Matrix3x3 rot_z(cos(-yaw),-sin(-yaw),0,   sin(-yaw),cos(-yaw),0,   0,0,1 );
    rotation = rot_x*rot_y*rot_z;

    //cout<<roll*180/3.141592<<" "<<pitch*180/3.141592<<" "<<yaw*180/3.141592<<endl;
}

vector<int> TSP()
{
    double table[azi_ele_list.size()][azi_ele_list.size()];
    // Make orthodrominc distance table
    for(int i = 0; i<azi_ele_list.size(); i++)
    {
        double azi01 = azi_ele_list[i].first;
        double ele01 = azi_ele_list[i].second;
        for(int j = i+1; i<azi_ele_list.size(); i++)
        {
            double azi02 = azi_ele_list[j].first;
            double ele02 = azi_ele_list[j].second;
            double d_azi = abs(azi01 - azi02);
            double d_ele = abs(ele01 - ele02);
            double o_dist = 2*asin(sqrt(sin(d_ele/2)*sin(d_ele/2)+cos(d_ele)*cos(0)*sin(d_azi/2)*sin(d_azi/2)));
            table[i][j] = o_dist;
            table[j][i] = o_dist;
        }
    }

    double distanceFromGimbal[azi_ele_list.size()]; 
    vector<int> order;
    vector<double> risk_values;
    // Make orthodromic distance table from gimbal
    for(int i = 0; i<azi_ele_list.size(); i++)
    {
        double azi02 = azi_ele_list[i].first;
        double ele02 = azi_ele_list[i].second;
        double d_azi = abs(yaw - azi02);
        double d_ele = abs(pitch - ele02);
        double o_dist = 2*asin(sqrt(sin(d_ele/2)*sin(d_ele/2)+cos(d_ele)*cos(0)*sin(d_azi/2)*sin(d_azi/2)));
        distanceFromGimbal[i] = o_dist;
        // For the permutation
        order.push_back(i);
        // Compute the risk of each target
        double risk_value = 1/(sqrt(actual_target_list[i].second[0]*actual_target_list[i].second[0] + actual_target_list[i].second[1]*actual_target_list[i].second[1] + actual_target_list[i].second[2]*actual_target_list[i].second[2]));
        risk_values.push_back(risk_value);
    }

    double min_cost = INT_MAX;
    vector<int> optimal_order;
    // Try all permutatons.
    do
    {
        double cost = 0;

        cost += distanceFromGimbal[order.front()];

        for (int i = 0; i < order.size()-1; i++) 
        {
            cost += table[order[i]][order[i+1]];
            cost += risk_values[order[i]] * (i+1) * cost_param;
        }

        cost += risk_values[order.back()] * order.size() * cost_param;

        if(min_cost > cost)
        {
            min_cost = cost;
            optimal_order.clear();
            optimal_order = order;
        }
    }   
    while (next_permutation(order.begin(), order.end()));

    cout<<"TSP + Cost function based prioritzation:"<<endl;
    for(auto x:optimal_order)
        cout<<x<<" "<<1/risk_values[x]<<endl;

    return optimal_order;
}

void fisheye_point(const geometry_msgs::PolygonStamped& msg)	
{
    vector<pair<double,int>> ortho_dist_list;
    azi_ele_list.clear();

    if(!msg.polygon.points[0].x == 0)
    {
        // The x component of the first message is the size of the detections.
        // Make a target list (azi_ele list and actual target coordinates list)
        for(int i=1; i<msg.polygon.points[0].x+1; i++)
        {
            geometry_msgs::Vector3Stamped fisheye_vector;

            double fisheye_sensor_width=3.2; //3.2
            double fisheye_sensor_height=3.2; //3.2

            // node.getParam("/range_estimation_correction/fisheye_front2",fisheye_front);

            fisheye_vector.header.stamp = ros::Time(0); 

            //////////////////////////////////////////////////////////////////////////////////////////////
            // If the vector is not converted from the detection in terms of the camera direction,
            // The vector direction should be changed.
            // bool fisheye_front;         
            // if(fisheye_front)
            // {
            //     fisheye_vector.vector.x = msg.polygon.points[i].x/1440*fisheye_sensor_width*0.001; //ENU Frame ( we are using back side camera of omnicam)
            //     fisheye_vector.vector.y = -msg.polygon.points[i].z/1440*fisheye_sensor_width*0.001; //ENU Frame ( we are using back side camera of omnicam)
            //     fisheye_vector.vector.z = msg.polygon.points[i].y/1440*fisheye_sensor_width*0.001; //ENU Frame ( we are using back side camera of omnicam)
            //     cout<<"front"<<endl;
            // }
            // else
            // {
            //     fisheye_vector.vector.x = -msg.polygon.points[i].x/1440*fisheye_sensor_width*0.001; //ENU Frame ( we are using back side camera of omnicam)
            //     fisheye_vector.vector.y = msg.polygon.points[i].z/1440*fisheye_sensor_width*0.001; //ENU Frame ( we are using back side camera of omnicam)
            //     fisheye_vector.vector.z = msg.polygon.points[i].y/1440*fisheye_sensor_width*0.001; //ENU Frame ( we are using back side camera of omnicam)
            //     cout<<"back"<<endl;
            // }
            //////////////////////////////////////////////////////////////////////////////////////////////


            fisheye_vector.vector.x = msg.polygon.points[i].x; 
            fisheye_vector.vector.y = msg.polygon.points[i].z; 
            fisheye_vector.vector.z = msg.polygon.points[i].y; 

            tf::Vector3 fisheye_vec_tf(fisheye_vector.vector.x, fisheye_vector.vector.y, fisheye_vector.vector.z);
            double fisheye_vector_length = sqrt(fisheye_vector.vector.x*fisheye_vector.vector.x + fisheye_vector.vector.y*fisheye_vector.vector.y + fisheye_vector.vector.z*fisheye_vector.vector.z);
            fisheye_vec_tf = fisheye_vec_tf/fisheye_vector_length*pseudo_range;
            tf::Vector3 translation_vector(-2,0.048,0.2525);
            tf::Vector3 translation_vector_omni(-1.8, -0.034, 0.1445);

            if(actual_target_list.empty())
            {
                geometry_msgs::Point32 pt_tmp;
                pt_tmp.x = msg.polygon.points[0].x;
                target_list_msgs.polygon.points.push_back(pt_tmp); // The first element is the size of the msg.
            }
            else
            {
                target_list_msgs.polygon.points[0].x = msg.polygon.points[0].x;
            }
            

            if(actual_target_list.size() < msg.polygon.points[0].x && i > actual_target_list.size())
            {
                tf::Vector3 target_in_global = fisheye_vec_tf - translation_vector_omni;
                vector<double> target_tmp;
                target_tmp.push_back(target_in_global.x());
                target_tmp.push_back(target_in_global.y());
                target_tmp.push_back(target_in_global.z());
                actual_target_list.push_back(make_pair(true,target_tmp));

                geometry_msgs::Point32 pt_tmp;
                pt_tmp.x = target_in_global.x();
                pt_tmp.y = target_in_global.y();
                pt_tmp.z = target_in_global.z();
                target_list_msgs.polygon.points.push_back(pt_tmp);
            }

            // REMEMBER //
            // A rotation matirx and translation vector is about "A VECTOR" not a frame.
            // Therefore, in order to know the coordinates of a point in 'another frame',
            // You need to translate the point in the oppsite direction first,
            // and rotate the vector in the opposite direction.
            // e.g.) If the gimbal frame is rotated about 45 deg (clockwise of course)
            //       The point vector should be rotated by -045 deg to know the coordintate pof the point
            //       in the gimbal frame.

            tf::Vector3 fisheye_in_gimbal = (fisheye_vec_tf - translation_vector);

            // If new target is detected, add it to the list.
            double azim0 = -atan2(fisheye_vec_tf.x(), fisheye_vec_tf.y());
            double elev0 = atan(fisheye_vec_tf.z()/sqrt(fisheye_vec_tf.x()*fisheye_vec_tf.x()+fisheye_vec_tf.y()*fisheye_vec_tf.y()));
            double azim = -atan2(fisheye_in_gimbal.x(), fisheye_in_gimbal.y());
            // Add the omnicam offset 14 deg since the omnicam is looking at a bit of upward.
            double elev = atan(fisheye_in_gimbal.z()/sqrt(fisheye_in_gimbal.x()*fisheye_in_gimbal.x()+fisheye_in_gimbal.y()*fisheye_in_gimbal.y()))  + 12*3.141592/180;

            // This is for sorting.
            fisheye_in_gimbal = rotation*(fisheye_vec_tf - translation_vector);
            double azim_gimbal = -atan2(fisheye_in_gimbal.x(), fisheye_in_gimbal.y());
            // Add the omnicam offset 14 deg since the omnicam is looking at a bit of upward.
            double elev_gimbal = atan(fisheye_in_gimbal.z()/sqrt(fisheye_in_gimbal.x()*fisheye_in_gimbal.x()+fisheye_in_gimbal.y()*fisheye_in_gimbal.y()))  + 12*3.141592/180;
            double ortho_dist = 2*asin(sqrt(sin(elev_gimbal/2)*sin(elev_gimbal/2)+cos(elev_gimbal)*cos(0)*sin(azim_gimbal/2)*sin(azim_gimbal/2)));

//            cout<<"fisheye vec: "<<fisheye_vec_tf.x()<<" "<<fisheye_vec_tf.y()<<" "<<fisheye_vec_tf.z()<<endl;
//           cout<<"gimbal vec: "<<fisheye_in_gimbal.x()<<" "<<fisheye_in_gimbal.y()<<" "<<fisheye_in_gimbal.z()<<endl;

//            cout<<"omni_azim: "<<azim0*180/3.141592<<" omni_elev: "<<elev0*180/3.141592<<endl;
//            cout<<"gige_azim: "<<azim*180/3.141592<<" gige_elev: "<<elev*180/3.141592<<endl;

//            cout<<" "<<endl;

            // Key: ortho_dist, Value: index
            ortho_dist_list.push_back(make_pair(ortho_dist,i-1));
            // Azimuth, elevation list
            azi_ele_list.push_back(make_pair(azim,elev));        
        }

        // Base case (initial prioritization)
        if(cur_target.empty())
        {
            if(azi_ele_list.size()>1)
            {
                vector<int> tmp = TSP();
                for (auto x:tmp)
                    cur_target.push(x);
            }
            else
                cur_target.push(0);
        }


        cout<<"Current target index: "<<cur_target.front()<<"/"<<cur_target.size()<<endl; // From 0

        // For range estimation
        geometry_msgs::Twist vector_msg;
        vector_msg.linear.x=msg.polygon.points[cur_target.front()+1].x;
        vector_msg.linear.y=msg.polygon.points[cur_target.front()+1].z;
        vector_msg.linear.z=msg.polygon.points[cur_target.front()+1].y;
        target_vector_pub.publish(vector_msg);

        // For gimbal control
        geometry_msgs::Point32 pt_tmp;
        geometry_msgs::PolygonStamped msg;
        pt_tmp.x = azi_ele_list[cur_target.front()].first; // azimuth
        pt_tmp.y = azi_ele_list[cur_target.front()].second; // elevation
        msg.polygon.points.push_back(pt_tmp);
        gimbal_cue.publish(msg);

        // When the gimbal sees the current target.
        if(ortho_dist_list[cur_target.front()].first < observed_thres)
        {
            cout<<"in"<<endl;

            // Is target cueing is done for the current target?
            std_msgs::Bool target_acquired_msgs;
            target_acquired_msgs.data = true;
            target_acquired_cue.publish(target_acquired_msgs);
        }
        else
        {
            std_msgs::Bool target_acquired_msgs;
            target_acquired_msgs.data = false;
            target_acquired_cue.publish(target_acquired_msgs);
        }    

        // If central detection is on, update the actual target range from the range estimation.
        if(central_detection)
        {
            // Update the actual target position to the actual target list
            if(actual_target[0] != 0)
            {
                actual_target_list[cur_target.front()].second[0] = actual_target[0];
                actual_target_list[cur_target.front()].second[1] = actual_target[1];
                actual_target_list[cur_target.front()].second[2] = actual_target[2];

                target_list_msgs.polygon.points[cur_target.front()+1].x = actual_target[0];
                target_list_msgs.polygon.points[cur_target.front()+1].y = actual_target[1];
                target_list_msgs.polygon.points[cur_target.front()+1].z = actual_target[2];

                cout<<"Target updated"<<endl;
                cout<<actual_target[0]<<" "<<actual_target[1]<<" "<<actual_target[2]<<endl;
            }
        }
        

        // When the waiting for the central vision camera is done, go on to the next target.
        if(central_detection_prev == true && central_detection == false)
        {
            int cur_pos = cur_target.front();
            cur_target.pop();
            cout<<"Next!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1!!"<<endl;

            // When the gimbal observed all the targets, restart to prioritze (reset).
            // This is where we need to use TSP + cost function
            if(cur_target.empty())
            {
                if(azi_ele_list.size()>1)
                {
                    vector<int> tmp = TSP();
                    for (auto x:tmp)
                        cur_target.push(x);
                }
                else
                    cur_target.push(0);
                
                // We don't want to keep observing the same target that we just saw. 
                if(cur_pos == cur_target.front())
                {
                    cur_target.pop();
                }
            }

        }

        target_list_pub.publish(target_list_msgs);
        central_detection_prev = central_detection;
    }


}

void actualTargetCB(const geometry_msgs::Twist& msg)
{
    actual_target[0]= msg.linear.x;
    actual_target[1]= msg.linear.y;
    actual_target[2]= msg.linear.z;
}

void central_detection_CB(const std_msgs::Bool& msg)
{
    central_detection = msg.data;
}


int main(int argc, char** argv){

	ros::init(argc,argv,"cue_scheduling");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("gimbal_imu_angles", 10, &poseCallback);
    ros::Subscriber fisheye_sub = node.subscribe("/gimbal_cue_periph", 1000, &fisheye_point);
    ros::Subscriber target_sub = node.subscribe("/target_epipolar", 1, &actualTargetCB);
    ros::Subscriber central_detection_sub = node.subscribe("/central_detection", 10, &central_detection_CB);

    ang_pub = node.advertise<geometry_msgs::PoseStamped>("/gimbal_target_orientation", 1);
    gimbal_cue = node.advertise<geometry_msgs::PolygonStamped>("/target_bearing_periph",100);
    target_vector_pub = node.advertise<geometry_msgs::Twist>("/image_frame_point_fisheye", 10);
    target_acquired_cue = node.advertise<std_msgs::Bool>("/target_acquired", 1);    
    target_list_pub = node.advertise<geometry_msgs::PolygonStamped>("/target_list_tf", 1);    

	ros::spin();

	return 0;
}

