#include <string.h>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Bool.h"
#include <sstream>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Joy.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <opencv2/opencv.hpp>
#include "opencv2/core/utility.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <mavros_msgs/RCIn.h>


using namespace std;
using namespace tf2;
using namespace cv;

cv_bridge::CvImagePtr cv_ptr;
// Mat frame;

double pitch_ce = 0, yaw_ce = 0, omni_angles[3] = {0, 0, 0}, gim_angles[3] = {0, 0, 0}, elev = 0, azim = 0;
double elev_old = 0, azim_old = 0, t = 0, t_old = 0, dt = 0, max_ang_spd = 1, pit_cmd = 0, yaw_cmd = 0;
double targ_bearing[2] = {0, 0}, delta_x = 0, delta_y = 0, area_ratio = 0;

bool central_detection = false, peripheral_detection = false, manual_control = true;
bool target_acquired_by_central = false;
int counter = 0, zoom = 0;
bool first_in = true;
int wait;

geometry_msgs::PoseStamped gm_angle_msg;
geometry_msgs::TwistStamped gm_speed_msg;
geometry_msgs::TwistStamped gm_man_speed_msg;

//Publishers and Subscribers---------------------------
ros::Publisher pitch_sp_pub;
ros::Publisher pitch_st_pub;
ros::Publisher pitch_en_pub;

ros::Publisher yaw_sp_pub;
ros::Publisher yaw_st_pub;
ros::Publisher yaw_en_pub;
ros::Publisher central_detection_pub;


ros::Publisher manual_zoom;
// ----------------------------------------------------
// Image Retrieval Class
class ImageConverter
{
  image_transport::Subscriber image_sub_;
 
public:

  ros::NodeHandle n;
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;
  
    
  ImageConverter()
      : it_(n)
  {
    // Subscrive to input video feed and publish output video feed
    image_transport::TransportHints hints("compressed", ros::TransportHints());
    // image_sub_ = it_.subscribe("/img_out", 1, &ImageConverter::imageCb, this, hints);
    // image_pub_ = it_.advertise("/peripheral_detection_image", 1);
  }

  ~ImageConverter()
  {}

  void imageCb(const sensor_msgs::ImageConstPtr &msg)
  {
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    //   cout << "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFO0" << endl;
    }

    catch (cv_bridge::Exception &e)

    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // frame = cv_ptr->image;
  }
};

//Yaw Control Effort Callback
void yaw_ce_sub_cb(const std_msgs::Float64 msg)
{
    yaw_ce = msg.data;
}

//Pitch Control Effort Callback
void pitch_ce_sub_cb(const std_msgs::Float64 msg)
{
    pitch_ce = msg.data;
}

//Callback for central camera detection
void central_cue_cb(const geometry_msgs::PolygonStamped msg)
{
    // msg.polygon.points.x[0];
    // msg.polygon.points.data;
    geometry_msgs::Point32 box_pos, img_size, box_size;

    //If something shows up here, set a flag high
    peripheral_detection = true;

    //Convert pixel value to elevation and azimuth angles using current piixhawk orientation from IMU callback, use TF for this and check in rviz
    //FOV -> 195 per camera
    double fov = 195;

    box_pos = msg.polygon.points[0];
    img_size = msg.polygon.points[1];

    azim_old = azim;
    elev_old = elev;

    t_old = t;
    t = ros::Time::now().toSec();  
    dt = t - t_old;

    azim = (-fov/img_size.x)*box_pos.x + fov/2;
    elev = (-fov/img_size.y)*box_pos.y + fov/2;

    //Gimbal speed limit in degrees per second to prevent agressive angle chnages
    int gimbal_spd_limit = 20; 

    if (elev > 80)
    {
        elev = 70;
    }
    else if (elev < -30)
    {
        elev = -30;
    }

    cout << "Elev = " << elev << "  Azim = " << azim << endl;

    //Publish posestamped message to gimbal angle topic
    Quaternion q(azim*M_PI/180, elev*M_PI/180, 0);

    gm_angle_msg.header.stamp = ros::Time::now();
    gm_angle_msg.pose.orientation.x = q.getX();
    gm_angle_msg.pose.orientation.y = q.getY();
    gm_angle_msg.pose.orientation.z = q.getZ();
    gm_angle_msg.pose.orientation.w = q.getW();

    

}

//Callback for central camera detection
void central_cue_cb2(const geometry_msgs::PolygonStamped msg)
{
    if(!central_detection)
    {
        peripheral_detection = true;
        targ_bearing[1] = msg.polygon.points[0].x; // azim
        targ_bearing[0] = msg.polygon.points[0].y; // elev
    }
    else
        peripheral_detection = false;

}
void peripheral_target_acquired(const std_msgs::Bool msg)
{
    target_acquired_by_central = msg.data;
}

//Callback for peripheral camera detection
void central_detec_cb(const geometry_msgs::PolygonStamped msg)
{
    geometry_msgs::Point32 box_pos, img_size, box_size;    

    box_pos = msg.polygon.points[0];
    img_size = msg.polygon.points[1];
    box_size = msg.polygon.points[2];
    
    area_ratio = (box_size.x*box_size.y)/(img_size.x*img_size.y);

    if(target_acquired_by_central && first_in)
    {
        //Once the gimbal follows the omnicam cue, then follow the central detection.

            first_in = false;
            wait = 0;
            central_detection = true;

            std_msgs::Bool central_detection_msgs;
            central_detection_msgs.data = true;
            central_detection_pub.publish(central_detection_msgs);
    }

        // // Set setpoint to half of image widht/height (center)
        // tmp_msg.data = img_size.x/2;
        // yaw_sp_pub.publish(tmp_msg);
        // tmp_msg.data = img_size.y/2;
        // pitch_sp_pub.publish(tmp_msg);

        // // Set state to filter input
        // tmp_msg.data = box_pos.x;
        // yaw_st_pub.publish(tmp_msg);
        // tmp_msg.data = box_pos.y;
        // pitch_st_pub.publish(tmp_msg);


    if(wait > 200)
    {
        central_detection = false;
        first_in = true;
        std_msgs::Bool central_detection_msgs;
        central_detection_msgs.data = false;
        central_detection_pub.publish(central_detection_msgs);
    }

    if(central_detection)
    {       
        //Target Bearing based gimbal control
        
        //Calculate Focal Length (linear interpolation)
        double focal_length = (static_cast<double>(zoom)/100)*(57.6-4.8) + 4.8;

        //Caluculate FOV
        double fov_h = 2*atan((4.8/2)/focal_length);
        double fov_v = 2*atan((3.6/2)/focal_length);
        
        //Calculate delta of box from image center
        delta_x = img_size.x/2 - box_pos.x;
        delta_y = img_size.y/2 - box_pos.y;

        // cout << "FOV = " << fov_h << " " << fov_v <<endl;
        
        //Calculate Target Bearing
        //Elevation (pitch)
        targ_bearing[0] = (fov_v/img_size.y)*delta_y + gim_angles[1];
        //Azimuth (yaw)
        targ_bearing[1] = (fov_h/img_size.x)*delta_x + gim_angles[2];

    }


}

//Callback for IMU on omnicam/central cam
void central_att_cb(const sensor_msgs::Imu msg)
{
    Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
    Matrix3x3 m(q);
    m.getRPY(omni_angles[0], omni_angles[1], omni_angles[2]);

    // cout << "Roll = " << roll*180/M_PI << "   Pitch = " << pitch*180/M_PI << "    Yaw = " << yaw*180/M_PI-90 << endl;
}

//Mavros Callback for Manual RC Override of Gimbal
void joystick_callback(const sensor_msgs::Joy msg)
{
    // cout << sizeof(msg.axes) << "   " << sizeof(msg.buttons) << endl;
    std_msgs::Int8MultiArray zm;
    zm.data.clear();

    if (sizeof(msg.buttons) > 0)
    {
        if (msg.buttons[9] == 1 && counter > 30)
        {
            counter = 0;
            manual_control = !manual_control;
            ROS_INFO("Manual Control is %d",manual_control);
        }
        zm.data.push_back(msg.buttons[8]);
    }
    

    if (counter < 31)
    {
        counter++;
    }

    if (sizeof(msg.axes) > 0)
    {
        pit_cmd = msg.axes[1];
        yaw_cmd = msg.axes[2];
        zm.data.push_back(round((msg.axes[3]+1)*100/2));
    }

    zm.data.push_back(manual_control);
    manual_zoom.publish(zm);

    gm_man_speed_msg.header.stamp = ros::Time::now();
    gm_man_speed_msg.twist.angular.x = 0;
    gm_man_speed_msg.twist.angular.y = -60*pit_cmd;
    gm_man_speed_msg.twist.angular.z = -60*yaw_cmd;


}
int output_count = 0;
void gimbal_angles_cb(const geometry_msgs::PoseStamped msg)
{
    Quaternion q(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
    Matrix3x3 m(q);
    m.getRPY(gim_angles[1], gim_angles[0], gim_angles[2]);

    std_msgs::Float64 tmp_msg;

    // Set setpoint to target bearing
    tmp_msg.data = targ_bearing[0]*180/M_PI;
    pitch_sp_pub.publish(tmp_msg);
    tmp_msg.data = targ_bearing[1]*180/M_PI;
    yaw_sp_pub.publish(tmp_msg);


    // Set state to gimbal angles
    tmp_msg.data = gim_angles[1]*180/M_PI; //Gimbal current pitch angle
    pitch_st_pub.publish(tmp_msg);
    tmp_msg.data = gim_angles[2]*180/M_PI; //Gimbal current yaw angle
    yaw_st_pub.publish(tmp_msg);

    //Publish twiststamped message to gimbal speed topic (pitch_ce/yaw_ce normalized)
    usleep(1000);

    // Attenuation based on camera zoom
    max_ang_spd = 1 - (0.3 * zoom/100) - area_ratio;
    

    gm_speed_msg.header.stamp = ros::Time::now();
    gm_speed_msg.twist.angular.x = 0;
    gm_speed_msg.twist.angular.y = pitch_ce*max_ang_spd;
    gm_speed_msg.twist.angular.z = yaw_ce*max_ang_spd;

    if (output_count >= 60)
    {
        cout << "GOVERNOR_NODE---------------------------------------------------------------------" << endl;
        cout << "Gimbal Roll = " << gim_angles[0]*180/M_PI << "   Pitch = " << gim_angles[1]*180/M_PI << "    Yaw = " << gim_angles[2]*180/M_PI << endl;
        cout << "Deltas X = " << delta_x << "   Y = " << delta_y << endl;
        cout << "Target Elev = " << targ_bearing[0]*180/M_PI << "    Azim =  " << targ_bearing[1]*180/M_PI << endl;
        cout << "Attenuation:   " << max_ang_spd << "   Zoom:   " << zoom << endl;
        cout << "Area Ratio:   " << area_ratio << endl << endl;

        output_count = 0;
    }

    output_count++;

}

//Callback for camera zoom
void zoom_cb(const std_msgs::Float64 msg)
{
    zoom = msg.data;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "governor_node");
    
    // ros::NodeHandle n;
    
    ImageConverter ic;
    

    // Gimbal Control Topics - Publish to only one of these at a time! The driver will prioritize speed control if both are sent.
    ros::Publisher ang_pub = ic.n.advertise<geometry_msgs::PoseStamped>("/gimbal_target_orientation", 1);
    ros::Publisher spd_pub = ic.n.advertise<geometry_msgs::TwistStamped>("/gimbal_target_speed", 1);

    // PID node topics
    // sp = Setpoint, st = state, ce = control effort, en = enable  
    yaw_sp_pub = ic.n.advertise<std_msgs::Float64>("/gimbal_yaw/setpoint", 1);
    yaw_st_pub = ic.n.advertise<std_msgs::Float64>("/gimbal_yaw/state", 1);
    yaw_en_pub = ic.n.advertise<std_msgs::Bool>("/gimbal_yaw/pid_enable", 1);    
    ros::Subscriber yaw_ce_sub = ic.n.subscribe("/gimbal_yaw/control_effort",1, yaw_ce_sub_cb);
    central_detection_pub = ic.n.advertise<std_msgs::Bool>("/central_detection", 1);    


    pitch_sp_pub = ic.n.advertise<std_msgs::Float64>("/gimbal_pitch/setpoint", 1);
    pitch_st_pub = ic.n.advertise<std_msgs::Float64>("/gimbal_pitch/state", 1);
    pitch_en_pub = ic.n.advertise<std_msgs::Bool>("/gimbal_pitch/pid_enable", 1);    
    ros::Subscriber pitch_ce_sub = ic.n.subscribe("/gimbal_pitch/control_effort",1, pitch_ce_sub_cb);

    // Detection Messages - A message coming into these means something has been detected
    ros::Subscriber central_cue = ic.n.subscribe("/gimbal_cue",1, central_cue_cb);
    ros::Subscriber central_cue2 = ic.n.subscribe("/target_bearing_periph",1, central_cue_cb2);
    ros::Subscriber peripheral_detection_bool = ic.n.subscribe("/target_acquired",1, peripheral_target_acquired);


    ros::Subscriber periph_detec = ic.n.subscribe("/yolo_detection_box",1, central_detec_cb);
    // ros::Subscriber periph_detec = ic.n.subscribe("/filtered_box",1, periph_detec_cb);
    ros::Subscriber central_att = ic.n.subscribe("mavros/imu/data",1, central_att_cb);
    ros::Subscriber gimbal_angles = ic.n.subscribe("/gimbal_imu_angles",1, gimbal_angles_cb);


    // Subscribe to Pixhawk RC commands from MAVROS
    ros::Subscriber sub = ic.n.subscribe("/joy", 10, &joystick_callback);

    ros::Subscriber zoom = ic.n.subscribe("/zoom",1, zoom_cb);


    manual_zoom = ic.n.advertise<std_msgs::Int8MultiArray>("man_zoom", 1);


    ros::Rate loop_rate(60);

    int no_detect = 0, peri_detect_timeout = 0;
    stringstream s;

    while(ros::ok())
    {
        if(peripheral_detection && !manual_control)
        {
            // Listen to the peripheral vision cue before the gimbal looking at the target
            spd_pub.publish(gm_speed_msg);
            no_detect = 0;
            peri_detect_timeout = 30;
        }        
        else if(central_detection && !manual_control)
        {
            // Once the gimbal looks at the target, then listen to the central vision cue.
            spd_pub.publish(gm_speed_msg);
            no_detect = 0;
            peri_detect_timeout = 30;
        }
        else if (manual_control)
        {
            spd_pub.publish(gm_man_speed_msg);
            no_detect = 0;
        }
        else    //Wait for peri_detect_timeout before returning to central cue after peripheral loss
        {
            peri_detect_timeout--;
            
            if(peri_detect_timeout < 0)
            {
                peri_detect_timeout = 0;
            }
        }

        if(central_detection)
        {
            wait++;
            cout<<"Wait for the next target... "<<wait<<endl;
        }

        if(!peripheral_detection && !central_detection)
        {
            no_detect++;
            // s.
            // putText(frame,"")
        }

        if(no_detect > 50)
        {
            gm_speed_msg.twist.angular.x = 0;
            gm_speed_msg.twist.angular.y = 0;
            gm_speed_msg.twist.angular.z = 0;
            spd_pub.publish(gm_speed_msg);
        }    

        // cout << "Pitch Command: " << pit_cmd << "   Yaw Command: " << yaw_cmd << endl;

        // cout<< "Periph Detect: " << peripheral_detection << "   Central Detect: " << central_detection <<  "    No Detect: " << no_detect << endl << endl;

        // s.str("");
        // // s  << "Range = " << range << "  " << "Meas Flag = " << meas_flag << "  " << "Meas Count = " << meas_counter;
        // s  << "WHAT";
        // putText(cv_ptr->image, s.str(), Point(10, 50), FONT_HERSHEY_SIMPLEX, 1.25, Scalar(100, 0, 255), 4);

        // ic.image_pub_.publish(cv_ptr->toImageMsg());

        //peripheral_detection = false;
        //central_detection = false;
        
        ros::spinOnce();    
        loop_rate.sleep();

    }

    gm_speed_msg.twist.angular.x = 0;
    gm_speed_msg.twist.angular.y = 0;
    gm_speed_msg.twist.angular.z = 0;
    spd_pub.publish(gm_speed_msg);            
    

    return(0);
}  
