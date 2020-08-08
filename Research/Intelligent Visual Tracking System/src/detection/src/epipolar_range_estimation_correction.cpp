#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <vector>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

static const std::string OPENCV_WINDOW = "Image window";
using namespace cv;
using std::vector;
using namespace std;
ofstream myfile;

geometry_msgs::Vector3Stamped rotated_normal_vector;
geometry_msgs::Vector3Stamped rotated_fisheye_vector;
geometry_msgs::Vector3Stamped fisheye_vector;


visualization_msgs::Marker arrow_normal, arrow_fisheye, rotated_arrow_normal, rotated_arrow_fisheye;
double omnicam_location_enu[3] = {1.8, 0.034, -0.1445};
double gimbal_location_enu[3] = {-0.2, 0.082, -0.397};


// listener.lookupTransform("rtk_base_station", "gimbal", ros::Time(0), transform);


double f_cx=721.2172155119429;
double f_cy=721.9473150432297;

//double b=4786.405854; // 5m
double b=1851.086438; // 2m
double gige_sensor_width_half=4.8/2;
double gige_sensor_height_half=3.6/2;
double gige_pixel_size=0.00375;  

double fisheye_sensor_width=3.2; //3.2
double fisheye_sensor_height=3.2; //3.2
double fl_1=0.1;
double fl_2;

double alpha;

double normal_roll, normal_pitch, normal_yaw;
double fisheye_roll, fisheye_pitch, fisheye_yaw;

Mat normal_cam_frame;

Mat basler_lo=(Mat_<double>(2, 1) << -2.162, 2.527);
Mat omni_lo=(Mat_<double>(2, 1) << -3.985, 4.145);

double ro_angle=atan(1.618/1.823);
Mat drone_r = (Mat_<double>(2, 2) << cos(ro_angle), -sin(ro_angle), sin(ro_angle), cos(ro_angle));



//GigE zoom0
Mat gige0_K = (Mat_<double>(3, 3) << 1296.6925639253514, 0.0, 642.7956006508207, 0.0, 1305.4672890088902, 475.32342351959164, 0.0, 0.0, 1.0);
Mat gige0_D = (Mat_<double>(1, 4) << -0.289312, 0.359446, -0.000467, 0.000284);

//GigE zoom50
Mat gige50_K = (Mat_<double>(3, 3) << 3251.4617167620245, 0.0, 664.6412636008588, 0.0, 3274.9962244450044, 474.08490776791405, 0.0, 0.0, 1.0);
Mat gige50_D = (Mat_<double>(1, 4) << -0.029953, -1.056533, -0.010504, 0.010500);

Mat rotation_gige, rotation_fisheye;
Mat rotation_gige_inv, rotation_fisheye_inv;

double n_cx;
double n_cy;

double zoom;
int zoom_compute;

int maximum_length =4;
int minimum_length = 0;
int find_number = 8;
int target_determination = 9;


double normal_x, normal_y, fisheye_x, fisheye_y;
double drone_x, drone_y, drone_z;
double ground_truth;

Point prepoint1,prepoint2,prepoint3,prepoint4,prepoint5,prepoint6,prepoint7;

const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
const int MAX_NUM_OBJECTS=50;
const int MIN_OBJECT_AREA = 10*10;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;

class ImageConverter
{

public: 


// Mat minus=drone_r*basler_lo;

void chatterCallback(const nav_msgs::Odometry& msg)	
{
//   geometry_msgs::Point coord = msg.pose.pose.position;
// 	drone_x = coord.x;
// 	drone_y = coord.y;
// 	drone_z = coord.z;
// 	Mat drone = drone_r * (Mat_<double>(2, 1) << drone_x, drone_y);
//   ground_truth=drone.at<double>(1,0)-minus.at<double>(1,0);
  //cout<<R23<<endl;

}

void normal_point(const geometry_msgs::PolygonStamped& msg)	
{
		        
        normal_x = msg.polygon.points[0].x;
		normal_y = msg.polygon.points[0].y;
        double BBS_x = msg.polygon.points[2].x;
		double BBS_y = msg.polygon.points[2].y;


    double n_x=(normal_x-n_cx)/1280*gige_sensor_width_half;
    double n_y=(n_cy-(normal_y-BBS_y*0.4))/960*gige_sensor_height_half;

    geometry_msgs::Vector3Stamped normal_vector;
    normal_vector.header.frame_id = "gimbal_camera";
    normal_vector.header.stamp = ros::Time(0);
    normal_vector.vector.x = n_x; //ENU Frame
    normal_vector.vector.y = fl_2; //ENU Frame
    normal_vector.vector.z = n_y; //ENU Frame
    //cout<<fl_2<<endl;
    cout<<"gimbal vec: "<<normal_vector.vector.x<<" "<<normal_vector.vector.y<<" "<<normal_vector.vector.z<<endl;
    listener.transformVector("observer_aircraft", ros::Time(0), normal_vector, "gimbal_camera", rotated_normal_vector);

    // double abc = 4.0 * atan2(transform.getOrigin().y(), transform.getOrigin().x());
    // double def = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));

    // cout<<"normal_rotated"<<endl<<rotated_normal_vector<<endl;


    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    arrow_normal.header.frame_id = "gimbal_camera";
    arrow_normal.header.stamp = ros::Time::now();

    arrow_normal.ns = "arrow_gimbal";
    arrow_normal.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    arrow_normal.type = visualization_msgs::Marker::ARROW;
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    arrow_normal.action = visualization_msgs::Marker::ADD;

    arrow_normal.points.resize(2);
    arrow_normal.points[0].x = -0;
    arrow_normal.points[0].y = 0;
    arrow_normal.points[0].z = -0;
    arrow_normal.points[1].x = normal_vector.vector.x*100;
    arrow_normal.points[1].y = normal_vector.vector.y*100;
    arrow_normal.points[1].z = normal_vector.vector.z*100;

    // Set the scale of the mararrow_normal
    arrow_normal.scale.x = 0.1;
    arrow_normal.scale.y = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    arrow_normal.color.r = 0.0f;
    arrow_normal.color.g = 1.0f;
    arrow_normal.color.b = 0.0f;
    arrow_normal.color.a = 1.0;

    marker_pub.publish(arrow_normal);


    // // Set the namespace and id for this marker.  This serves to create a unique ID
    // // Any marker sent with the same namespace and id will overwrite the old one
    // rotated_arrow_normal.header.frame_id = "observer_aircraft";
    // rotated_arrow_normal.header.stamp = ros::Time::now();

    // rotated_arrow_normal.ns = "arrow_gimbal_rtk";
    // rotated_arrow_normal.id = 2;

    // // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    // rotated_arrow_normal.type = visualization_msgs::Marker::ARROW;
    // // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    // rotated_arrow_normal.action = visualization_msgs::Marker::ADD;

    // rotated_arrow_normal.points.resize(2);
    // rotated_arrow_normal.points[0].x = 0.0f;
    // rotated_arrow_normal.points[0].y = 0.0f;
    // rotated_arrow_normal.points[0].z = 0.0f;
    // rotated_arrow_normal.points[1].x = rotated_normal_vector.vector.x*100;
    // rotated_arrow_normal.points[1].y = rotated_normal_vector.vector.y*100;
    // rotated_arrow_normal.points[1].z = rotated_normal_vector.vector.z*100;

    // // Set the scale of the marker -- 1x1x1 here means 1m on a side
    // rotated_arrow_normal.scale.x = 0.1;
    // rotated_arrow_normal.scale.y = 0.2;

    // // Set the color -- be sure to set alpha to something non-zero!
    // rotated_arrow_normal.color.r = 0.0f;
    // rotated_arrow_normal.color.g = 0.0f;
    // rotated_arrow_normal.color.b = 1.0f;
    // rotated_arrow_normal.color.a = 1.0;

    // marker_pub.publish(rotated_arrow_normal);

}

void fisheye_point(const geometry_msgs::Twist& msg)	
{
	// 	fisheye_x = msg.linear.x;
	// 	fisheye_y = msg.linear.y;

    // double f_x=(fisheye_x-f_cx)/1440*fisheye_sensor_width;
    // double f_y=(f_cy-fisheye_y)/1440*fisheye_sensor_height;
    listener.lookupTransform("gimbal_camera", ros::Time(0), "omnicam", ros::Time(0), "observer_aircraft", transform); // from omnicam frame to gimbal frame

    // bool fisheye_front;
    // nh_2.getParam("/range_estimation_correction/fisheye_front2",fisheye_front);

    fisheye_vector.header.frame_id = "omnicam";
    fisheye_vector.header.stamp = ros::Time(0); 
    // if(fisheye_front)
    // {
    //   fisheye_vector.vector.x = -msg.linear.x/1440*fisheye_sensor_width*0.001; //ENU Frame ( we are using back side camera of omnicam)
    //   fisheye_vector.vector.y = -msg.linear.z/1440*fisheye_sensor_width*0.001; //ENU Frame ( we are using back side camera of omnicam)
    //   fisheye_vector.vector.z = -msg.linear.y/1440*fisheye_sensor_width*0.001; //ENU Frame ( we are using back side camera of omnicam)
    // }
    // else
    // {
    //   fisheye_vector.vector.x = msg.linear.x/1440*fisheye_sensor_width*0.001; //ENU Frame ( we are using back side camera of omnicam)
    //   fisheye_vector.vector.y = msg.linear.z/1440*fisheye_sensor_width*0.001; //ENU Frame ( we are using back side camera of omnicam)
    //   fisheye_vector.vector.z = msg.linear.y/1440*fisheye_sensor_width*0.001; //ENU Frame ( we are using back side camera of omnicam)
    // }
    
    fisheye_vector.vector.x = msg.linear.x; 
    fisheye_vector.vector.y = msg.linear.y; 
    fisheye_vector.vector.z = msg.linear.z; 

    listener.transformVector("observer_aircraft", ros::Time(0), fisheye_vector, "omnicam", rotated_fisheye_vector);


    // cout<<"fisheye_rotated"<<endl<<rotated_fisheye_vector<<endl;


    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    arrow_fisheye.header.frame_id = "omnicam";
    arrow_fisheye.header.stamp = ros::Time::now();

    arrow_fisheye.ns = "arrow_omnicam";
    arrow_fisheye.id = 1;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    arrow_fisheye.type = visualization_msgs::Marker::ARROW;
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    arrow_fisheye.action = visualization_msgs::Marker::ADD;

    arrow_fisheye.points.resize(2);
    arrow_fisheye.points[0].x = 0.0f;
    arrow_fisheye.points[0].y = 0.0f;
    arrow_fisheye.points[0].z = 0.0f;
    arrow_fisheye.points[1].x = fisheye_vector.vector.x*100;
    arrow_fisheye.points[1].y = fisheye_vector.vector.y*100;
    arrow_fisheye.points[1].z = fisheye_vector.vector.z*100;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    arrow_fisheye.scale.x = 0.1;
    arrow_fisheye.scale.y = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    arrow_fisheye.color.r = 0.0f;
    arrow_fisheye.color.g = 1.0f;
    arrow_fisheye.color.b = 0.0f;
    arrow_fisheye.color.a = 1.0;

    marker_pub.publish(arrow_fisheye);


    // // Set the namespace and id for this marker.  This serves to create a unique ID
    // // Any marker sent with the same namespace and id will overwrite the old one
    // rotated_arrow_fisheye.header.frame_id = "observer_aircraft";
    // rotated_arrow_fisheye.header.stamp = ros::Time::now();

    // rotated_arrow_fisheye.ns = "arrow_omnicam_rtk";
    // rotated_arrow_fisheye.id = 3;

    // // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    // rotated_arrow_fisheye.type = visualization_msgs::Marker::ARROW;
    // // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    // rotated_arrow_fisheye.action = visualization_msgs::Marker::ADD;

    // rotated_arrow_fisheye.points.resize(2);
    // rotated_arrow_fisheye.points[0].x = 0.0f;
    // rotated_arrow_fisheye.points[0].y = 0.0f;
    // rotated_arrow_fisheye.points[0].z = 0.0f;
    // rotated_arrow_fisheye.points[1].x = rotated_fisheye_vector.vector.x*100;
    // rotated_arrow_fisheye.points[1].y = rotated_fisheye_vector.vector.y*100;
    // rotated_arrow_fisheye.points[1].z = rotated_fisheye_vector.vector.z*100;

    // // Set the scale of the marker -- 1x1x1 here means 1m on a side
    // rotated_arrow_fisheye.scale.x = 0.1;
    // rotated_arrow_fisheye.scale.y = 0.2;

    // // Set the color -- be sure to set alpha to something non-zero!
    // rotated_arrow_fisheye.color.r = 0.0f;
    // rotated_arrow_fisheye.color.g = 0.0f;
    // rotated_arrow_fisheye.color.b = 1.0f;
    // rotated_arrow_fisheye.color.a = 1.0;

    // marker_pub.publish(rotated_arrow_fisheye);

}



void range_callback(const std_msgs::Float64::ConstPtr& msg)
{
	double range=msg->data;
	
	
	// zoom = sensor_size*range*prcnt_coverage/target_size
    double desired_focal = 0.00626*range*0.1/1;

    if(desired_focal <= 0.0048)
	{
		zoom = 0;
	}
	else if(desired_focal >= 0.0576)
    {
		zoom = 100;
	}    
    else
        zoom_compute = floor((1893.93*desired_focal-6.09)/10.0);
		
    zoom = zoom_compute * 10;

    //zoom = 0;

    ground_truth=range;

    if(zoom==0)
    {
        fl_2=gige0_K.at<double>(0,0) * gige_pixel_size;
        alpha=fl_2/fl_1;
        n_cx=gige0_K.at<double>(0,2);
        n_cy=gige0_K.at<double>(1,2);
    }

    if(zoom==10)
    {
        fl_2=10.08;
        alpha=fl_2/fl_1;
        n_cx=gige0_K.at<double>(0,2);
        n_cy=gige0_K.at<double>(1,2);
    }

    if(zoom==20)
    {
        fl_2=15.36;
        alpha=fl_2/fl_1;
        n_cx=gige0_K.at<double>(0,2);
        n_cy=gige0_K.at<double>(1,2);
    }

    if(zoom==30)
    {
        fl_2=20.64;
        alpha=fl_2/fl_1;
        n_cx=gige0_K.at<double>(0,2);
        n_cy=gige0_K.at<double>(1,2);
    }

    if(zoom==40)
    {
        fl_2=25.92;
        alpha=fl_2/fl_1;
        n_cx=gige0_K.at<double>(0,2);
        n_cy=gige0_K.at<double>(1,2);
    }

    if(zoom==50)
    {
        fl_2=31.2;
        alpha=fl_2/fl_1;
        n_cx=gige0_K.at<double>(0,2);
        n_cy=gige0_K.at<double>(1,2);
    }

    if(zoom==60)
    {
        fl_2=36.48;
        alpha=fl_2/fl_1;
        n_cx=gige0_K.at<double>(0,2);
        n_cy=gige0_K.at<double>(1,2);
    }

    if(zoom==70)
    {
        fl_2=41.76;
        alpha=fl_2/fl_1;
        n_cx=gige0_K.at<double>(0,2);
        n_cy=gige0_K.at<double>(1,2);
    }

    if(zoom==80)
    {
        fl_2=47.04;
        alpha=fl_2/fl_1;
        n_cx=gige0_K.at<double>(0,2);
        n_cy=gige0_K.at<double>(1,2);
    }

    if(zoom==90)
    {
        fl_2=52.32;
        alpha=fl_2/fl_1;
        n_cx=gige0_K.at<double>(0,2);
        n_cy=gige0_K.at<double>(1,2);
    }

    if(zoom==100)
    {
        fl_2=57.6;
        alpha=fl_2/fl_1;
        n_cx=gige0_K.at<double>(0,2);
        n_cy=gige0_K.at<double>(1,2);
    }




	cout<<"range :"<<range<<" zoom :"<<zoom<<" focal length:"<<fl_2<<endl;

}


void zoom_callback(const std_msgs::Float64::ConstPtr& msg)
{
	double zoom_actual=msg->data;
	

    fl_2 = zoom_actual/100 * (57.6-4.8) + 4.8;



}

private:
	double x, y, z;
	double g_x;
	double g_y;

  Mat T_mxkmGkm = (Mat_<double>(2,1) << 0, 0);
  Mat T_SxkmGkm = (Mat_<double>(2,2) << 5, 0, 0, 5);
  ros::NodeHandle nh_2;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber image_sub2_;

  image_transport::Publisher image_pub_;
  ros::Subscriber pose_sub;
    
  ros::Subscriber normal_sub;
  ros::Subscriber fisheye_sub;

  ros::Subscriber range_sub;
  ros::Subscriber zoom_sub;

  tf::TransformListener listener;
  tf::StampedTransform transform;

  ros::Publisher p_pub;
  ros::Publisher s_pub;
  ros::Publisher target_pub;
  ros::Publisher marker_pub;
  ros::Publisher uncertainty_pub;

public:
  ImageConverter()
  : it_(nh_2)
  {
    // Subscrive to input video feed and publish output video feed
        image_transport::TransportHints hints("compressed", ros::TransportHints());
    image_sub_ = it_.subscribe("/undistort_fisheye", 1,
    &ImageConverter::imageCb2, this);
    image_sub2_ = it_.subscribe("/camera/image_raw", 1,
    &ImageConverter::imageCb, this, hints);
    
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    normal_sub = nh_2.subscribe("/yolo_detection_box", 1000, &ImageConverter::normal_point, this);
    fisheye_sub = nh_2.subscribe("/image_frame_point_fisheye", 1000, &ImageConverter::fisheye_point, this);

    pose_sub = nh_2.subscribe("/hexacopter/gps/rtkfix", 1000, &ImageConverter::chatterCallback, this);
    range_sub = nh_2.subscribe("/range", 1000, &ImageConverter::range_callback, this);
    zoom_sub = nh_2.subscribe("/zoom", 1000, &ImageConverter::zoom_callback, this);
    marker_pub = nh_2.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // listener.waitForTransform("rtk_base_station", "base", ros::Time(0), ros::Duration(3));


    p_pub = nh_2.advertise<geometry_msgs::Twist>("/rbe_target_point", 10);
    s_pub = nh_2.advertise<geometry_msgs::Twist>("/observation_global_point", 10);
    target_pub = nh_2.advertise<geometry_msgs::Twist>("/target_epipolar", 10);
    uncertainty_pub = nh_2.advertise<geometry_msgs::Twist>("/target_uncertainty", 10);

    cv::namedWindow(OPENCV_WINDOW);
    x = 0;
    y = 0;
    z = 0;
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }



  void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat src_normal;

    src_normal = cv_ptr->image;




    Mat fisheye_location = (Mat_<double>(3,1) << omnicam_location_enu[0], omnicam_location_enu[1], omnicam_location_enu[2]); // ENU frame
    Mat normal_location = (Mat_<double>(3,1) << gimbal_location_enu[0], gimbal_location_enu[1], gimbal_location_enu[2]); // ENU frame
    Mat baseline_vector = fisheye_location - normal_location;   //ENU frame

    tf::Quaternion q = transform.getRotation();
    tf::Matrix3x3 rotation_matrix(q); // rotation from omnicam to gimbal
    double translation_x = transform.getOrigin().x();
    double translation_y = transform.getOrigin().y();
    double translation_z = transform.getOrigin().z();

    tf::Vector3 translation_vector(translation_x, translation_y, translation_z);
    tf::Vector3 fisheye_vector3(fisheye_vector.vector.x, fisheye_vector.vector.y, fisheye_vector.vector.z);
    tf::Vector3 fisheye_in_gimbal_frame = rotation_matrix * fisheye_vector3 + translation_vector;


    double azi = atan(fisheye_vector.vector.x/fisheye_vector.vector.y);
    double ele = atan(fisheye_vector.vector.z/sqrt(fisheye_vector.vector.x*fisheye_vector.vector.x+fisheye_vector.vector.y*fisheye_vector.vector.y));

    double azi2 = atan(fisheye_in_gimbal_frame.x()/fisheye_in_gimbal_frame.y());
    double ele2 = atan(fisheye_in_gimbal_frame.z()/sqrt(fisheye_in_gimbal_frame.x()*fisheye_in_gimbal_frame.x()+fisheye_in_gimbal_frame.y()*fisheye_in_gimbal_frame.y()));

    cout<<"cue angle comparison: "<<azi*180/3.141592<<","<<azi2*180/3.141592<<" "<<ele*180/3.141592<<","<<ele2<<endl;


    Mat p_l = (Mat_<double>(3,1) << rotated_fisheye_vector.vector.x, rotated_fisheye_vector.vector.y, rotated_fisheye_vector.vector.z); // NED frame
    Mat p_r = (Mat_<double>(3,1) << rotated_normal_vector.vector.x, rotated_normal_vector.vector.y, rotated_normal_vector.vector.z); // NED frame


    ///////////////Method 1. Cross product method -> Working better
    double x_l = p_l.at<double>(0,0);
    double y_l = p_l.at<double>(0,1);
    double z_l = p_l.at<double>(0,2);
    double x_r = p_r.at<double>(0,0);
    double y_r = p_r.at<double>(0,1);
    double z_r = p_r.at<double>(0,2);
    double x_b = baseline_vector.at<double>(0,0);
    double y_b = baseline_vector.at<double>(0,1);
    double z_b = baseline_vector.at<double>(0,2);


    Mat pr_vb = (Mat_<double>(3,1) << -z_r*y_b+y_r*z_b, z_r*x_b-x_r*z_b, -y_r*x_b+x_r*y_b);
    Mat pr_pl = (Mat_<double>(3,1) << -z_r*y_l+y_r*z_l, z_r*x_l-x_r*z_l, -y_r*x_l+x_r*y_l);

    double prvb=sqrt((-z_r*y_b+y_r*z_b)*(-z_r*y_b+y_r*z_b)+(z_r*x_b-x_r*z_b)*(z_r*x_b-x_r*z_b)+(-y_r*x_b+x_r*y_b)*(-y_r*x_b+x_r*y_b));
    double prpl=sqrt((-z_r*y_l+y_r*z_l)*(-z_r*y_l+y_r*z_l)+(z_r*x_l-x_r*z_l)*(z_r*x_l-x_r*z_l)+(-y_r*x_l+x_r*y_l)*(-y_r*x_l+x_r*y_l));
    
    Mat target_location = prvb/prpl*p_l+fisheye_location;
    // Mat target_location = 4000*p_r+normal_location;
    //////////////////////////////////////////////////////////////////////////////

    ////////////////////Method 2.  Mid-point method
    double x1 = fisheye_location.at<double>(0,0);
    double y1 = fisheye_location.at<double>(0,1);
    double z1 = fisheye_location.at<double>(0,2);
    double x2 = normal_location.at<double>(0,0);
    double y2 = normal_location.at<double>(0,1);
    double z2 = normal_location.at<double>(0,2);

    double Lx = p_l.at<double>(0,0);
    double Ly = p_l.at<double>(0,1);
    double Lz = p_l.at<double>(0,2);
    double Rx = p_r.at<double>(0,0);
    double Ry = p_r.at<double>(0,1);
    double Rz = p_r.at<double>(0,2);
    
    double mu=(Lz*Lz * (Rx*  (x1 - x2) + Ry*  (y1 - y2)) + Ly*Ly*  (Rx * (x1 - x2) + Rz*  (z1 - z2)) + Lx*Lx*  (Ry*  (y1 - y2) + Rz*  (z1 - z2)) + Lx*  Lz*  (Rz*  (-x1 + x2) + Rx * (-z1 + z2)) + Ly*  (Lx*  Ry*  (-x1 + x2) + Lx*  Rx*  (-y1 + y2) + Lz*  Rz*  (-y1 + y2) + Lz*  Ry*  (-z1 + z2)))/(Lz*Lz*  (Rx*Rx*  + Ry*Ry) - 2*  Lx*  Lz*  Rx*  Rz  - 2*  Ly*  Ry*  (Lx*  Rx + Lz * Rz) + Ly*Ly * (Rx*Rx + Rz*Rz) + Lx*Lx*  (Ry*Ry + Rz*Rz));
    double lamda = (Lx*(Ry*Ry*(-x1 + x2) + Rx*Ry*(y1 - y2) + Rz*(-Rz*x1 + Rz*x2 + Rx*z1 - Rx*z2)) +  Ly*(Rx*Ry*(x1 - x2) + Rx*Rx*(-y1 + y2) + Rz*(-Rz*y1 + Rz*y2 + Ry*z1 - Ry*z2)) + Lz*(Rx*Rz*(x1 - x2) + Rx*Rx*(-z1 + z2) + Ry*(Rz*y1 - Rz*y2 - Ry*z1 + Ry*z2)))/(Lz*Lz*(Rx*Rx + Ry*Ry) - 2*Lx*Lz*Rx*Rz - 2*Ly*Ry*(Lx*Rx + Lz*Rz) + Ly*Ly*(Rx*Rx + Rz*Rz) + Lx*Lx*(Ry*Ry + Rz*Rz));
    // Mat target_location = lamda*p_l*10 + fisheye_location;
    //  Mat target_location = mu*p_r*100 + normal_location;
    // Mat target_location = (mu*p_r*100 + normal_location + lamda*p_l*10 + fisheye_location)/2;
    //////////////////////////////////////////////////


    ///// Uncertainty propagation //////
    double dxwdLx = pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2))/pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(1/2)) - (Lx*(2*Ry*(Lx*Ry - Ly*Rx) + 2*Rz*(Lx*Rz - Lz*Rx))*pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2)))/(2*pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(3/2)));
    double dxwdLy = (Lx*(2*Rx*(Lx*Ry - Ly*Rx) - 2*Rz*(Ly*Rz - Lz*Ry))*pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2)))/(2*pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(3/2)));
    double dxwdLz = (Lx*(2*Rx*(Lx*Rz - Lz*Rx) + 2*Ry*(Ly*Rz - Lz*Ry))*pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2)))/(2*pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(3/2)));
    double dxwdRx = (Lx*(2*Ly*(Lx*Ry - Ly*Rx) + 2*Lz*(Lx*Rz - Lz*Rx))*pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2)))/(2*pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(3/2))) - (Lx*(2*(Ry*(x1 - x2) - Rx*(y1 - y2))*(y1 - y2) + 2*(Rz*(x1 - x2) - Rx*(z1 - z2))*(z1 - z2)))/(2*pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(1/2))*pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2)));
    double dxwdRy = (Lx*(2*(Ry*(x1 - x2) - Rx*(y1 - y2))*(x1 - x2) - 2*(Rz*(y1 - y2) - Ry*(z1 - z2))*(z1 - z2)))/(2*pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(1/2))*pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2))) - (Lx*(2*Lx*(Lx*Ry - Ly*Rx) - 2*Lz*(Ly*Rz - Lz*Ry))*pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2)))/(2*pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(3/2)));
    double dxwdRz = (Lx*(2*(Rz*(x1 - x2) - Rx*(z1 - z2))*(x1 - x2) + 2*(Rz*(y1 - y2) - Ry*(z1 - z2))*(y1 - y2)))/(2*pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(1/2))*pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2))) - (Lx*(2*Lx*(Lx*Rz - Lz*Rx) + 2*Ly*(Ly*Rz - Lz*Ry))*pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2)))/(2*pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(3/2)));

    double dywdLx = -(Ly*(2*Ry*(Lx*Ry - Ly*Rx) + 2*Rz*(Lx*Rz - Lz*Rx))*pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2)))/(2*pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(3/2)));
    double dywdLy = pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2))/pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(1/2)) + (Ly*(2*Rx*(Lx*Ry - Ly*Rx) - 2*Rz*(Ly*Rz - Lz*Ry))*pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2)))/(2*pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(3/2)));
    double dywdLz = (Ly*(2*Rx*(Lx*Rz - Lz*Rx) + 2*Ry*(Ly*Rz - Lz*Ry))*pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2)))/(2*pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(3/2)));
    double dywdRx = (Ly*(2*Ly*(Lx*Ry - Ly*Rx) + 2*Lz*(Lx*Rz - Lz*Rx))*pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2)))/(2*pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(3/2))) - (Ly*(2*(Ry*(x1 - x2) - Rx*(y1 - y2))*(y1 - y2) + 2*(Rz*(x1 - x2) - Rx*(z1 - z2))*(z1 - z2)))/(2*pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(1/2))*pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2)));
    double dywdRy = (Ly*(2*(Ry*(x1 - x2) - Rx*(y1 - y2))*(x1 - x2) - 2*(Rz*(y1 - y2) - Ry*(z1 - z2))*(z1 - z2)))/(2*pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(1/2))*pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2))) - (Ly*(2*Lx*(Lx*Ry - Ly*Rx) - 2*Lz*(Ly*Rz - Lz*Ry))*pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2)))/(2*pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(3/2)));
    double dywdRz = (Ly*(2*(Rz*(x1 - x2) - Rx*(z1 - z2))*(x1 - x2) + 2*(Rz*(y1 - y2) - Ry*(z1 - z2))*(y1 - y2)))/(2*pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(1/2))*pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2))) - (Ly*(2*Lx*(Lx*Rz - Lz*Rx) + 2*Ly*(Ly*Rz - Lz*Ry))*pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2)))/(2*pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(3/2)));

    double dzwdLx = -(Lz*(2*Ry*(Lx*Ry - Ly*Rx) + 2*Rz*(Lx*Rz - Lz*Rx))*pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2)))/(2*pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(3/2)));
    double dzwdLy = (Lz*(2*Rx*(Lx*Ry - Ly*Rx) - 2*Rz*(Ly*Rz - Lz*Ry))*pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2)))/(2*pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(3/2)));
    double dzwdLz = pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2))/pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(1/2)) + (Lz*(2*Rx*(Lx*Rz - Lz*Rx) + 2*Ry*(Ly*Rz - Lz*Ry))*pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2)))/(2*pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(3/2)));
    double dzwdRx = (Lz*(2*Ly*(Lx*Ry - Ly*Rx) + 2*Lz*(Lx*Rz - Lz*Rx))*pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2)))/(2*pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(3/2))) - (Lz*(2*(Ry*(x1 - x2) - Rx*(y1 - y2))*(y1 - y2) + 2*(Rz*(x1 - x2) - Rx*(z1 - z2))*(z1 - z2)))/(2*pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(1/2))*pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2)));
    double dzwdRy = (Lz*(2*(Ry*(x1 - x2) - Rx*(y1 - y2))*(x1 - x2) - 2*(Rz*(y1 - y2) - Ry*(z1 - z2))*(z1 - z2)))/(2*pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(1/2))*pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2))) - (Lz*(2*Lx*(Lx*Ry - Ly*Rx) - 2*Lz*(Ly*Rz - Lz*Ry))*pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2)))/(2*pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(3/2)));
    double dzwdRz = (Lz*(2*(Rz*(x1 - x2) - Rx*(z1 - z2))*(x1 - x2) + 2*(Rz*(y1 - y2) - Ry*(z1 - z2))*(y1 - y2)))/(2*pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(1/2))*pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2))) - (Lz*(2*Lx*(Lx*Rz - Lz*Rx) + 2*Ly*(Ly*Rz - Lz*Ry))*pow(((Ry*(x1 - x2) - Rx*(y1 - y2))*(Ry*(x1 - x2) - Rx*(y1 - y2)) + (Rz*(x1 - x2) - Rx*(z1 - z2))*(Rz*(x1 - x2) - Rx*(z1 - z2)) + (Rz*(y1 - y2) - Ry*(z1 - z2))*(Rz*(y1 - y2) - Ry*(z1 - z2))),(1/2)))/(2*pow(((Lx*Ry - Ly*Rx)*(Lx*Ry - Ly*Rx) + (Lx*Rz - Lz*Rx)*(Lx*Rz - Lz*Rx) + (Ly*Rz - Lz*Ry)*(Ly*Rz - Lz*Ry)),(3/2)));

    double sigLx = 0.1;
    double sigLy = 0.1;
    double sigLz = 0.1;

    double sigRx = 0.1;
    double sigRy = 0.1;
    double sigRz = 0.1;

    Mat dFdx = (Mat_<double>(3,6) << dxwdLx*dxwdLx, dxwdLy*dxwdLy, dxwdLz*dxwdLz, dxwdRx*dxwdRx, dxwdRy*dxwdRy, dxwdRz*dxwdRz, dywdLx*dywdLx, dywdLy*dywdLy, dywdLz*dywdLz, dywdRx*dywdRx, dywdRy*dywdRy, dywdRz*dywdRz, dzwdLx*dzwdLx, dzwdLy*dzwdLy, dzwdLz*dzwdLz, dzwdRx*dzwdRx, dzwdRy*dzwdRy, dzwdRz*dzwdRz);
    Mat sigma = (Mat_<double>(6,1) << sigLx*sigLx, sigLy*sigLy, sigLz*sigLz, sigRx*sigRx, sigRy*sigRy, sigRz*sigRz);
    Mat final_sigma_square = dFdx * sigma;
    double target_uncertainty_x =  sqrt(final_sigma_square.at<double>(0,0));
    double target_uncertainty_y =  sqrt(final_sigma_square.at<double>(0,1));
    double target_uncertainty_z =  sqrt(final_sigma_square.at<double>(0,2));    

    //cout<<target_uncertainty_x<<" "<<target_uncertainty_y<<" "<<target_uncertainty_z<<endl;
    double target_range = sqrt(target_location.at<double>(0,0)*target_location.at<double>(0,0)+target_location.at<double>(0,1)*target_location.at<double>(0,1)+target_location.at<double>(0,2)*target_location.at<double>(0,2));

    cout<<target_range<<endl;
    
    // Publish the target location to TF
    geometry_msgs::Twist target_location_for_publish;
    target_location_for_publish.linear.x=target_location.at<double>(0,0);
    target_location_for_publish.linear.y=target_location.at<double>(0,1);
    target_location_for_publish.linear.z=target_location.at<double>(0,2);
    target_pub.publish(target_location_for_publish);

    // Publish the target location uncertainty TF
    geometry_msgs::Twist target_uncertainty_for_publish;
    target_uncertainty_for_publish.linear.x=target_uncertainty_x;
    target_uncertainty_for_publish.linear.y=target_uncertainty_y;
    target_uncertainty_for_publish.linear.z=target_uncertainty_z;
    uncertainty_pub.publish(target_uncertainty_for_publish);    

  //   sprintf(str0,"Ground truth : %f",ground_truth);
  //   sprintf(str,"Estimated range : %f",z);
  //   sprintf(str2,"Error : %f",error);
  //   sprintf(str3,"Percent error: %f",percent);


	// 	putText(src_normal,str0,Point(10,50),2,1,Scalar(0,0,0),2);
	// 	putText(src_normal,str,Point(10,100),2,1,Scalar(0,0,0),2);
 	// 	putText(src_normal,str2,Point(10,150),2,1,Scalar(0,0,0),2);
	// 	putText(src_normal,str3,Point(10,200),2,1,Scalar(0,0,0),2);
  //   putText(src_normal,str5,Point(10,250),2,1,Scalar(0,0,0),2);
	// 	putText(src_normal,str6,Point(10,300),2,1,Scalar(0,0,0),2);

  //   if(normal_x>1280 | normal_y>960 | normal_x <0 | normal_y<0){
  //       myfile << "Outside of FOV \n";
  //       putText(src_normal,str4,Point(10,350),2,1,Scalar(0,0,1),2);
  //   }

  // myfile << ground_truth <<" "<< n_distance_c_mm<<" "<<n_distance_c_pixel<<" "<<f_distance_c_mm<<" "<<f_distance_c_pixel<<" "<< z << " "<< error <<" "<< percent<<"\n";


	//cv::imshow("src", preorig);
  resize(src_normal,src_normal,Size(960,600));    
	//imshow("normal",src_normal);
	int key1 = waitKey(20);

}

 void imageCb2(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr2;
    try
    {
      cv_ptr2 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat src_fishehye;

    src_fishehye = cv_ptr2->image;
    resize(src_fishehye,src_fishehye,Size(500,500));
    imshow("fisheye",src_fishehye);
  	int key1 = waitKey(20);

}



void getEulerAngles(Mat &rotCamerMatrix,Vec3d &eulerAngles){

    Mat cameraMatrix,rotMatrix,transVect,rotMatrixX,rotMatrixY,rotMatrixZ;
    double* _r = rotCamerMatrix.ptr<double>();
    double projMatrix[12] = {_r[0],_r[1],_r[2],0,
                          _r[3],_r[4],_r[5],0,
                          _r[6],_r[7],_r[8],0};

    decomposeProjectionMatrix( Mat(3,4,CV_64FC1,projMatrix),
                               cameraMatrix,
                               rotMatrix,
                               transVect,
                               rotMatrixX,
                               rotMatrixY,
                               rotMatrixZ,
                               eulerAngles);
}


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter_range");
  ImageConverter ic;
  myfile.open ("range.txt");
  ros::spin();
  myfile.close();
  return 0;
}
