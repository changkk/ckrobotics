#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <sstream>
#include <vector>
#include "sensor_msgs/Image.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/PolygonStamped.h"
#include <geometry_msgs/TransformStamped.h>
#include "sensor_msgs/CompressedImage.h"
#include "std_msgs/Float64.h"
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/video.hpp>
#include <cmath>
#include <opencv2/video/tracking.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include "opencv2/cudaoptflow.hpp"
#include "opencv2/cudaarithm.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "ocam_functions.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>



using namespace cv;
// using namespace cv::cuda;
using namespace std;

static const string OPENCV_WINDOW = "Image window";

// Image Resize
bool meas_flag = false, send_cue = false;
int w = 1440, l = 1440;
// int w = 1504, l = 1504;

Size size(w, l);
Size size_null(0,0);
cv_bridge::CvImagePtr cv_ptr;
Ptr<BackgroundSubtractor> pMOG2; //MOG2 Background subtractor
Mat fgMaskMOG2;
Mat fgMaskMOG2_open;
Mat frame;
Mat frame_p = Mat::zeros(size, CV_8UC1);
Mat grad_x;
Mat grad_y;
Mat sobel_img = Mat::zeros(size, CV_8UC1);
Mat d_ft = Mat::zeros(size, CV_8UC1);
Mat mag = Mat::zeros(size, CV_8UC1);
Mat ang = Mat::zeros(size, CV_8UC1);

vector<vector<Point> > contours;
vector<Vec4i> hierarchy;
double z_data[2] = {0, 0};


Mat z = Mat(2,1,CV_64FC1,z_data);


KalmanFilter KF(4,2,0);
KalmanFilter KF_target(4,2,0);
Mat_<float> measurement(2,1); 


Mat and_img, and_img2, canny_edge;

int64 t0 = 0, t1 = 1, tc0 = 0, tc1 = 0;
int no_cons = 0, no_meas_counter = 0, meas_counter = 0;
Point2i est_center;
Point2i est_center_target;


// Mat mat = Mat::zeros(size, CV_8UC1);
Mat bin_out_diff = Mat::zeros(size, CV_8UC1);
char charry[20];

double t_old = 0, t_now = 0, dt = 0, xv = 0, yv = 0;
float range = 0;
std::string to_string(double x);
// Ptr<Tracker> tracker;


class ImageConverter
{
  ros::NodeHandle nh_;
  ros::Subscriber sub;
  ros::Publisher gimbal_cue;
  ros::Publisher target_vector;
  ros::Subscriber imu_sub;

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
 

public:
  ImageConverter()
      : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_transport::TransportHints hints("compressed", ros::TransportHints());
    image_sub_ = it_.subscribe("/omnicam/image_raw", 1, &ImageConverter::imageCb, this, hints);
    sub = nh_.subscribe("range", 10, &ImageConverter::rangeCB, this);
    
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    // gimbal_cue = nh_.advertise("gimbal_cue", 1);
    gimbal_cue = nh_.advertise<geometry_msgs::PolygonStamped>("gimbal_cue",1);
    target_vector = nh_.advertise<geometry_msgs::Twist>("/image_frame_point_fisheye", 10);
    imu_sub = nh_.subscribe("/mavros/imu/data", 1000, &ImageConverter::imuCallback, this);


    pMOG2 = createBackgroundSubtractorMOG2(30,30);
    cout << getBuildInformation() << endl;

    
    
 
    KF.statePre.at<float>(0) = w/2;
    KF.statePre.at<float>(1) = l/2;
    KF.statePre.at<float>(2) = 0;
    KF.statePre.at<float>(3) = 0;
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(1e-3));
    setIdentity(KF.measurementNoiseCov, Scalar::all(100));
    setIdentity(KF.errorCovPost, Scalar::all(.005));
    
    KF_target.statePre.at<float>(0) = w/2;
    KF_target.statePre.at<float>(1) = l/2;
    KF_target.statePre.at<float>(2) = 0;
    KF_target.statePre.at<float>(3) = 0;
    setIdentity(KF_target.measurementMatrix);
    setIdentity(KF_target.processNoiseCov, Scalar::all(1e-3));
    setIdentity(KF_target.measurementNoiseCov, Scalar::all(1));
    setIdentity(KF_target.errorCovPost, Scalar::all(.005));

    // namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    destroyWindow(OPENCV_WINDOW);
  }

  void rangeCB(const std_msgs::Float64 &msg)
  {
    range = msg.data;
  }

  void imuCallback(const sensor_msgs::Imu& msg2)	
{

  static tf2_ros::TransformBroadcaster br;
  
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "rtk_base_station";
  transformStamped.child_frame_id = "omnicam";

  // transformStamped.transform.translation.x = 7.244; //NED frame
  // transformStamped.transform.translation.y = -4.593; //NED frame
  // transformStamped.transform.translation.z =-0.191; //NED frame

  transformStamped.transform.translation.x = 1.07 ; //NED frame
  transformStamped.transform.translation.y = 0.034; //NED frame
  transformStamped.transform.translation.z =-0.0945; //NED frame

  // tf2::Quaternion q;
  // q.setRPY(0, 0, msg->theta);





        	geometry_msgs::Quaternion q = msg2.orientation;
        //q1=imu.x;
       // q1=imu.y;
        //q1=imu.z;
        //q1=imu.w;


	// roll (x-axis rotation)
	double sinr = +2.0 * (q.w * q.x + q.y * q.z);
	double cosr = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
	double roll = atan2(sinr, cosr);
	double pitch;
	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w * q.y - q.z * q.x);
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny = +2.0 * (q.w * q.z + q.x * q.y);
	double cosy = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
	double yaw = atan2(siny, cosy);

///////////////////////////////////////////////////////////////////////////////////////



	//refers to pixhawk frame, pitch, roll, yaw

	Mat rotation_fisheye;

	double fisheye_roll = 0;
	double fisheye_pitch = 0;
	double fisheye_yaw = 0*3.141592/180;


        // Abbreviations for the various angular functions
	double cy = cos(fisheye_yaw * 0.5);
	double sy = sin(fisheye_yaw * 0.5);
	double cr = cos(fisheye_roll * 0.5);
	double sr = sin(fisheye_roll * 0.5);
	double cp = cos(fisheye_pitch * 0.5);
	double sp = sin(fisheye_pitch * 0.5);

	double q_w = cy * cr * cp + sy * sr * sp;
	double q_x = cy * sr * cp - sy * cr * sp;
	double q_y = cy * cr * sp + sy * sr * cp;
	double q_z = sy * cr * cp - cy * sr * sp;


  transformStamped.transform.rotation.x = q_x;  //NED frame
  transformStamped.transform.rotation.y = q_y;  //NED frame
  transformStamped.transform.rotation.z = q_z;  //NED frame
  transformStamped.transform.rotation.w = q_w;  //NED frame

  br.sendTransform(transformStamped);

	//refers to camera frame, rotation321(roll,pitch,yaw)
	rotation321(fisheye_roll,fisheye_pitch,fisheye_yaw,rotation_fisheye);

	// Check the angles //
	Vec3d eulerAngles_fisheye;
	Mat rotation_fisheye_inv = rotation_fisheye.inv();
	getEulerAngles(rotation_fisheye_inv,eulerAngles_fisheye);


}

  void imageCb(const sensor_msgs::ImageConstPtr &msg)
  {
    // cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //   circle(cv_ptr->image, Point(50, 50), 10, CV_RGB(255,0,0));


    frame = cv_ptr->image;
    frame = frame(Range::all(), Range(0, 1504));
    resize(frame, frame, size);
    flip(frame,frame,-1);
    GaussianBlur(frame, frame, Size(3, 3), 10, 10);

    // bilateralFilter(frame,frame_small,9,18,5);
    // cvtColor(frame, frame_hsv, COLOR_BGR2HSV);
    cvtColor(frame, frame, COLOR_BGR2GRAY);
    

    pMOG2->apply(frame, fgMaskMOG2);

    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(fgMaskMOG2, fgMaskMOG2_open, MORPH_OPEN, element);

    Sobel(frame, grad_x, CV_16S, 1, 0, 1);
    Sobel(frame, grad_y, CV_16S, 0, 1, 1);
    Canny(frame,canny_edge,50,150);

    convertScaleAbs(grad_x, grad_x);
    convertScaleAbs(grad_y, grad_y);
    addWeighted(grad_x, 0.5, grad_y, 0.5, 0, sobel_img);

    
    bitwise_and(sobel_img, fgMaskMOG2_open, and_img); 
    threshold(and_img,and_img,0,255,CV_THRESH_BINARY);

    Mat element1 = getStructuringElement(MORPH_RECT, Size(2,2));     
    Mat element2 = getStructuringElement(MORPH_RECT, Size(1, 1)); 
       
    morphologyEx(and_img, and_img, MORPH_OPEN, element2);

    erode(and_img,and_img,element1);
    erode(and_img,and_img,element1);
    dilate(and_img,and_img,element2);
    dilate(and_img,and_img,element2);


    //Optical Flow--------------------------------------------------------------------------------------------------
    threshold(sobel_img,sobel_img,150,0,THRESH_TOZERO_INV);
    findContours( and_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    // cout << contours.size() << endl;
    if (contours.size() > 1)
    {
      no_cons = contours.size();        
    }
    else
    {
      no_cons = contours.size();
    }  

    /// Approximate contours to polygons + get bounding rects and circles
    vector<vector<Point> > contours_poly(contours.size());
    vector<Rect> boundRect(contours.size());
    vector<Point2f> center(contours.size());
    vector<float> radius(contours.size());

    for (int i = 0; i < no_cons; i++)
    {
      approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
      boundRect[i] = boundingRect(Mat(contours_poly[i]));
      minEnclosingCircle((Mat)contours_poly[i], center[i], radius[i]); // draw circles which include contours
    }

    /// Draw polygonal contour + bonding rects + circles
    Mat drawing = Mat::zeros(and_img.size(), CV_8UC1);
    // and_img.copyTo(drawing);
    frame.copyTo(drawing);

        double max_center=200000;
        double max_center_num=-2;

    for (int i = 0; i < no_cons; i++)
    {
      // Scalar color = ;
      // drawContours(drawing, contours_poly, i, Scalar(255, 0, 255), 1, 8, vector<Vec4i>(), 0, Point());
      // rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), Scalar(255, 0, 255), 2, 8, 0);
      circle(drawing, center[i], radius[i], Scalar(255, 0, 255), 2, 8, 0);      
      rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), Scalar( 255,255,0 ), 2, 8, 0 ); 
        if(center[i].y<max_center)
        {
            max_center = center[i].y;
            max_center_num = i;
        }

    }


    t_now = ros::Time::now().toSec();
    dt = t_now - t_old;
    t_old = t_now;

    // cvSort

    if (center.size() > 0 )
    {
      if ( center[max_center_num].x != 0 & center[max_center_num].y != 0)
      {
        // cout << center[0].x << ", " << center[0].y << endl;

        z.at<double>(0,0) = center[max_center_num].x;
        z.at<double>(1,0) = center[max_center_num].y;
        meas_flag = true;
        no_meas_counter = 0;
        meas_counter++;

      }
      else
      {
        meas_flag = false;
        no_meas_counter++;
        meas_counter = 0;
      }
    }
    else
    {
      meas_flag = false;
      no_meas_counter++;
      meas_counter = 0;
      
    }

    //Kalman Filter

    if (no_meas_counter < 20)    
    {
      Mat prediction = KF.predict();
      Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
      est_center.x = round(predictPt.x);
      est_center.y = round(predictPt.y);


      Mat prediction_target = KF_target.predict();
      Point predictPt_target(prediction_target.at<float>(0),prediction_target.at<float>(1));
      est_center_target.x = round(predictPt_target.x);
      est_center_target.y = round(predictPt_target.y);
        
      send_cue = true;
    }
    else
    {
      send_cue = false;
    }
    
    if (meas_flag & meas_counter > 7)
    {
      measurement(0) = z.at<double>(0,0);
      measurement(1) = z.at<double>(1,0); 

      Mat estimated = KF.correct(measurement);
      Point statePt(estimated.at<float>(0),estimated.at<float>(1));
      est_center.x = round(statePt.x);
      est_center.y = round(statePt.y);

      Mat estimated_target = KF_target.correct(measurement);
      Point statePt_target(estimated_target.at<float>(0),estimated_target.at<float>(1));
      est_center_target.x = round(statePt_target.x);
      est_center_target.y = round(statePt_target.y);

      send_cue = true;      
    }

    if (est_center.x > l || est_center.y > w || est_center.x < 0 || est_center.y < 0)
    {
      send_cue = false;
    }

    if (send_cue)
    {
        geometry_msgs::Point32 pt_tmp;
        geometry_msgs::PolygonStamped msg;

        msg.polygon.points.reserve(2); // ensure that there will be space for at least 10 points

        pt_tmp.x = est_center.x;
        pt_tmp.y = est_center.y;
        // msg.polygon.points[0] = pt_tmp;
        msg.polygon.points.push_back(pt_tmp);

        pt_tmp.x = w;
        pt_tmp.y = l;
        // msg.polygon.points[1] = pt_tmp;
        msg.polygon.points.push_back(pt_tmp);

        gimbal_cue.publish(msg);


     // for target vector

        struct ocam_model o; // our ocam_models for the fisheye and catadioptric cameras
        get_ocam_model(&o, "/home/changkoo/catkin_ws/src/perception/src/Insta360_calib_results.txt");

        double point3D[3], point2D[2];                              // the image point in pixel coordinates  
        point2D[0] = est_center_target.x;
        point2D[1] = est_center_target.y;
        cam2world(point3D, point2D, &o); 

        vector<Point2f> p_point;
        vector<Point2f> out_point;

        p_point.push_back(Point(est_center_target.x,est_center_target.y));

        geometry_msgs::Twist i;
        i.linear.x=point3D[0];
        i.linear.y=point3D[1];
        i.linear.z=point3D[2];
        target_vector.publish(i);

        cout<<point3D[0]<<" "<<point3D[1]<<" "<<point3D[2]<<endl;
    }

    circle(drawing, est_center, 15, Scalar(0, 0, 0), 8, 8, 0);
    circle(drawing, est_center_target, 15, Scalar(0, 0, 0), 8, 8, 0);
 

    //Update Image Text----------------------------------------------------------------------------------------------------------------------------------
    stringstream s;

    s.str("");
    s << "Gimbal cue";
    putText(drawing, s.str(), Point(est_center.x+18, est_center.y-18), FONT_HERSHEY_SIMPLEX, 1., Scalar(100, 0, 255), 2);
    putText(drawing, "Target", Point(est_center_target.x+18, est_center_target.y+36), FONT_HERSHEY_SIMPLEX, 1., Scalar(100, 0, 255), 2);
    

    s.str("");
    // s  << "Range = " << range << "  " << "Meas Flag = " << meas_flag << "  " << "Meas Count = " << meas_counter;
    s  << "RngActual = " << range << "  " << "FPS = " << 1/dt << "  Send cue = " << send_cue;
    putText(drawing, s.str(), Point(10, 50), FONT_HERSHEY_SIMPLEX, 1.25, Scalar(100, 0, 255), 4);

    // s.str("");
    // s  << "No meas count = " << no_meas_counter << "  " << "Send cue = " << send_cue;
    // putText(drawing, s.str(), Point(10, 100), FONT_HERSHEY_SIMPLEX, 1.25, Scalar(100, 0, 255), 4);        

    // sprintf(charry, "Range = %f", range);
    // // putText(frame, charry, Point(50, 100), FONT_HERSHEY_SIMPLEX, 1, Scalar(256, 256, 256), 2);
    // putText(fgMaskMOG2_open, charry, Point(50, 100), FONT_HERSHEY_SIMPLEX, 1, Scalar(256, 256, 256), 2);
    // putText(sobel_img, charry, Point(50, 100), FONT_HERSHEY_SIMPLEX, 1, Scalar(256, 256, 256), 2);


    // Update GUI Windows---------------------------------------------------------------------------------------------------------------------------------
    // imshow("Originial Image", frame_cropped);
    // imshow("Gaussian Smoothing", frame);
    // imshow("Gaussian Smoothing Opt Flow", frame_p);
    // imshow("Gaussian Smoothing Opt Flow Old ", img_old);
    
    
    // vector<Mat> hsv;
    // split(frame_hsv,hsv);
    // imshow("Gaussian Smoothing H", hsv[0]);
    // imshow("Gaussian Smoothing S", hsv[1]);
    // imshow("Gaussian Smoothing V", hsv[2]);
    
    
    // imshow("FG Mask MOG 2", fgMaskMOG2);
    // imshow("FG Mask MOG 2 open", fgMaskMOG2_open);
    // imshow("Bin Out Diff", bin_out_diff);
    // imshow("Sobel", sobel_img);
    // imshow("And Image", and_img);
    // imshow("flow", mag);
    // imshow("And Img2", and_img2);
    imshow("Contours", drawing);
    // imshow("Canny", canny_edge);
    
 
    waitKey(3);

    // Output modified video stream
    // image_pub_.publish(cv_ptr->toImageMsg());
    
    // &drawing -> cv_ptr;
    cv_bridge::CvImage out_msg;
    out_msg.header   = cv_ptr->header; // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1; // Or whatever/
    
    
    out_msg.image = drawing; // Your cv::Mat

    image_pub_.publish(out_msg.toImageMsg());

    // image_pub_.publish(&drawing->toImageMsg());
    
  }

  void rotation321(double th1, double th2, double th3, Mat &resultmatrix){

	double pi=3.141592;

	//th1=th1*pi/180;
	//th2=th2*pi/180;
	//th3=th3*pi/180;

  double m00=cos(th2)*cos(th3);
  double m01=-cos(th2)*sin(th3);
  double m02=sin(th2);
  double m10=sin(th1)*sin(th2)*cos(th3)+cos(th1)*sin(th3);
  double m11=-sin(th1)*sin(th2)*sin(th3)+cos(th1)*cos(th3);
  double m12=-sin(th1)*cos(th2);
  double m20=-cos(th1)*sin(th2)*cos(th3)+sin(th1)*sin(th3);
  double m21=cos(th1)*sin(th2)*sin(th3)+sin(th1)*cos(th3);
  double m22=cos(th1)*cos(th2);



   resultmatrix = (Mat_<double>(3,3) << m00,m01,m02,m10,m11,m12,m20,m21,m22);


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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_converter");
  // cout << getBuildInformation;

  // intialization of KF...
  KF.transitionMatrix = (Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
  KF_target.transitionMatrix = (Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);

  measurement.setTo(Scalar(0));
  ImageConverter ic;

  ros::spin();

  return 0;
}
