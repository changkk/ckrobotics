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
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"
#include <opencv2/core/core.hpp>
#include <sensor_msgs/Imu.h>
#include <opencv2/stitching.hpp>


static const std::string OPENCV_WINDOW = "Image window";
using namespace cv;
using std::vector;
using namespace std;


double PI = 3.141592;
int H_MIN = 0;
int H_MAX = 10;
int S_MIN = 0;
int S_MAX = 10;
int V_MIN = 200;
int V_MAX = 255;
	Mat preorig, preorig2, preorig3, preorig4, preorig5, preorig6, preorig7, preorig8, preorig9, preorig10;
	Mat preorig11, preorig12, preorig13, preorig14, preorig15, preorig16, preorig17, preorig18, preorig19, preorig20;
	Mat preorig21, preorig22, preorig23, preorig24, preorig25, preorig26, preorig27, preorig28, preorig29, preorig30;
	Mat preorig31, preorig32, preorig33, preorig34, preorig35, preorig36, preorig37, preorig38, preorig39, preorig40;
	Mat preorig41, preorig42, preorig43, preorig44, preorig45, preorig46, preorig47, preorig48, preorig49, preorig50;
	Mat preorig51, preorig52, preorig53, preorig54, preorig55, preorig56, preorig57, preorig58, preorig59, preorig60;
    Mat first_image;
    Mat preorig20_orig;


Mat K = (Mat_<double>(3, 3) << 445.17984003885283, 0.0, 721.2172155119429, 0.0, 445.8816513187479, 721.9473150432297, 0.0, 0.0, 1.0);
Mat D = (Mat_<double>(1, 4) << -0.017496068223756347, -0.003243014339251435, 0.000839223410670477, -0.0004219234265229322);
Mat H = Mat::eye(3,3,CV_32FC1);

const double INLIER_THRESHOLD = 1.0; // pixel distance
    timeval start_time, t1, t2;
    Mat img1, img2;
    vector<KeyPoint> kp1, kp2;
    Mat grey1, grey2;

    int opencv_inliers;

double roll,pitch,yaw;
double roll_first, pitch_first, yaw_first;
double angle_first=0;
Mat relation_matrix= Mat::eye(3,3,CV_32FC1);

vector<Point2f> color;

        vector<Point2f> trackedFeatures;
        Mat prevGray;
        Mat gray; 
        vector<Point2f> corners;
        vector<uchar> status; vector<float> errors;
        bool            freshStart=true;
        Mat_<float>     rigidTransform=Mat::eye(3,3,CV_32FC1);


        Mat prevGray2,prevGray3,prevGray4,prevGray5,prevGray6,prevGray7,prevGray8;
    	Mat gray2, orig_gray; 
        vector<uchar> status2; vector<float> errors2;



    Mat T_mxkmGkm = (Mat_<double>(2,1) << 0, 0);
    Mat T_SxkmGkm = (Mat_<double>(2,2) << 5, 0, 0, 5);

	Mat T_A = (Mat_<double>(2,2) << 1, 0, 0, 1);
	Mat T_B = (Mat_<double>(2,2) << 1, 0, 0, 1);
	Mat T_C = (Mat_<double>(2,2) << 1, 0, 0, 1);
	Mat T_A_t = T_A.t();
	Mat T_B_t = T_B.t();
	Mat T_C_t = T_C.t();
	Mat T_U = (Mat_<double>(2,1) << 1, 1);

	Mat T_Swkm = (Mat_<double>(2,2) << 1, 0, 0, 1);
    Mat T_Svk = (Mat_<double>(2,2) << 0, 0, 0, 0);

    Mat T_mxkGkm;
    Mat T_SxkGkm;
    Mat T_mxkGk;
    Mat T_SxkGk;
    Mat T_Kk;

    Mat eye = (Mat_<double>(2,2) << 1, 0, 0, 1);
    int frame_number=1;
    Mat ST_Z;
    Point maxpoint, maxpoint_prev;

const int MAX_FEATURES = 500;
const float GOOD_MATCH_PERCENT = 0.5f;

int maximum_length =150;
double minimum_length =50;
int find_number = 200;
int target_determination = 9;

double length;
double pi=3.141592;
double focal_length = 445.17984003885283;
double ccd_x=2.3*0.001;
double ccd_y=1.6*0.001;

Point prepoint1,prepoint2,prepoint3,prepoint4,prepoint5,prepoint6,prepoint7;
int minimum_distance = 10;
double average;

const int FRAME_WIDTH = 1440;
const int FRAME_HEIGHT = 1440;
const int MAX_NUM_OBJECTS=50;
const int MIN_OBJECT_AREA = 10*10;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
int first=0;

int first_detection=0;
int first_maxpoint=0;

    int detection=0;
    int first_double=0;
    double v_x, v_y;


class Tracker {

public:

    //Tracker():freshStart(true) {
      //  rigidTransform = Mat::eye(3,3,CV_32FC1); //affine 2x3 in a 3x3 matrix
    //}

    void processImage(Mat& img) {
        cvtColor(img,gray,CV_BGR2GRAY);
    
        if(freshStart==true){
            H = Mat::eye(3,3,CV_32FC1); //affine 2x3 in a 3x3 matrix
        }

        if(trackedFeatures.size() < 200) {
            goodFeaturesToTrack(gray,corners,300,0.01,10);
            //cout << "found " << corners.size() << " features\n";
            for (int i = 0; i < corners.size(); ++i) {
                trackedFeatures.push_back(corners[i]);
            }
        }

        if(!prevGray.empty()) {
            calcOpticalFlowPyrLK(prevGray,gray,trackedFeatures,corners,status,errors,Size(5,5));

            if(first==0)
            {
                first=1;
            }

            if(countNonZero(status) < status.size() * 0.8) {
                cout << "cataclysmic error \n";
                angle_first=0;
                H = Mat::eye(3,3,CV_32FC1);
                trackedFeatures.clear();
                prevGray.release();
                freshStart = true;
                return;
            } else
                freshStart = false;
                
            vector<uchar> status_homography;
               
                //cout<<freshStart<<endl;
            if(freshStart==false)
            {
                Mat_<float> new_H = findHomography(trackedFeatures, corners, status_homography, CV_RANSAC, INLIER_THRESHOLD);
                opencv_inliers = accumulate(status_homography.begin(), status_homography.end(), 0);
                //Mat_<float> nrt33 = Mat_<float>::eye(3,3);
                //new_H.copyTo(nrt33.rowRange(0,3));
                //H *= nrt33;
                H = new_H * H;
            }

            //Mat_<float> newRigidTransform = estimateRigidTransform(trackedFeatures,corners,false);




            trackedFeatures.clear();
            for (int i = 0; i < status.size(); ++i) {
                if(status[i]) {
                    trackedFeatures.push_back(corners[i]);
                }
            }
        }

        gray.copyTo(prevGray);
    }
};




class ImageConverter
{

public: 

void chatterCallback(const geometry_msgs::PoseStamped& msg)	
{
        	geometry_msgs::Point coord = msg.pose.position;
		x = coord.x;
		y = coord.y;
		z = coord.z;
	
}

void imuCallback(const sensor_msgs::Imu& msg2)	
{
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

    if(angle_first==0)
    {
        roll_first=roll;
        pitch_first=pitch;
        yaw_first=yaw;
        angle_first=1;
    }
    
    double pitch_diff=pitch-pitch_first;
    double roll_diff=roll-roll_first;
    double yaw_diff=yaw-yaw_first;

    cout<<"IMU: "<<pitch_diff*180/CV_PI<<" "<<roll_diff*180/CV_PI<<" "<<yaw_diff*180/CV_PI<<endl;

    rotation321(-pitch,yaw,roll, relation_matrix);

    
}

private:
	double x, y, z;
	double g_x, g_x_prev;
	double g_y, g_y_prev;
    double g_distance;
	Mat gray4;
        Mat frame,orig_warped;

	double detected_time=0;
	double fps_check=0;
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber pose_sub;
  ros::Publisher p_pub;
  ros::Publisher s_pub;
  ros::Publisher i_pub;
  ros::Subscriber imu_sub;


public:
  ImageConverter()
  : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/webcam/image_raw", 1,
    &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/stitched_fisheye", 1);

    pose_sub = nh_.subscribe("/hexacopter/mavros/local_position/pose", 1000, &ImageConverter::chatterCallback, this);
    imu_sub = nh_.subscribe("/mavros/imu/data", 1000, &ImageConverter::imuCallback, this);

    p_pub = nh_.advertise<geometry_msgs::Twist>("/rbe_target_point", 10);
    s_pub = nh_.advertise<geometry_msgs::Twist>("/observation_global_point", 10);
    i_pub = nh_.advertise<geometry_msgs::Twist>("/image_frame_point_fisheye", 10);

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

    frame_number=frame_number+1;

    detection=0;
    cv::Mat src;
    src = cv_ptr->image;
    vector<Point2f> trackedFeatures3, trackedFeatures2;
    vector<Point2f> corners2;
    double maxlen=0;
    int maxpoint_location;

        cv::Rect myROI(736, 0, 736, 736);
        cv::Mat orig(src,myROI);

        cv::Rect myROI2(0, 0, 736, 736);
        cv::Mat orig2(src,myROI2);
        vector<Mat> images_array;
        Mat panorama;

        resize(orig, orig, Size(1440,1440), 0);
        resize(orig2, orig2, Size(1440,1440), 0);

        Mat orig_show;
        Mat resultImage = latitudeCorrection(orig, Point2i(720,720), 720, 3, 2);
        Mat resultImage2 = latitudeCorrection(orig2, Point2i(720,720), 720, 3, 2);


        cv::Stitcher s=Stitcher::createDefault(false);
        //cv::Stitcher stitcher = cv::Stitcher::createDefault(true);
        cv::Rect myROI3(0, 0, 648, 1296);
        cv::Rect myROI4(648, 0, 648, 1296);
        cv::Mat stitch1(resultImage2,myROI4);
        cv::Mat stitch2(resultImage2,myROI3);
        images_array.push_back(stitch1);
        images_array.push_back(resultImage);
        images_array.push_back(stitch2);


        Stitcher::Status stitching_status = s.stitch(images_array, panorama);
        Mat tmp;
        //cout<<panorama.size()<<"asdfasdf"<<images_array[1].R<<endl;

        if(stitching_status==0)
        {
    	    resize(panorama, tmp, panorama.size()/10);
            imshow("stitched",panorama);
        }

		sensor_msgs::ImagePtr msg3;
		msg3 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", panorama).toImageMsg();
      	image_pub_.publish(msg3);
        
        //imshow("adsfadsfad2",resultImage2);

        // //imshow("corrected",resultImage);
        //     resize(orig,orig_show,Size(800,800));
		// fisheye::undistortImage(orig, orig, K, D, K, Size(orig.cols, orig.rows));

        //     Tracker tracker;



        

        waitKey(30);

}



  void rbe (Mat &field){

	double p_x, p_y;
	
    if(detection==0)
    {
	    //Predict Only
	    T_mxkGkm = T_A*T_mxkmGkm + T_B*T_U;
	    T_SxkGkm = T_A*T_SxkmGkm*T_A_t + T_Swkm;
	    T_mxkGk = T_mxkGkm;
	    T_SxkGk = T_SxkGkm;

		p_x=T_mxkGk.at<double>(0,0) + field.cols;
		p_y=-T_mxkGk.at<double>(1,0) + field.rows;
		putText(preorig20_orig,"Predicting",Point(p_x+10,p_y),2,1,Scalar(0,0,0),2);

    }

    if(detection==1)
    {

	    Mat T_X = (Mat_<double>(2,1) << g_x,g_y);

        //Predict
	    T_mxkGkm = T_A*T_mxkmGkm + T_B*T_U;
	    T_SxkGkm = T_A*T_SxkmGkm*T_A_t + T_Swkm;

	    T_X = T_X + T_B*T_U;
	    ST_Z = T_X;
	
	    //Correction
	    T_Kk = T_SxkGkm*T_C_t/(T_C*T_SxkGkm*T_C_t + T_Svk);
	    T_mxkGk = T_mxkGkm + T_Kk*(ST_Z - T_C*T_mxkGkm);
	    T_SxkGk = (eye - T_Kk*T_C)*T_SxkGkm;
        //cout<<g_distance<<endl;

		p_x=T_mxkGk.at<double>(0,0) + field.cols;
		p_y=-T_mxkGk.at<double>(1,0) + field.rows;

        putText(preorig20_orig,"Correcting",Point(p_x+10,p_y),2,1,Scalar(0,0,0),2);

    }

    //cout<<T_U<<endl;

	cv::circle(preorig20_orig,Point(p_x,p_y),10,Scalar(0,255,0),-1);

	T_mxkmGkm = T_mxkGk;
	T_SxkmGkm = T_SxkGk;



  }

Mat latitudeCorrection(Mat imgOrg, Point2i center, int radius, double camerFieldAngle, int nmber)
{
	if (!(camerFieldAngle > 0 && camerFieldAngle <= PI))
	{
		cout << "The parameter \"camerFieldAngle\" must be in the interval (0,PI]." << endl;
		return Mat();
	}
	double rateOfWindow = 0.9;
	int width = imgOrg.size().width*rateOfWindow;
	int height = width;
	Size imgSize(width, height);

	Mat retImg(imgSize, CV_8UC3, Scalar(0, 0, 0));

	double dx = camerFieldAngle / imgSize.width;
	double dy = dx;

	//coordinate for latitude map
	double latitude;
	double longitude;

	//unity sphere coordinate 
	double x, y, z, r;

	//parameter cooradinate of sphere coordinate
	double Theta_sphere;
	double Phi_sphere;

	//polar cooradinate for fish-eye Image
	double p;
	double theta;

	//cartesian coordinate 
	double x_cart, y_cart;

	//Image cooradinate of imgOrg
	int u, v;

	//Image cooradinate of imgRet
	int u_latitude, v_latitude;

	//offset of imgRet Origin
	double longitude_offset, latitude_offset;
	longitude_offset = (PI - camerFieldAngle) / 2;
	latitude_offset = (PI - camerFieldAngle) / 2;

	cv::Mat_<Vec3b> _retImg = retImg;
	cv::Mat_<Vec3b> _imgOrg = imgOrg;
//according to the correct type to do the calibration


		for (int j = 0; j < imgSize.height; j++)
		{

			latitude = latitude_offset + j*dy;
			for (int i = 0; i < imgSize.width; i++)
			{

				longitude = longitude_offset + i*dx;
				//Convert from latitude cooradinate to the sphere cooradinate
				x = -sin(latitude)*cos(longitude);
				y = cos(latitude);
				z = sin(latitude)*sin(longitude);

				//Convert from sphere cooradinate to the parameter sphere cooradinate
				Theta_sphere = acos(z);
				Phi_sphere = cvFastArctan(y, x);//return value in Angle
				Phi_sphere = Phi_sphere*PI / 180;//Convert from Angle to Radian


				//Convert from parameter sphere cooradinate to fish-eye polar cooradinate
				p = sin(Theta_sphere);
				theta = Phi_sphere;

				//Convert from fish-eye polar cooradinate to cartesian cooradinate
				x_cart = p*cos(theta);
				y_cart = p*sin(theta);

				//double R = radius / sin(camerFieldAngle / 2);
				double R = radius;
				//Convert from cartesian cooradinate to image cooradinate
				u = x_cart*R + center.x;
				v = -y_cart*R + center.y;

				//if (pow(u - center.x, 2) + pow(v - center.y, 2) > pow(radius, 2))
				//{
				//	_imgOrg(v, u)[0] = 255;
				//	_imgOrg(v, u)[1] = 255;
				//	_imgOrg(v, u)[2] = 255;
				//	continue;
				//}
				_retImg.at<Vec3b>(j, i) = _imgOrg.at<Vec3b>(v, u);
			}
		}



	//imwrite("C:\\Users\\Joker\\Desktop\\ret4.jpg", retImg);
	//imshow("org", _imgOrg);
	//imshow("ret", _retImg);
	//cv::waitKey();
#ifdef _DEBUG_
	cv::namedWindow("Corrected Image", CV_WINDOW_AUTOSIZE);
	imshow("Corrected Image", retImg);
	cv::waitKey();
#endif
	return retImg;
}


void rotation321(double th1, double th2, double th3, Mat &resultmatrix){

	double pi=3.141592;

	th1=th1*pi/180;
	th2=th2*pi/180;
	th3=th3*pi/180;

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

void rotation123(double th1, double th2, double th3, Mat &resultmatrix){

	double pi=3.141592;

	th1=th1*pi/180;
	th2=th2*pi/180;
	th3=th3*pi/180;

  double m00=cos(th3)*cos(th2);
  double m01=-sin(th3)*cos(th1)+cos(th3)*sin(th2)*sin(th1);
  double m02=sin(th3)*sin(th1)+cos(th3)*sin(th2)*cos(th1);
  double m10=sin(th3)*cos(th2);
  double m11=cos(th3)*cos(th1)+sin(th3)*sin(th2)*sin(th1);
  double m12=-cos(th3)*sin(th1)+sin(th3)*sin(th2)*cos(th1);
  double m20=-sin(th2);
  double m21=cos(th2)*sin(th1);
  double m22=cos(th2)*cos(th1);



   resultmatrix = (Mat_<double>(3,3) << m00,m01,m02,m10,m11,m12,m20,m21,m22);



}


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter_fisheye");
  ImageConverter ic;
  ros::spin();
  return 0;
}
