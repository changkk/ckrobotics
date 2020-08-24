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
#include "ocam_functions.h"


static const std::string OPENCV_WINDOW = "Image window";
using namespace cv;
using std::vector;
using namespace std;

Mat first_image;
Mat image_show;
double PI = 3.141592;


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
vector<Point2f> distorted_points, undistorted_points;
vector<Point2f> color;

vector<Point2f> trackedFeatures;
Mat prevGray;
Mat gray; 
vector<Point2f> corners;
vector<uchar> status; vector<float> errors;
bool            freshStart=true;
int image_hold = 0;
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

int maximum_length =80;
double minimum_length =5;
int find_number = 300;
int target_determination = 9;

double length;
double pi=3.141592;
double focal_length = 445.17984003885283;
double ccd_x=2.3*0.001;
double ccd_y=1.6*0.001;
int num_frame = 0;
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


    void processImage(Mat& img) {
        cvtColor(img,gray,CV_BGR2GRAY);
    
        if(freshStart==true){
            H = Mat::eye(3,3,CV_32FC1); //affine 2x3 in a 3x3 matrix
        }

        if(trackedFeatures.size() < 200) {
            goodFeaturesToTrack(gray,corners,300,0.01,10);
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
               
            if(freshStart==false)
            {
                Mat_<float> new_H = findHomography(trackedFeatures, corners, status_homography, CV_RANSAC, INLIER_THRESHOLD);
                opencv_inliers = accumulate(status_homography.begin(), status_homography.end(), 0);
                H = new_H * H;
            }


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

    //cout<<"IMU: "<<pitch_diff*180/CV_PI<<" "<<roll_diff*180/CV_PI<<" "<<yaw_diff*180/CV_PI<<endl;

    rotation321(-pitch,yaw,roll, relation_matrix);

    
}

private:
	double x, y, z;
	double g_x, g_x_prev;
	double g_y, g_y_prev;
    double g_distance;
	Mat gray4;
        Mat frame,orig_warped,tmp;

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
  ros::Publisher target_vector_pub;

public:
  ImageConverter()
  : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed

    image_transport::TransportHints hints("compressed", ros::TransportHints());

    image_sub_ = it_.subscribe("/omnicam/image_raw", 1,
    &ImageConverter::imageCb, this, hints);
    image_pub_ = it_.advertise("/undistort_fisheye", 1);

    pose_sub = nh_.subscribe("/hexacopter/mavros/local_position/pose", 1000, &ImageConverter::chatterCallback, this);
    imu_sub = nh_.subscribe("/mavros/imu/data", 1000, &ImageConverter::imuCallback, this);

    target_vector_pub = nh_.advertise<geometry_msgs::Twist>("/image_frame_point_fisheye", 10);
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
    num_frame = num_frame + 1;


    if (num_frame > 20)
    {
        freshStart=true;
        num_frame = 0;
    }

    if (freshStart == true)
    {
        image_hold = 0;
    }

    image_hold = image_hold + 1;

    detection=0;
    cv::Mat src;
    src = cv_ptr->image;
    vector<Point2f> trackedFeatures3, trackedFeatures2;
    vector<Point2f> corners2;
    double maxlen=0;
    int maxpoint_location;
    Mat orig_show;


    // Crop the image using region of interest
    cv::Rect myROI(0, 1504, 1504, 1504);
    cv::Mat orig2(src,myROI);

    // Resize image
    resize(orig2, orig2, Size(1440,1440), 0);
    Mat orig;

    // Undistort images
    fisheye::undistortImage(orig2, orig, K, D, K, Size(orig.cols, orig.rows));

    // Flip the image
    flip(orig,orig,0);

    // Find corner -> use optical flow -> compute Homography matrix
    Tracker tracker;
    tracker.processImage(orig);

    Mat dst;    

    if(first==1)
    {
        // Homography matrix obtained
        Mat invH = H.inv(DECOMP_SVD);

        // Transform image using homography matrix
        warpPerspective(orig, dst, invH, dst.size());

        Mat rotation_H, translation_H, normals;
        std::vector<cv::Mat> Rs, Ts;

        // Decompose homography mat to estimate the camera pose (roll pitch yaw angle)
        decomposeHomographyMat(invH,K,Rs,Ts, cv::noArray());
        
        for (auto R_ : Rs) {

            Vec3d eulerAngles_H;
            Mat invR_=R_.inv(DECOMP_SVD);
            getEulerAngles(invR_,eulerAngles_H); // Output is in degrees not in radians!!!!!! and these angles in 3-2-1 direction and add (-) for each angles.
        }


        // Make a ROI everytime to exclude the black background after transform the image.
		cvtColor(orig,orig_gray,CV_BGR2GRAY);
        cvtColor(dst,gray2,CV_BGR2GRAY);
		dst.copyTo(image_show);


        ////////////////////// Polygon ROI////////////////////////////////////

            Mat mask(gray2.rows, gray2.cols, CV_8UC1, cv::Scalar(0));
            
            Point P1(300,300);
            Point P2(gray2.rows-100,300);
            Point P3(gray2.rows-100, gray2.cols-300);
            Point P4(100,gray2.cols-300);

            double newp1z = invH.at<float>(2, 0) * P1.x + invH.at<float>(2, 1) * P1.y + invH.at<float>(2, 2);
            int newp1x = (invH.at<float>(0, 0) * P1.x + invH.at<float>(0, 1) * P1.y + invH.at<float>(0, 2)) / newp1z;
            int newp1y = (invH.at<float>(1, 0) * P1.x + invH.at<float>(1, 1) * P1.y + invH.at<float>(1, 2)) / newp1z;            
            Point new_p1(newp1x,newp1y);

            double newp2z = invH.at<float>(2, 0) * P2.x + invH.at<float>(2, 1) * P2.y + invH.at<float>(2, 2);
            int newp2x = (invH.at<float>(0, 0) * P2.x + invH.at<float>(0, 1) * P2.y + invH.at<float>(0, 2)) / newp2z;
            int newp2y = (invH.at<float>(1, 0) * P2.x + invH.at<float>(1, 1) * P2.y + invH.at<float>(1, 2)) / newp2z;            
            Point new_p2(newp2x,newp2y);
            
            double newp3z = invH.at<float>(2, 0) * P3.x + invH.at<float>(2, 1) * P3.y + invH.at<float>(2, 2);
            int newp3x = (invH.at<float>(0, 0) * P3.x + invH.at<float>(0, 1) * P3.y + invH.at<float>(0, 2)) / newp3z;
            int newp3y = (invH.at<float>(1, 0) * P3.x + invH.at<float>(1, 1) * P3.y + invH.at<float>(1, 2)) / newp3z;            
            Point new_p3(newp3x,newp3y);

            double newp4z = invH.at<float>(2, 0) * P4.x + invH.at<float>(2, 1) * P4.y + invH.at<float>(2, 2);
            int newp4x = (invH.at<float>(0, 0) * P4.x + invH.at<float>(0, 1) * P4.y + invH.at<float>(0, 2)) / newp4z;
            int newp4y = (invH.at<float>(1, 0) * P4.x + invH.at<float>(1, 1) * P4.y + invH.at<float>(1, 2)) / newp4z;            
            Point new_p4(newp4x,newp4y);

            vector< vector<Point> >  co_ordinates;
            co_ordinates.push_back(vector<Point>());

            co_ordinates[0].push_back(new_p1);
            co_ordinates[0].push_back(new_p2);
            co_ordinates[0].push_back(new_p3);
            co_ordinates[0].push_back(new_p4);
            drawContours( mask,co_ordinates,0, Scalar(255),CV_FILLED, 8 );

            /////////////////////////////////////////////////////////////////////////////////////

            Mat mask_show;
            resize(mask,mask_show,Size(800,800));


    
        // Use optical flow to find a moving object after the image stabilization

        if(!prevGray2.empty() && image_hold > 2) {
            goodFeaturesToTrack(prevGray2,trackedFeatures2,250,0.1,5,mask);
            calcOpticalFlowPyrLK(prevGray2,gray2,trackedFeatures2,corners2,status2,errors2,Size(10,10));

            if(countNonZero(status) < status.size() * 0.8) {
                //cout << "cataclysmic error \n";
                trackedFeatures2.clear();
                prevGray2.release();
				goto out;
            } else

            for (int i = 0; i < trackedFeatures2.size(); ++i) 
			{
                // Draw circle on the detected moving target
				circle(image_show, Point(trackedFeatures2.at(i).x,trackedFeatures2.at(i).y), 2, Scalar(0, 0, 255), -1); //red(past)
				circle(image_show,  Point(corners2.at(i).x,corners2.at(i).y), 2, Scalar(255, 0, 0), -1); //blue(current)
				line(image_show, Point(trackedFeatures2.at(i).x,trackedFeatures2.at(i).y), Point(corners2.at(i).x,corners2.at(i).y), Scalar(0,255,0), 1);
                
				double length=sqrt((trackedFeatures2.at(i).x-corners2.at(i).x)*(trackedFeatures2.at(i).x-corners2.at(i).x)+(trackedFeatures2.at(i).y-corners2.at(i).y)*(trackedFeatures2.at(i).y-corners2.at(i).y));

                    // Specify threshold for the detection.
					if(length<maximum_length&&length>minimum_length)
                    { // if the length is shorter than my threshold
					    if(maxlen<length)
					    {
                            maxlen=length;
                            maxpoint.x=corners2.at(i).x;
					        maxpoint.y=corners2.at(i).y;
					        maxpoint_location = i;


					    }
	                }

			}
            


            if(maxlen>minimum_length) // if the length chosen is larger than the threshold
		    {
			    circle(image_show,  maxpoint, 15, Scalar(0, 0, 255), 3); // draw
                g_x=maxpoint.x-orig.cols;
                g_y=-maxpoint.y+orig.rows;


                
                if(first_detection==0)
                {
                    first_detection=1;
                    g_distance=0;
                }
                else
                {
                    g_x_prev=maxpoint_prev.x-orig.cols;
                    g_y_prev=-maxpoint_prev.y+orig.rows;
                    g_distance=sqrt((g_x-g_x_prev)*(g_x-g_x_prev)+(g_y-g_y_prev)*(g_y-g_y_prev));               

                }
                
                // For the Kalman filter (velocity)
                if(g_distance<400&&400<maxpoint.x<1200&&400<maxpoint.y<1000)
                {
					v_x=(g_x-g_x_prev)/frame_number;
					v_y=(g_y-g_y_prev)/frame_number;
					T_U = (Mat_<double>(2,1) << v_x, v_y);
                    detection=1;
					maxpoint_prev=maxpoint;
                    frame_number=1;

                }

				if(first_maxpoint==0)
				{
					maxpoint_prev=maxpoint;
					first_maxpoint=1;						
				}


            }


            trackedFeatures2.clear();

        }

        if(first_detection==1)
        {
            rbe(orig);
            cv::circle(orig2,Point(distorted_points[0].x,distorted_points[0].y),20,Scalar(0,255,0),3);
        }


        Mat double_show;
        resize(image_show,double_show,Size(800,800));
        resize(orig2,orig_show,Size(800,800));
        imshow("double",double_show);

    }

					
        out:;

        gray2.copyTo(prevGray2);
        orig_gray.copyTo(prevGray3);



        waitKey(30);

}


// Kalman filter function for keep the detection
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
		//putText(image_show,"Predicting",Point(p_x+10,p_y),2,1,Scalar(0,0,0),2);

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

        //putText(image_show,"Correcting",Point(p_x+10,p_y),2,1,Scalar(0,0,0),2);

    }

    //cout<<T_U<<endl;

	cv::circle(image_show,Point(p_x,p_y),20,Scalar(0,255,0),3);
	T_mxkmGkm = T_mxkGk;
	T_SxkmGkm = T_SxkGk;

    undistorted_points.push_back(Point(p_x,p_y));

    Vec3d srcv = Vec3d(undistorted_points[0].x, undistorted_points[0].y, 1.0); //Creating a vector in homogeneous coords
    Mat K_inv = K.inv();
    Mat dstv = K_inv*Mat(srcv); //Doing martix by vector multiplication
    undistorted_points[0].x = dstv.at<double>(0,0); //Extracting resulting normalised x coord
    undistorted_points[0].y = dstv.at<double>(0,1); //Extracting resulting normalised y coord
    cv::fisheye::distortPoints(undistorted_points,distorted_points,K,D);
    undistorted_points.clear();


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
