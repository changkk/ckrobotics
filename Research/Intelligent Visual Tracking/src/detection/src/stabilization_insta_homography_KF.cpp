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
#include <opencv2/core/ocl.hpp>
#include <opencv2/opencv.hpp>


static const std::string OPENCV_WINDOW = "Image window";
using namespace cv;
using std::vector;
using namespace std;

double PI = 3.141592;


Mat K = (Mat_<double>(3, 3) << 445.17984003885283, 0.0, 721.2172155119429, 0.0, 445.8816513187479, 721.9473150432297, 0.0, 0.0, 1.0);
Mat D = (Mat_<double>(1, 4) << -0.017496068223756347, -0.003243014339251435, 0.000839223410670477, -0.0004219234265229322);
Mat H = Mat::eye(3,3,CV_32FC1);

const double INLIER_THRESHOLD = 1.0; // pixel distance
Mat preorig20_orig;

int opencv_inliers;


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


    Mat eye = (Mat_<double>(2,2) << 1, 0, 0, 1);
    int frame_number=1;
    Mat ST_Z;
    Point maxpoint, maxpoint_prev;

int maximum_length =200;
double minimum_length =8;
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

int first=0;

int first_detection=0;
int first_maxpoint=0;

    int detection=0;
    int first_double=0;
    double v_x, v_y;

//Kalman filter

KalmanFilter KF(4,2,0);
Mat_<float> measurement(2,1); 


Point2i est_center;
double g_x, g_x_prev;
double g_y, g_y_prev;

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
                //opencv_inliers = accumulate(status_homography.begin(), status_homography.end(), 0);
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

private:
	double x, y, z;

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


    target_vector_pub = nh_.advertise<geometry_msgs::Twist>("/image_frame_point_fisheye", 10);
    p_pub = nh_.advertise<geometry_msgs::Twist>("/rbe_target_point", 10);
    s_pub = nh_.advertise<geometry_msgs::Twist>("/observation_global_point", 10);
    i_pub = nh_.advertise<geometry_msgs::Twist>("/image_frame_point_fisheye", 10);


    //cv::namedWindow(OPENCV_WINDOW);
    x = 0;
    y = 0;
    z = 0;
  }

  ~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
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
    cv::Rect myROI;
    bool fisheye_front;
    nh_.getParam("/stabilization/fisheye_front",fisheye_front);

    if(fisheye_front)
    {
        myROI = cv::Rect(1504, 0, 1504, 1504);
    }
    else
    {
        myROI = cv::Rect(0, 0, 1504, 1504);
    }
    
    Mat orig2(src,myROI);


    resize(orig2, orig2, Size(1440,1440), 0);
    Mat orig;
    //Mat orig = latitudeCorrection(orig2, Point2i(720,720), 720, 3, 2);
    fisheye::undistortImage(orig2, orig, K, D, K, Size(orig.cols, orig.rows));



    Tracker tracker;
    flip(orig,orig,0);
    tracker.processImage(orig);



    Mat dst;    

    if(first==1)
    {
                Mat invH = H.inv(DECOMP_SVD);


            warpPerspective(orig, dst, invH, dst.size());

            //imshow("IMU_based_stabilization",dst);
            Mat rotation_H, translation_H, normals;
            std::vector<cv::Mat> Rs, Ts;
            decomposeHomographyMat(invH,K,Rs,Ts, cv::noArray());
            
            /*
            for (Mat R_ : Rs) {

                Vec3d eulerAngles_H;
                Mat invR_=R_.inv(DECOMP_SVD);
                getEulerAngles(invR_,eulerAngles_H); // Output is in degrees not in radians!!!!!! and these angles in 3-2-1 direction and add (-) for each angles.
                //std::cout <<"EIS: "<<-eulerAngles_H << std::endl << std::endl;
                //std::cout <<"EIS: "<<eulerAngles_H[0]<<" "<<-eulerAngles_H[2]<<" "<<eulerAngles_H[1]<< std::endl;
            }
            */

		cvtColor(orig,orig_gray,CV_BGR2GRAY);
        cvtColor(dst,gray2,CV_BGR2GRAY);
		dst.copyTo(preorig20_orig);


        



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

            

			//cv::Mat roi(mask, cv::Rect(200,300,orig.cols-500,orig.rows-500));
			//roi = cv::Scalar(255, 255, 255);
            	
            Mat mask_show;
            resize(mask,mask_show,Size(800,800));
            //imshow("mask",mask_show);

            //if(trackedFeatures2.size() < 200) {
            //goodFeaturesToTrack(gray2,trackedFeatures2,250,0.1,5,mask);
            //}

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
				circle(preorig20_orig, Point(trackedFeatures2.at(i).x,trackedFeatures2.at(i).y), 2, Scalar(0, 0, 255), -1); //red(past)
				circle(preorig20_orig,  Point(corners2.at(i).x,corners2.at(i).y), 2, Scalar(255, 0, 0), -1); //blue(current)
				line(preorig20_orig, Point(trackedFeatures2.at(i).x,trackedFeatures2.at(i).y), Point(corners2.at(i).x,corners2.at(i).y), Scalar(0,255,0), 1);
                
				double length=sqrt((trackedFeatures2.at(i).x-corners2.at(i).x)*(trackedFeatures2.at(i).x-corners2.at(i).x)+(trackedFeatures2.at(i).y-corners2.at(i).y)*(trackedFeatures2.at(i).y-corners2.at(i).y));

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
			    //circle(preorig20_orig,  maxpoint, 15, Scalar(0, 0, 255), 3); // draw
                g_x=maxpoint.x-orig.cols;
                g_y=-maxpoint.y+orig.rows;
                cout<<maxpoint<<endl;
                            cout<<detection<<endl;


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
                
                if(g_distance<1000 && maxpoint.x<1200 && maxpoint.y<700)
                {
					v_x=(g_x-g_x_prev)/frame_number;
					v_y=(g_y-g_y_prev)/frame_number;
                    //cout<<maxpoint<<" "<<maxpoint_prev<<" "<<T_U<<endl;        
					maxpoint_prev=maxpoint;
                    frame_number=1;
                    detection=1;    

                }

				if(first_maxpoint==0)
				{
					maxpoint_prev=maxpoint;
					first_maxpoint=1;						
				}


            }

            trackedFeatures2.clear();
            //for (int i = 0; i < status2.size(); ++i) {
            //    if(status[i]) {
            //        trackedFeatures2.push_back(corners2[i]); // save corners of prevGray and go for next image
            //    }
            //}

            cout<<detection<<endl;
        }

        if(first_detection==1)
        {
            rbe(orig);
            //cv::circle(orig2,Point(distorted_points[0].x,distorted_points[0].y),20,Scalar(0,255,0),3);
        }


        Mat double_show;
        resize(preorig20_orig,double_show,Size(800,800));
        resize(orig2,orig_show,Size(800,800));
        flip(orig_show,orig_show,0);
        
        sensor_msgs::ImagePtr results_image;
        results_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", double_show).toImageMsg();
        image_pub_.publish(results_image);
        //imshow("double2",double_show);
        //imshow("Original",orig_show);

    }
		    //printf("OpenCV Inliers: %d\n", opencv_inliers);

					
        out:;

        gray2.copyTo(prevGray2);
        orig_gray.copyTo(prevGray3);

        waitKey(30);

}


  void rbe (Mat &field){

	double p_x, p_y;
	
    if(detection==0)
    {
	    //Predict Only
        Mat prediction = KF.predict();
        Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
        est_center.x = round(predictPt.x);
        est_center.y = round(predictPt.y);
    }

    if(detection==1)
    {
        Mat prediction = KF.predict();
        Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
        est_center.x = round(predictPt.x);
        est_center.y = round(predictPt.y);

        measurement(0) = maxpoint.x;
        measurement(1) = maxpoint.y; 

        Mat estimated = KF.correct(measurement);
        Point statePt(estimated.at<float>(0),estimated.at<float>(1));
        est_center.x = round(statePt.x);
        est_center.y = round(statePt.y);
    }


	cv::circle(preorig20_orig,Point(est_center.x,est_center.y),20,Scalar(0,255,0),3);



    undistorted_points.push_back(Point(est_center.x,est_center.y));

    Vec3d srcv = Vec3d(undistorted_points[0].x, undistorted_points[0].y, 1.0); //Creating a vector in homogeneous coords
    Mat K_inv = K.inv();
    Mat dstv = K_inv*Mat(srcv); //Doing martix by vector multiplication
    undistorted_points[0].x = dstv.at<double>(0,0); //Extracting resulting normalised x coord
    undistorted_points[0].y = dstv.at<double>(0,1); //Extracting resulting normalised y coord
    cv::fisheye::distortPoints(undistorted_points,distorted_points,K,D);
    undistorted_points.clear();

     // for target vector

    struct ocam_model o; // our ocam_models for the fisheye and catadioptric cameras
    get_ocam_model(&o, "/home/nvidia/visual_tracker_2.0/src/detection/src/Insta360_calib_results.txt");

    double point3D[3], point2D[2];                              // the image point in pixel coordinates  
    point2D[0] = distorted_points[0].x;
    point2D[1] = distorted_points[0].y;
    cam2world(point3D, point2D, &o); 

    vector<Point2f> p_point;
    vector<Point2f> out_point;

    cout<<point3D[0]<<" "<<point3D[1]<<" "<<point3D[2]<<endl;

    geometry_msgs::Twist i;
    i.linear.x=point3D[0];
    i.linear.y=point3D[1];
    i.linear.z=point3D[2];
    target_vector_pub.publish(i);

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
  ros::init(argc, argv, "image_converter_fisheye");
  ImageConverter ic;

	// Kalman Filter
    KF.transitionMatrix = (Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
    KF.statePre.at<float>(0) = 100;
    KF.statePre.at<float>(1) = 100;
    KF.statePre.at<float>(2) = 1;
    KF.statePre.at<float>(3) = 1;  
    // Set KF noise params
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(1));
    setIdentity(KF.measurementNoiseCov, Scalar::all(0.001));
    setIdentity(KF.errorCovPost, Scalar::all(.005));

  ros::spin();
  return 0;
}
