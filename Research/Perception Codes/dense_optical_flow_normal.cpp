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

#include <opencv2/core/core.hpp>

static const std::string OPENCV_WINDOW = "Image window";
using namespace cv;
using std::vector;
using namespace std;

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

vector<Point2f> color;
    vector<Point2f> trackedFeatures;


    Mat prevGray;
    Mat gray; 
        vector<Point2f> corners;
            vector<uchar> status; vector<float> errors;

const int MAX_FEATURES = 500;
const float GOOD_MATCH_PERCENT = 0.5f;

int maximum_length =10;
double minimum_length =4;
int find_number = 100;
int target_determination = 9;

double length;
double pi=3.141592;
double focal_length = 17.2*0.001;
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
    bool            freshStart=true;
    Mat_<float>     rigidTransform;


class Tracker {

public:

    //Tracker():freshStart(true) {
      //  rigidTransform = Mat::eye(3,3,CV_32FC1); //affine 2x3 in a 3x3 matrix
    //}

    void processImage(Mat& img) {
        cvtColor(img,gray,CV_BGR2GRAY);
    
        if(freshStart==true){
            rigidTransform = Mat::eye(3,3,CV_32FC1); //affine 2x3 in a 3x3 matrix
        }

        if(trackedFeatures.size() < 200) {
            goodFeaturesToTrack(gray,corners,300,0.01,10);
            cout << "found " << corners.size() << " features\n";
            for (int i = 0; i < corners.size(); ++i) {
                trackedFeatures.push_back(corners[i]);
            }
        }

        if(!prevGray.empty()) {
            calcOpticalFlowPyrLK(prevGray,gray,trackedFeatures,corners,status,errors,Size(5,5));

            if(countNonZero(status) < status.size() * 0.8) {
                cout << "cataclysmic error \n";
                rigidTransform = Mat::eye(3,3,CV_32FC1);
                trackedFeatures.clear();
                prevGray.release();
                freshStart = true;
                return;
            } else
                freshStart = false;
                
                //cout<<freshStart<<endl;
            Mat_<float> newRigidTransform = estimateRigidTransform(trackedFeatures,corners,false);
            Mat_<float> nrt33 = Mat_<float>::eye(3,3);
            newRigidTransform.copyTo(nrt33.rowRange(0,2));
            rigidTransform *= nrt33;

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
	double g_x;
	double g_y;

Mat T_mxkmGkm = (Mat_<double>(2,1) << 0, 0);
Mat T_SxkmGkm = (Mat_<double>(2,2) << 5, 0, 0, 5);
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
	Mat preorig, preorig2, preorig3, preorig4, preorig5, preorig6, preorig7, preorig8, preorig9, preorig10;
	Mat preorig11, preorig12, preorig13, preorig14, preorig15, preorig16, preorig17, preorig18, preorig19, preorig20;
	Mat preorig21, preorig22, preorig23, preorig24, preorig25, preorig26, preorig27, preorig28, preorig29, preorig30;
	Mat preorig31, preorig32, preorig33, preorig34, preorig35, preorig36, preorig37, preorig38, preorig39, preorig40;
	Mat preorig41, preorig42, preorig43, preorig44, preorig45, preorig46, preorig47, preorig48, preorig49, preorig50;
	Mat preorig51, preorig52, preorig53, preorig54, preorig55, preorig56, preorig57, preorig58, preorig59, preorig60;
	Mat K = (Mat_<double>(3, 3) << 445.17984003885283, 0.0, 721.2172155119429, 0.0, 445.8816513187479, 721.9473150432297, 0.0, 0.0, 1.0);
	Mat D = (Mat_<double>(1, 4) << -0.017496068223756347, -0.003243014339251435, 0.000839223410670477, -0.0004219234265229322);

public:
  ImageConverter()
  : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/videofile/image_raw/", 1,
    &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/undistort_fisheye", 1);

    pose_sub = nh_.subscribe("/hexacopter/mavros/local_position/pose", 1000, &ImageConverter::chatterCallback, this);
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


	//fps_check=fps_check+0.195;
	//if(fps_check>20)
	//{
	//	std::cout<<"fps_check: "<<fps_check<<endl;
	//}


    cv::Mat src;

    src = cv_ptr->image;

    Mat frame,orig_warped,tmp;

    Tracker tracker;

	Mat flowUmat, flow, gray, gray2;
	Point premaxpoint1, premaxpoint2, premaxpoint3, premaxpoint4;
		//pyramid

        //cv::Rect myROI(0, 0, 1504, 1504);
        cv::Mat orig(src);
		//Mat orig;
		Mat out;
		//resize(orig, orig, Size(1440,1440), 0);
		//fisheye::undistortImage(orig, orig, K, D, K, Size(orig.cols, orig.rows));


        tracker.processImage(orig);

		Mat orig_orig_warped;
        Mat invTrans = rigidTransform.inv(DECOMP_SVD);
        warpAffine(orig,orig_warped,invTrans.rowRange(0,2),Size());
		
		if (preorig40.empty() == false ) {

	    	vector<Point2f> trackedFeatures2;
			Mat_<float>     rigidTransform2;
        	Mat prevGray2;
    	    Mat gray2; Mat gray3;
			Mat preorig_orig, preorig20_orig;
            Mat orig_copy;
        	vector<Point2f> corners2, corners3, graph;
			 //preorig.copyTo(preorig_orig);
			cvtColor(preorig40,gray2,CV_BGR2GRAY);
			cvtColor(orig,gray3,CV_BGR2GRAY);
			orig_warped.copyTo(orig_copy);

        	calcOpticalFlowFarneback(gray2, gray3, flowUmat, 0.5, 2, 50, 1, 1, 1, 0);
            // first/two number - pyrimid size (classically 0.5), pyramid layer numbers
            // 50 - window size
            // iteration number
            // smoothness - large number -> smoother surface (5 or 7)
            // poly_sigma - 5 -> 1.1 / 7 - 1.5

			Mat flow_channel[2];
			flowUmat.copyTo(flow);


            // By y += 5, x += 5 you can specify the grid 
    	for (int y = 0; y < orig.rows; y += 10) {
     	for (int x = 0; x < orig.cols; x += 10)
     	{
            // get the flow from y, x position * 10 for better visibility
            const Point2f flowatxy = flow.at<Point2f>(y, x) * 10;
            // draw line at flow direction
       		line(orig_copy, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), Scalar(0,255,0));
            // draw initial point
       		circle(orig_copy, Point(x, y), 1, Scalar(0, 0, 0), -1);
   		}
     	}
		

		imshow("double",orig_copy);


 		 preorig37.copyTo(preorig36);
		 preorig38.copyTo(preorig37);
		 preorig39.copyTo(preorig38);
		 preorig40.copyTo(preorig39);
		 orig_warped.copyTo(preorig40);

		//imshow("fisheye",preorig20_orig);
		sensor_msgs::ImagePtr msg;
		msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", preorig20_orig).toImageMsg();
      	image_pub_.publish(msg);
	}
	else {

 		 preorig37.copyTo(preorig36);
		 preorig38.copyTo(preorig37);
		 preorig39.copyTo(preorig38);
		 preorig40.copyTo(preorig39);
		 orig_warped.copyTo(preorig40);

	}





   

	//cv::imshow("src", preorig);    
	//imshow("orig",orig);
	int key1 = waitKey(20);
  		//image_pub_.publish(cv_ptr->toImageMsg());

}






void drawObject(vector<Point2f> color,Mat &frame){

		for(int i=0; i<color.size(); i++){

	cv::circle(frame,Point(color.at(i).x,color.at(i).y),3,Scalar(0,0,255),-1);
	

		}
				std::cout << "found " << color  << " features\n";


}	
void morphOps(Mat &thresh){

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle

	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(2,2));
    //dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(7,7));

	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);


	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);
	


}
void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed){


	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
		color.clear();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
                if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
					x = moment.m10/area;
					y = moment.m01/area;
					objectFound = true;
					refArea = area;
					
					Point point2(x,y);

					color.push_back(point2);


				}else objectFound = false;


			}
			//let user know you found an object
			if(objectFound ==true){
				//putText(cameraFeed,"Tracking Object",Point(0,50),2,1,Scalar(0,255,0),2);
				//draw object location on screen
				drawObject(color,cameraFeed);
				
				}


		}//else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}
	

}




};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter_fisheye");
  ImageConverter ic;
  ros::spin();
  return 0;
}
