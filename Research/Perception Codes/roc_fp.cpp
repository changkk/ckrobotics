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


vector<Point2f> color;

int maximum_length =100;
double minimum_length =5.5;
int find_number = 50;
int target_determination = 9;

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

class Tracker {
    vector<Point2f> trackedFeatures_old;
	vector<Point2f> trackedFeatures;
    Mat             prevGray;
public:
    bool            freshStart;
    Mat_<float>     rigidTransform;

    Tracker():freshStart(true) {
        rigidTransform = Mat::eye(3,3,CV_32FC1); //affine 2x3 in a 3x3 matrix
    }

    void processImage(Mat& img) {
        Mat gray; cvtColor(img,gray,CV_BGR2GRAY);
        vector<Point2f> corners;


		        if(trackedFeatures_old.size() < 450) {

								if(!prevGray.empty()) {
									goodFeaturesToTrack(prevGray,corners,500,0.01,10);
									//cout << "found " << corners.size() << " features\n";
									for (int i = 0; i < corners.size(); ++i) {
										trackedFeatures_old.push_back(corners[i]);
									}
								}

            }

        


        if(!prevGray.empty()) {
            vector<uchar> status; vector<float> errors;
            calcOpticalFlowPyrLK(prevGray,gray,trackedFeatures_old,corners,status,errors,Size(10,10));
				
			//for (int i = 0; i < status.size(); ++i) {
				//cout << "found " << trackedFeatures_old.at(i) << corners.at(i) << " features\n";

	//		}


            if(countNonZero(status) < status.size() * 0.8) {
                cout << "cataclysmic error \n";
                rigidTransform = Mat::eye(3,3,CV_32FC1);
                trackedFeatures_old.clear();
                prevGray.release();
                freshStart = true;
                return;
            } else
                freshStart = false;


            Mat_<float> newRigidTransform = estimateRigidTransform(trackedFeatures_old,corners,false);
            Mat_<float> nrt33 = Mat_<float>::eye(3,3);
            newRigidTransform.copyTo(nrt33.rowRange(0,2));
            rigidTransform *= nrt33;

            trackedFeatures_old.clear();
            for (int i = 0; i < status.size(); ++i) {
                if(status[i]) {
                    trackedFeatures_old.push_back(corners[i]);
					
                }
            }
        }

        //for (int i = 0; i < trackedFeatures_old.size(); ++i) {
         //   circle(img,trackedFeatures_old[i],3,Scalar(0,0,255),CV_FILLED);
        //}

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
    image_sub_ = it_.subscribe("/omnicam/image_raw", 1,
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


    bool trackObjects = true;
    bool useMorphOps = true;
	average=100;

	Mat HSV;
	Mat threshold;
	int x=0, y=0;


    cv::Mat src;

    src = cv_ptr->image;

    Mat frame,orig_warped,tmp;

    Tracker tracker;

	Mat flowUmat, flow;
	Point premaxpoint1, premaxpoint2, premaxpoint3, premaxpoint4;
		//pyramid



        cv::Rect myROI(0, 0, 1504, 1504);
        cv::Mat orig(src,myROI);
		//Mat orig;
		Mat out;
		resize(orig, orig, Size(1440,1440), 0);
		fisheye::undistortImage(orig, orig, K, D, K, Size(orig.cols, orig.rows));
        tracker.processImage(orig);

		Mat orig_orig_warped;
        Mat invTrans = tracker.rigidTransform.inv(DECOMP_SVD);



		if (preorig40.empty() == false ) {

	    	vector<Point2f> trackedFeatures2;
			Mat_<float>     rigidTransform2;
        	Mat prevGray2;
    	    Mat gray2; Mat gray3; 
			Mat preorig_orig, preorig20_orig;
        	vector<Point2f> corners2, corners3, graph;
			 //preorig.copyTo(preorig_orig);
			 orig.copyTo(preorig20_orig);
			cvtColor(preorig40,gray2,CV_BGR2GRAY);
			cvtColor(orig,gray3,CV_BGR2GRAY);

			//color detection

			//cvtColor(preorig40,HSV,COLOR_BGR2HSV);
			//inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);
			//if(useMorphOps)
			//	morphOps(threshold);
			//if(trackObjects)
			//	trackFilteredObject(x,y,threshold,preorig20_orig);
				//imshow(windowName,cameraFeed);


        	if(trackedFeatures2.size() < 5) 
			{
			

				cv::Mat mask = cv::Mat::zeros(gray3.size(), CV_8UC1);  
				cv::Mat roi(mask, cv::Rect(400,0,1039,720));
				roi = cv::Scalar(255, 255, 255);	
            
				goodFeaturesToTrack(gray2,corners2,find_number,0.0001,10,mask);

            	for (int i = 0; i < corners2.size(); ++i) 
				{
                trackedFeatures2.push_back(corners2[i]);
            	}
        	}

        	vector<uchar> status2; vector<float> errors2;
        	calcOpticalFlowPyrLK(gray2,gray3,trackedFeatures2,corners2,status2,errors2,Size(10,10));			
			Point maxpoint, point1, point2, point3, point4;
			Point averagep;
			double maxlen=0;
					


			for (int i = 0; i < status2.size(); ++i) 
			{
				
				circle(preorig20_orig, Point(trackedFeatures2.at(i).x,trackedFeatures2.at(i).y), 2, Scalar(0, 0, 255), -1);
				circle(preorig20_orig,  Point(corners2.at(i).x,corners2.at(i).y), 2, Scalar(255, 0, 0), -1);
				line(preorig20_orig, Point(trackedFeatures2.at(i).x,trackedFeatures2.at(i).y), Point(corners2.at(i).x,corners2.at(i).y), Scalar(0,255,0), 1);
					
				
					// length between points
					double length=sqrt((trackedFeatures2.at(i).x-corners2.at(i).x)*(trackedFeatures2.at(i).x-corners2.at(i).x)+(trackedFeatures2.at(i).y-corners2.at(i).y)*(trackedFeatures2.at(i).y-corners2.at(i).y));
					


					if(length<maximum_length&&length>minimum_length){ // if the length is shorter than my threshold
					if(maxlen<length)
					{maxlen=length;
					maxpoint.x=corners2.at(i).x;
					maxpoint.y=corners2.at(i).y;
					}
					if(point4.x==0){
					if(point3.x==0){
					if(point2.x==0){
					if(point1.x==0){
					point1.x=corners2.at(i).x;
					point1.y=corners2.at(i).y;
					goto out;
					}
					point2.x=corners2.at(i).x;
					point2.y=corners2.at(i).y;
					goto out;
					}
					point3.x=corners2.at(i).x;
					point3.y=corners2.at(i).y;
					goto out;
					}
					point4.x=corners2.at(i).x;
					point4.y=corners2.at(i).y;
					goto out;
					}

					out:;
					}


			}


					
		int targeti=0;
		double minleng=200000;
					
		for(int i=0; i<color.size(); i++)
		{

		double lengg=sqrt((color.at(i).x-maxpoint.x)*(color.at(i).x-maxpoint.x)+(color.at(i).y-maxpoint.y)*(color.at(i).y-maxpoint.y));
		if(minleng>lengg)
		{
		minleng=lengg;
		targeti=i;
		std::cout<<color.size()<<std::endl;
		}
		}
						
					
		if(color.size()>0)
		{
		//circle(preorig20_orig,Point(color.at(targeti).x,color.at(targeti).y),5,Scalar(255,0,0),-1);
		}
							

		////////find the shortest length

		if(maxlen>minimum_length) // if the length chosen is larger than the threshold
		{
			circle(preorig20_orig,  maxpoint, 15, Scalar(0, 0, 255), 3); // draw

			detected_time=detected_time+0.195;
			std::cout<<"detected_time: "<<detected_time<<endl;
			
			geometry_msgs::Twist i;
			i.linear.x=maxpoint.x;
			i.linear.y=maxpoint.y;
			i_pub.publish(i);
			
			if(!premaxpoint4.x==0) // find the distance between consecutive target points and if they are close they are target.
			{
				double lengthpoints=sqrt((premaxpoint1.x-averagep.x)*(premaxpoint1.x-averagep.x)+(premaxpoint1.y-averagep.y)*(premaxpoint1.y-averagep.y));
				double lengthpoints2=sqrt((premaxpoint2.x-premaxpoint1.x)*(premaxpoint2.x-premaxpoint1.x)+(premaxpoint2.y-premaxpoint1.y)*(premaxpoint2.y-premaxpoint1.y));
				double lengthpoints3=sqrt((premaxpoint3.x-premaxpoint2.x)*(premaxpoint3.x-premaxpoint2.x)+(premaxpoint3.y-premaxpoint2.y)*(premaxpoint3.y-premaxpoint2.y));
				double lengthpoints4=sqrt((premaxpoint4.x-premaxpoint3.x)*(premaxpoint4.x-premaxpoint3.x)+(premaxpoint4.y-premaxpoint3.y)*(premaxpoint4.y-premaxpoint3.y));
				double average=(lengthpoints+lengthpoints2+lengthpoints3+lengthpoints4)/4;

				if(average<target_determination)
				{
				//circle(preorig20_orig,  maxpoint, 10, Scalar(0, 0, 255), -1);
				std::cout << maxlen <<  averagep <<" \n";
				}

				premaxpoint4=premaxpoint3;premaxpoint3=premaxpoint2;
				premaxpoint2=premaxpoint1;premaxpoint1=averagep;
			}
			else
			{
				premaxpoint4=premaxpoint3;premaxpoint3=premaxpoint2;
				premaxpoint2=premaxpoint1;premaxpoint1=averagep;
			}		
		}

		//resize(preorig20_orig, preorig20_orig, Size(FRAME_WIDTH,FRAME_HEIGHT), 0);
		imshow("double",preorig20_orig);
		//imshow("thres",threshold);


 		 preorig37.copyTo(preorig36);
		 preorig38.copyTo(preorig37);
		 preorig39.copyTo(preorig38);
		 preorig40.copyTo(preorig39);
		 orig.copyTo(preorig40);

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
		 orig.copyTo(preorig40);

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
