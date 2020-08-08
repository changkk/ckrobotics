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

static const std::string OPENCV_WINDOW = "Image window";
using namespace cv;
using std::vector;


int H_MIN = 0;
int H_MAX = 0;
int S_MIN = 0;
int S_MAX = 0;
int V_MIN = 255;
int V_MAX = 255;


double pi=3.141592;



//Mat black = Mat::zeros( Size(640,480), CV_8UC3 );

Point prepoint1,prepoint2,prepoint3,prepoint4,prepoint5,prepoint6,prepoint7;
int minimum_distance = 10;
double average;
double minimum_white=1;

//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 10*10;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;




class ImageConverter
{

public: void chatterCallback(const geometry_msgs::PoseStamped& msg)
	{
        	geometry_msgs::Point coord = msg.pose.position;
		x = coord.x;
		y = coord.y;
		z = coord.z;

		if(z<8){
		minimum_white=40;
		}
		if(8<z<10){
		minimum_white=4;
		}
		if(10<z<15){
		minimum_white=2;
		}
		if(15<z){
		minimum_white=1;
		}
	}



private:
  double x, y, z;
double g_x;
double g_y;


Mat T_mxkmGkm = (Mat_<double>(2,1) << 0, 0);
Mat T_SxkmGkm = (Mat_<double>(2,2) << 5, 0, 0, 5);
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber pose_sub;

  ros::Publisher k_pub;

public:
  ImageConverter()
  : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/hexacopter/fisheye_camera/image_raw", 1,
    &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    pose_sub = nh_.subscribe("/hexacopter/mavros/local_position/pose", 1000, &ImageConverter::chatterCallback, this);

    k_pub = nh_.advertise<geometry_msgs::Twist>("/fisheye_image_frame_point", 10);

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




    bool trackObjects = true;
    bool useMorphOps = true;
	average=100;


	Mat HSV;
	Mat threshold;
	int x=0, y=0;



    cv::Mat src;

    src = cv_ptr->image;


        cv::Rect myROI(0, 0, FRAME_WIDTH, FRAME_HEIGHT);
        cv::Mat cameraFeed(src,myROI);



Point cen(FRAME_WIDTH/2,FRAME_HEIGHT/2);
int radius = FRAME_WIDTH/2;

//get the Rect containing the circle:
Rect r(0,0, FRAME_WIDTH,FRAME_HEIGHT);

// obtain the image ROI:
Mat roi(cameraFeed, r);

// make a black mask, same size:
Mat mask(roi.size(), roi.type(), Scalar(255,255,0));
// with a white, filled circle in it:
//circle(mask, Point(radius,radius), radius, Scalar::all(255), -1);
ellipse(mask, cen, Size(FRAME_HEIGHT/2,FRAME_HEIGHT/2), 0,0,360,Scalar::all(255),-1);

// combine roi & mask:
Mat eye_cropped = roi & mask;



		cvtColor(eye_cropped,HSV,COLOR_BGR2HSV);


		inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);

		if(useMorphOps)
		morphOps(threshold);

		if(trackObjects)
			trackFilteredObject(x,y,threshold,cameraFeed);

		//show frames
                imshow("windowName2",threshold);
                imshow("windowName",cameraFeed);
                imshow("windowName1",HSV);
		moveWindow("windowName2",5100,100);
		moveWindow("windowName",3900,100);
		moveWindow("windowName1",4500,100);

		waitKey(30);


  image_pub_.publish(cv_ptr->toImageMsg());
}






  void drawObject(int x, int y,Mat &frame){

          Point point(x,y);



cv::circle(frame,Point(x,y),10,Scalar(0,0,255),-1);

prepoint1=point;
            geometry_msgs::Twist k;
				if(point.x>320&&point.y<240)
				{k.linear.y=(point.x-320)*2;
				k.linear.x=-(point.y-240)*2;}
				else
				{k.linear.y=(point.x-320)*2;
				k.linear.x=(point.y-240)*2;}
                                k_pub.publish(k);




  }
  void morphOps(Mat &thresh){


          Mat erodeElement = getStructuringElement( MORPH_RECT,Size(minimum_white,minimum_white));

          Mat dilateElement = getStructuringElement( MORPH_RECT,Size(6,6));

          erode(thresh,thresh,erodeElement);
          erode(thresh,thresh,erodeElement);


          dilate(thresh,thresh,dilateElement);
          dilate(thresh,thresh,dilateElement);



  }


  void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed){

          Mat temp;
          threshold.copyTo(temp);

          vector< vector<Point> > contours;
          vector<Vec4i> hierarchy;

          findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );

          double refArea = 0;
          bool objectFound = false;
          if (hierarchy.size() > 0) {
                  int numObjects = hierarchy.size();

          if(numObjects<MAX_NUM_OBJECTS){
                          for (int index = 0; index >= 0; index = hierarchy[index][0]) {

                                  Moments moment = moments((cv::Mat)contours[index]);
                                  double area = moment.m00;

                  if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
                                          x = moment.m10/area;
                                          y = moment.m01/area;
                                          objectFound = true;
                                          refArea = area;


                                  }else objectFound = false;


                          }

                          if(objectFound ==true){

                                  drawObject(x,y,cameraFeed);}

                          else
                          {
                                  //if(average<minimum_distance)
                                  {//int estimated_x=prepoint1.x+(prepoint1.x-prepoint2.x);
                                  //int estimated_y=prepoint1.y+(prepoint1.y-prepoint2.y);
				//int estimated_x=prepoint1.x;
                                //  int estimated_y=prepoint1.y;
                                //  cv::circle(cameraFeed,Point(estimated_x,estimated_y),10,Scalar(0,0,255),-1);
				//geometry_msgs::Twist q;
				//if(estimated_x>160&&estimated_y<120)
				//{q.linear.y=(estimated_x-160)/15;
				//q.linear.x=-(estimated_y-120)/15;}
				///else
                                //{q.linear.y=(estimated_x-160)/15;
				//q.linear.x=(estimated_y-120)/15;}
                                //p_pub.publish(q);

}

                          }

                  }
          }


  }




};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter3");
  ImageConverter ic;
  ros::spin();
  return 0;
}
