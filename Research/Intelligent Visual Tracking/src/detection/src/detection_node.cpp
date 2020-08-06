#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <sstream>
#include <vector>
#include "sensor_msgs/Image.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PolygonStamped.h"
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
//#include "opencv2/cudaoptflow.hpp"
//#include "opencv2/cudaarithm.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
//#include <opencv2/gpu/gpu.hpp>

using namespace cv;
// using namespace cv::cuda;
using namespace std;

static const string OPENCV_WINDOW = "Image window";

// Image Resize
bool meas_flag = false, send_cue = false;
int w = 1504*3/4, l = 1504*3/4;
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
// vector<Point2f> meas(1);


// double a_data[16] = {1.0, 0, 0, 0,
//                      0, 1.0, 0, 0,
//                      0, 0, 1.0, 0,
//                      0, 0, 0, 1.0};

// double c_data[8] = {1, 0, 0, 0,
//                     0, 1, 0, 0};

// double ct_data[8] = {1, 0,
//                      0, 1,
//                      0, 0,
//                      0, 0};

// double x_data[4] = {w/2,
//                     l/2,
//                     0.1,
//                     0.1};

// double p_data[16] = {1, 1, 1, 1,
//                      1, 1, 1, 1,
//                      1, 1, 1, 1,
//                      1, 1, 1, 1};

// double g_data[8] =  {1, 1,
//                      1, 1,
//                      1, 1,
//                      1, 1};

double z_data[2] = {0, 0};

// double r_data[4] = {100, 0,
//                    0, 100};


// Mat A_m = Mat(4,4,CV_32FC1,a_data);
// Mat A_m = Mat::eye(4,4,CV_64FC1);
// Mat C = Mat(2,4,CV_64FC1,c_data);
// Mat Ct = Mat(4,2,CV_64FC1,ct_data);
// Mat x = Mat(4,1,CV_64FC1,x_data);
// Mat x_old = Mat(4,1,CV_64FC1,x_data);
// Mat P = Mat(4,4,CV_64FC1,p_data);
// Mat A_mt = Mat(4,4,CV_64FC1,a_data);
// Mat G = Mat(4,2,CV_64FC1,g_data);
Mat z = Mat(2,1,CV_64FC1,z_data);
// Mat tmp = Mat(2,1,CV_64FC1,z_data);
// Mat tmp2 = Mat(4,1,CV_64FC1,x_data);
// Mat R = Mat(2,2,CV_64FC1,r_data);
// Mat I = Mat::eye(4,4,CV_64FC1);

KalmanFilter KF(4,2,0);
Mat_<float> measurement(2,1); 


Mat and_img, and_img2, canny_edge;
Mat img_old = Mat::zeros(size, CV_8UC1);
Mat frameL, frameR;
//cuda::GpuMat d_frameL(frame_p), d_frameR(img_old);
//cuda::GpuMat d_flow;
//Ptr<cuda::FarnebackOpticalFlow> d_calc = cuda::FarnebackOpticalFlow::create();
Mat flowxy, flowx, flowy, image_1;
bool running = true, gpuMode = true;
int64 t0 = 0, t1 = 1, tc0 = 0, tc1 = 0;
int no_cons = 0, no_meas_counter = 0, meas_counter = 0;
Point2i est_center;

// Mat mat = Mat::zeros(size, CV_8UC1);
Mat bin_out_diff = Mat::zeros(size, CV_8UC1);
char charry[20];

double t_old = 0, t_now = 0, dt = 0, xv = 0, yv = 0;
float range = 0;
std::string to_string(double x);
// Ptr<Tracker> tracker;

template <typename T>
inline T mapVal(T x, T a, T b, T c, T d)
{
    x = ::max(::min(x, b), a);
    return c + (d-c) * (x-a) / (b-a);
}

static void colorizeFlow(const Mat &u, const Mat &v, Mat &dst)
{
    double uMin, uMax;
    cv::minMaxLoc(u, &uMin, &uMax, 0, 0);
    double vMin, vMax;
    cv::minMaxLoc(v, &vMin, &vMax, 0, 0);
    uMin = ::abs(uMin); uMax = ::abs(uMax);
    vMin = ::abs(vMin); vMax = ::abs(vMax);
    float dMax = static_cast<float>(::max(::max(uMin, uMax), ::max(vMin, vMax)));

    dst.create(u.size(), CV_8UC3);
    for (int y = 0; y < u.rows; ++y)
    {
        for (int x = 0; x < u.cols; ++x)
        {
            dst.at<uchar>(y,3*x) = 0;
            dst.at<uchar>(y,3*x+1) = (uchar)mapVal(-v.at<float>(y,x), -dMax, dMax, 0.f, 255.f);
            dst.at<uchar>(y,3*x+2) = (uchar)mapVal(u.at<float>(y,x), -dMax, dMax, 0.f, 255.f);
        }
    }
}

void drawOptFlowMap (const Mat& flow, Mat& cflowmap, int step, const Scalar& color)
{
 for(int y = 0; y < cflowmap.rows; y += step)
 {
      for(int x = 0; x < cflowmap.cols; x += step)
      {
       const Point2f& fxy = flow.at< Point2f>(y, x);
       line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
        color);
       circle(cflowmap, Point(cvRound(x+fxy.x), cvRound(y+fxy.y)), 1, color, -1);

      }
 }
}

class ImageConverter
{
  ros::NodeHandle nh_;
  ros::Subscriber sub;
  ros::Publisher gimbal_cue;
  
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
    

    // namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    // cv::destroyWindow(OPENCV_WINDOW);
    
  }

  void rangeCB(const std_msgs::Float64 &msg)
  {
    range = msg.data;
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
    frame = frame(Range::all(), Range(1504, 3008));
    resize(frame, frame, size);
    GaussianBlur(frame, frame, Size(3, 3), 10, 10);

    // bilateralFilter(frame,frame_small,9,18,5);
    // cvtColor(frame, frame_hsv, COLOR_BGR2HSV);
    cvtColor(frame, frame, COLOR_BGR2GRAY);
    

    pMOG2->apply(frame, fgMaskMOG2);

    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(fgMaskMOG2, fgMaskMOG2_open, MORPH_OPEN, element);

    // t_now = ros::Time::now().toSec();
    // dt = t_now - t_old;
    // t_old = t_now;

    // cout << endl << dt << endl;

    // cout << endl << fgMaskMOG2_open.size() << endl << bin_out_old.size() << endl << bin_out_diff.size();

    // subtract(fgMaskMOG2_open, bin_out_old, bin_out_diff);
    // bin_out_old = fgMaskMOG2_open;

    Sobel(frame, grad_x, CV_16S, 1, 0, 1);
    Sobel(frame, grad_y, CV_16S, 0, 1, 1);
    Canny(frame,canny_edge,50,150);

    convertScaleAbs(grad_x, grad_x);
    convertScaleAbs(grad_y, grad_y);
    addWeighted(grad_x, 0.5, grad_y, 0.5, 0, sobel_img);
    // equalizeHist(sobel_img,sobel_img);
    // adaptiveThreshold(sobel_img,sobel_img,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY_INV,11,2);


    // addWeighted(and_img, 1, and_img, 1, 0, and_img);
    // log(and_img,and_img)
    // cv::log(and_img,and_img);
    // cv::pow(sobel_img,3,x3);
    // cv::pow(sobel_img,2,x2);

    // addWeighted(x3, 0.0000448, x2, -0.0146, 0, x3);
    // addWeighted(sobel_img, 1.65, Mat::ones(size, CV_8UC1), 61.547, 0, sobel_img);
    // addWeighted(x3, 1, sobel_img, 1, 0, sobel_img);
    
    bitwise_and(sobel_img, fgMaskMOG2_open, and_img);
    
    threshold(and_img,and_img,0,255,CV_THRESH_BINARY);
    
    Mat element2 = getStructuringElement(MORPH_RECT, Size(1, 1));    
    morphologyEx(and_img, and_img, MORPH_OPEN, element2);
    dilate(and_img,and_img,element2);
    

    //Optical Flow--------------------------------------------------------------------------------------------------
    threshold(sobel_img,sobel_img,150,0,THRESH_TOZERO_INV);
    // adaptiveThreshold(and_img,and_img,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY,3,0);


    // //------------------------------------------------------------------------------------------------------------    
    // // frame.copyTo(frame_p);
    // frame_p = frame;
    // GaussianBlur(frame_p, frame_p, Size(9, 9),10,10);
    
    // // cout << frame.type() << endl;
    // // cout << frame_p.type() << endl;
    // // cout << img_old.type() << endl;
    
    
    // tc0 = getTickCount();

    // d_frameL.upload(img_old);
    // d_frameR.upload(frame_p);    
    // d_calc->calc(d_frameR, d_frameL, d_flow);
    
    // // calcOpticalFlowFarneback(img_old, frame_p, flowxy,0.6, 7, 5, 7, 7, 1.2,0);
    
    // img_old = frame_p;

    // cuda::GpuMat planes[2];
    // cv::cuda::split(d_flow, planes);
    // planes[0].download(flowx);
    // planes[1].download(flowy);
    // d_flow.download(flowxy);
    
    // tc1 = getTickCount();

    // // cout << cv::max(flowxy,0) <<endl;
    // // cout << cv::min(flowxy,0) <<endl;
    // // cout << flowxy.size() <<endl;
    // // cout << flowxy.channels() <<endl;
    
    // // Mat planes[] = {flowx, flowy};
    // // split(flowxy, planes);
    // // flowx = planes[0];
    // // flowy = planes[1];

    // cartToPolar(flowx,flowy,mag,ang);
    
    // mag.convertTo(mag,CV_8U,40);
    // // GaussianBlur(mag, mag, Size(91, 91),100,100);
    // Sobel(mag, grad_x, CV_16S, 1, 0, 1);
    // Sobel(mag, grad_y, CV_16S, 0, 1, 1);
    
    // convertScaleAbs(grad_x, grad_x);
    // convertScaleAbs(grad_y, grad_y);
    
    // addWeighted(grad_x, 0.5, grad_y, 0.5, 0, mag);

    // threshold(mag,mag,80,255,CV_THRESH_BINARY);
    // dilate(mag,mag,element2);
    
    // //------------------------------------------------------------------------------------------------------------    

    // mag.convertTo(mag,)

    // adaptiveThreshold(mag,mag,255,CV_THRESH_BINARY,ADAPTIVE_THRESH_GAUSSIAN_C,13,0);

    // cout <<  sobel_img.type() << " " << sobel_img.size() << endl;
    // cout <<  mag.type() << " " << mag.size() << endl;
    
    // bitwise_and(sobel_img, mag, and_img2);

    //Historgram Calculation-----------------------------------------------------------------------------------
    
    // // // Initialize parameters
    // int histSize = 256;    // bin size
    // float rangew[] = { 0, 255 };
    // const float *ranges[] = { rangew };
 
    // // // Calculate histogram
    // MatND hist;
    // calcHist( &mag, 1, 0, Mat(), hist, 1, &histSize, ranges, true, false );
    // // // equalizeHist(hist,hist);

    // // // Plot the histogram
    // int hist_w = 500; int hist_h = 400;
    // int bin_w = cvRound( (double) hist_w/histSize );
 
    // Mat histImage( hist_h, hist_w, CV_8UC1, Scalar( 0,0,0) );
    // normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
     
    // for( int i = 1; i < histSize; i++ )
    // {
    //   line( histImage, Point( bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1)) ) ,
    //                    Point( bin_w*(i), hist_h - cvRound(hist.at<float>(i)) ),
    //                    Scalar( 255, 0, 0), 2, 8, 0  );
    // }

    // if (histImage.size() == size_null)
    // {
    //   histImage = Mat::zeros(size,CV_8U);
    // }

    // // namedWindow( "Result", 1 );    
    // imshow( "Result", histImage );
    //---------------------------------------------------------------------------------------------------------

    // integral(mag,d_ft);

    // GaussianBlur(mag, mag, Size(9, 9), 100, 100);
    // adaptiveThreshold(mag,mag,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY,11,2);
    

    // cvtColor(flowxy, image_1, CV_GRAY2BGR);
    // cvtColor(flowxy, image_1, CV_32F);
    

    // Mat planes[] = {flowx, flowy};
    // split(flowxy, planes);
    // flowx = planes[0];
    // flowy = planes[1];

    // colorizeFlow(flowx, flowy, image_1);
    // flowxy
    // image_1 = flowxy;

    // Mat cflow;
    // cvtColor(img_old, cflow, CV_GRAY2BGR);
    // drawOptFlowMap(flowxy, cflow, 10, CV_RGB(0, 255, 0));

    // findContours(and_img,cont,)
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
      minEnclosingCircle((Mat)contours_poly[i], center[i], radius[i]);
    }

    /// Draw polygonal contour + bonding rects + circles
    Mat drawing = Mat::zeros(and_img.size(), CV_8UC1);
    // and_img.copyTo(drawing);
    frame.copyTo(drawing);

    for (int i = 0; i < no_cons; i++)
    {
      // Scalar color = ;
      // drawContours(drawing, contours_poly, i, Scalar(255, 0, 255), 1, 8, vector<Vec4i>(), 0, Point());
      // rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), Scalar(255, 0, 255), 2, 8, 0);
      circle(drawing, center[i], (int)radius[i], Scalar(255, 0, 255), 2, 8, 0);      
    }


    t_now = ros::Time::now().toSec();
    dt = t_now - t_old;
    t_old = t_now;

    // cvSort

    if (center.size() > 0 )
    {
      if ( center[0].x != 0 & center[0].y != 0)
      {
        // cout << center[0].x << ", " << center[0].y << endl;

        z.at<double>(0,0) = center[0].x;
        z.at<double>(1,0) = center[0].y;
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
      
      // z.at<double>(0,0) = center[0].x;
      // z.at<double>(1,0) = center[0].y;
    }

    //Kalman Filter

    if (no_meas_counter < 20)    
    {
      Mat prediction = KF.predict();
      Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
      est_center.x = round(predictPt.x);
      est_center.y = round(predictPt.y);

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
    }

    circle(drawing, est_center, 15, Scalar(0, 0, 0), 8, 8, 0);
    

    //Update Image Text----------------------------------------------------------------------------------------------------------------------------------
    stringstream s;
    // s.str("");
    // s << "opt. flow FPS: " << cvRound((getTickFrequency() / (tc1 - tc0)));
    // putText(image_1, s.str(), Point(5, 65), FONT_HERSHEY_SIMPLEX, 1., Scalar(255, 0, 255), 2);
    // putText(mag, s.str(), Point(5, 65), FONT_HERSHEY_SIMPLEX, 1., Scalar(100, 0, 255), 2);

    s.str("");
    s << "Cue";
    putText(drawing, s.str(), Point(est_center.x+18, est_center.y+18), FONT_HERSHEY_SIMPLEX, 1., Scalar(100, 0, 255), 2);
    

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
    // imshow("Contours", drawing);
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
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_converter");
  // cout << getBuildInformation;

  // intialization of KF...
  KF.transitionMatrix = (Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
  measurement.setTo(Scalar(0));
  ImageConverter ic;

  //   ros::Rate loop_rate(5);

  //   Mat fgMaskMOG2;
  //   Mat frame;est_center.x > l

  //   Ptr<BackgroundSubtractor> pMOG2; //MOG2 Background subtractor
  //   pMOG2 = createBackgroundSubtractorMOG2();
  // //   namedWindow(OPENCV_WINDOW);

  //   while(ros::ok)
  //   {
  //     // if(cv_ptr->image.rows>0)
  //     // {
  //     //     // frame = cv_ptr->image;
  //     // }

  //     pMOG2->apply(frame, fgMaskMOG2);
  //     // imshow("Frame", frame);
  //     imshow("FG Mask MOG 2", fgMaskMOG2);

  //     ros::spinOnce();
  //     loop_rate.sleep();
  //   }
  ros::spin();

  return 0;
}

// void image_callback(const sensor_msgs::CompressedImage img_msg)
// {
//         // usleep(delay);
// }

// void euler2quat (double pitch, double roll, double yaw)
// {
//     double cy = cos(yaw * 0.5);
// 	double sy = sin(yaw * 0.5);
// 	double cr = cos(roll * 0.5);
// 	double sr = sin(roll * 0.5);
// 	double cp = cos(pitch * 0.5);
// 	double sp = sin(pitch * 0.5);

// 	q[0] = cy * cr * cp + sy * sr * sp;
// 	q[1] = cy * sr * cp - sy * cr * sp;
// 	q[2] = cy * cr * sp + sy * sr * cp;
// 	q[3] = sy * cr * cp - cy * sr * sp;
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "detection_node");

//     ros::NodeHandle n;

//     ros::Subscriber sub_image = n.subscribe("omnicam_image/image", 1, image_callback);
//     ros::Publisher pub_angles = n.advertise<geometry_msgs::PoseStamped>("gimbal_target_orientation", 1);

//     while(ros::ok())
//     {

//     }

//     ros::spin();
//     return 0;
// }
