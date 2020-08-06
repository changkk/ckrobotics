#include <iostream>
#include "ros/ros.h"
#include "opencv2/core/utility.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
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
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>

using namespace cv;
using namespace std;
bool central_detection = false;
bool send_cue = false;
double targ_azim_det = 0, targ_azim_trk = 0, targ_elev_det = 0, targ_elev_trk = 0;
double hfov, vfov;
geometry_msgs::Point32 box_pos, img_size, box_size, est_center;    
int no_meas_counter = 0;

//Callback for central camera detection
void central_detec_cb(const geometry_msgs::PolygonStamped msg)
{

    box_pos = msg.polygon.points[0];
    img_size = msg.polygon.points[1];
    box_size = msg.polygon.points[2];
    

    //If something shows up here, set a flag high
    central_detection = true;
    no_meas_counter = 0;

//    targ_azim_det = (box_pos.x/img_size.x-0.5) * hfov + gimbal_yaw;
//    targ_elev_det = (box_pos.y/img_size.y-0.5) * vfov + gimbal_pitch;


}

void tracking_cb(const geometry_msgs::PolygonStamped msg)
{
//    geometry_msgs::Point32 box_pos, img_size, box_size;    

//    box_pos = msg.polygon.points[0];
//    img_size = msg.polygon.points[1];
//    box_size = msg.polygon.points[2];
    

    //If something shows up here, set a flag high
//    central_detection = true;

    // Target bearing relative to gimbal base (relative to boom)
//    targ_azim_trk = (box_pos.x/img_size.x-0.5) * hfov + gimbal_yaw;
//    targ_elev_trk = (box_pos.y/img_size.y-0.5) * vfov + gimbal_pitch;

}


// double boresight_angular_offset(double fov, double )

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bearing_ekf");
    ros::NodeHandle n;

    ros::Subscriber central_detec = n.subscribe("/yolo_detection_box", 1, central_detec_cb);
    ros::Subscriber tracker_info = n.subscribe("/tracker_info", 1, tracking_cb);
    ros::Publisher yolo_filtered_pub = n.advertise<geometry_msgs::PolygonStamped>("/filtered_box", 1);



    cv::KalmanFilter KF(4, 2, 0);
    Mat_<float> measurement(2,1);

    double rate = 30; //Hz

    KF.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1/rate, 0, 0, 1, 0, 1/rate, 0, 0, 1, 0, 0, 0, 0, 1);
    measurement.setTo(Scalar(0));

    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(1e-2));
    setIdentity(KF.measurementNoiseCov, Scalar::all(100));
    setIdentity(KF.errorCovPost, Scalar::all(.005));

    bool initialized = false;

    ros::Rate loop_rate(rate);

    est_center.x = 0;
    est_center.y = 0;

    while (ros::ok)
    {

        // Initialize KF
        if (central_detection && ~initialized)
        {
            KF.statePre.at<float>(0) = box_pos.x;
            KF.statePre.at<float>(1) = box_pos.y;
            KF.statePre.at<float>(2) = 0;
            KF.statePre.at<float>(3) = 0;

            initialized = true;
            // cout<<"initialized"<<endl;
        }

        if (initialized)
        {

            if (no_meas_counter < 90)
            {
                Mat prediction = KF.predict();
                Point predictPt(prediction.at<float>(0), prediction.at<float>(1));
                est_center.x = predictPt.x;
                est_center.y = predictPt.y;
                // cout<<"prediction"<<endl;

            }
            else
            {
                send_cue = false;
            }

            if (central_detection)
            {
                measurement(0) = box_pos.x;
                measurement(1) = box_pos.y;

                Mat estimated = KF.correct(measurement);
                Point statePt(estimated.at<float>(0), estimated.at<float>(1));
                est_center.x = statePt.x;
                est_center.y = statePt.y;

                send_cue = true;
                // cout<<"correction"<<endl;

            }
            else
            {
                no_meas_counter++;
                if(no_meas_counter>100)
                    no_meas_counter = 100;
            }
            
        }

        central_detection = false;

        if(send_cue)
        {

            geometry_msgs::PolygonStamped yolo_filtered;
            yolo_filtered.polygon.points.reserve(3);
            yolo_filtered.polygon.points.push_back(est_center);
            yolo_filtered.polygon.points.push_back(img_size);
            yolo_filtered.polygon.points.push_back(box_size);
            yolo_filtered_pub.publish(yolo_filtered);
    
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    //   ros::spin();

    return 0;
}
