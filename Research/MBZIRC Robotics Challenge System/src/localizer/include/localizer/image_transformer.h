/*
  Image Transformer class header file
*/
#ifndef LOCALIZER_IMAGE_TRANSFORMER
#define LOCALIZER_IMAGE_TRANSFORMER

// ROS dependencies
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/ModelStates.h>
#include <cv_bridge/cv_bridge.h>

// C++ dependencies
#include <opencv2/highgui/highgui.hpp>

class ImageTransformer {
  private:
    static const int SUB_IMAGE_QUEUE_SIZE;
    static const int SUB_CAM_INFO_QUEUE_SIZE;
    static const int SUB_CAM_POSE_QUEUE_SIZE;
    static const int PUB_TF_IMAGE_QUEUE_SIZE;

    std::string subImageTopic;
    std::string subCamInfoTopic;
    std::string subCamPoseTopic;
    std::string subImuTopic;
    std::string pubTfImageTopic;
    std::string uavModelName;

    ros::Subscriber imageSubscriber;
    ros::Subscriber camInfoSubscriber;
    ros::Subscriber camPoseSubscriber;
    ros::Subscriber imuSubscriber;
    ros::Publisher tfImagePublisher;

    bool camInfoReady;
    bool heightReady;
    bool imuReady;

    cv_bridge::CvImagePtr cvPtr;
    cv::Mat grayScaleImg;
    cv::Mat edgeImg;
    cv::Mat tfImg;

    int edgeDtLt;
    int edgeDtHt;

    double tfHeight;
    double height;
    double roll;
    double pitch;
    double yaw;

    cv::Mat camProjMat;

    // Flags for Peterson algorithm P-pose O-orientation
    bool flagP[2];
    bool flagO[2];
    int turnP;
    int turnO;

    geometry_msgs::Pose uavPose;

    void imageSubscriberCb(const sensor_msgs::Image::ConstPtr&);
    void camInfoSubscriberCb(const sensor_msgs::CameraInfo::ConstPtr&);
    void camPoseSubscriberCb(const gazebo_msgs::ModelStates::ConstPtr&);
    void imuSubscriberCb(const sensor_msgs::Imu::ConstPtr&);
    void transformImage();

    cv::Mat getXRotMat(double);
    cv::Mat getYRotMat(double);
    cv::Mat getZRotMat(double);
    cv::Mat getHmgnsRotMat(cv::Mat);
    cv::Mat getHmgnsTfMat(cv::Vec3d);
    cv::Mat getHmgnsCdts(cv::Point3d);
    std::vector<cv::Point2f> getPixelVector(cv::Mat, cv::Mat, cv::Mat, cv::Mat);
    cv::Mat getRotationMatrix(double, double, double);

  public:
    ImageTransformer(std::string, std::string, std::string, std::string, std::string, std::string, int, int, double);
    void start(ros::NodeHandle);
};

#endif
