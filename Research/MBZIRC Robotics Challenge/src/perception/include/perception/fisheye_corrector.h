/*
  Fisheye Corrector class header file
*/
#ifndef LOCALIZER_FISHEYE_CORRECTOR
#define LOCALIZER_FISHEYE_CORRECTOR

#include <perception/ocam_functions.h>

// ROS dependencies
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

class FisheyeCorrector {
  private :
    std::string calibFileName;

    int imgH;
    int imgW;

    struct ocam_model ocamModel;

  public :
    FisheyeCorrector(std::string, int, int);
    cv::Mat undistortImage(cv::Mat imageFrame);
};

#endif
