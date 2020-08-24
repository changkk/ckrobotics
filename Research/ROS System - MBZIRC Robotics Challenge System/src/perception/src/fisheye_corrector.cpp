/*
  Fisheye image undistortion implementation
*/

// ROS dependencies
#include <ros/ros.h>

// Project dependencies
#include <perception/fisheye_corrector.h>

FisheyeCorrector::FisheyeCorrector(std::string calibFileName, int imgH, int imgW) {
  this->calibFileName = calibFileName;

  this->imgH = imgH;
  this->imgW = imgW;

  get_ocam_model(&(this->ocamModel), &(this->calibFileName[0]));

  std::cout << "pol =" << std::endl;
  for (int i=0; i<this->ocamModel.length_pol; i++) {
    printf("\t%e\n",ocamModel.pol[i]);
  }

  std::cout << "invpol =" << std::endl;
  for (int i=0; i<ocamModel.length_invpol; i++) {
    printf("\t%e\n",ocamModel.invpol[i]);
  }

  printf("\nxc = %f\nyc = %f\n\nwidth = %d\nheight = %d\n",ocamModel.xc,ocamModel.yc,ocamModel.width,ocamModel.height);
}

cv::Mat FisheyeCorrector::undistortImage(cv::Mat imageFrame) {
  cv::Mat rectifiedDst, mapXP, mapYP;
  CvMat mapx, mapy;
  float sf = 4;

  try {
    mapXP = cv::Mat(this->imgH, this->imgW, CV_32FC1);
    mapYP = cv::Mat(this->imgH, this->imgW, CV_32FC1);

    mapx = mapXP;
    mapy = mapYP;

    create_perspecive_undistortion_LUT(&mapx, &mapy, &(this->ocamModel), sf);

    cv::remap(imageFrame, rectifiedDst, mapXP, mapYP, cv::INTER_LINEAR);

    sensor_msgs::ImagePtr pubImage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rectifiedDst).toImageMsg();
    pubImage->header.stamp = ros::Time::now();
  }
  catch (std::exception exception) {
    ROS_ERROR("%s", exception.what());
  }

  return rectifiedDst;
}
