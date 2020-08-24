#include <perception/color_detection.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

void ColorDetection::configureFilter(Target::Color color, unsigned int minHue, unsigned int minSaturation, unsigned int minValue, unsigned int maxHue, unsigned int maxSaturation, unsigned int maxValue) {
  this->mColor = color;
  this->mMinColor = cv::Scalar(minHue, minSaturation, minValue);
  this->mMaxColor = cv::Scalar(maxHue, maxSaturation, maxValue);
}

void ColorDetection::filterByColor(const cv::Mat& rgbFrame, int pixelLength, cv::Mat& outputFrame, bool fitStructure, bool isCircle) const {
  try {
    cv::Mat bgrImage = rgbFrame.clone();
    cv::medianBlur(bgrImage, bgrImage, 3);

    cv::Mat hsvFrame;
    cv::cvtColor(bgrImage, hsvFrame, cv::COLOR_BGR2HSV);

    cv::Mat thresholdFrame;
    cv::inRange(hsvFrame, this->mMinColor, this->mMaxColor, thresholdFrame);

    if (fitStructure && isCircle) {
      cv::Mat structuringElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(pixelLength, pixelLength));
      cv::Mat mask;
      cv::morphologyEx(thresholdFrame, mask, cv::MORPH_OPEN, structuringElement);
      outputFrame = mask.clone();
    }
    else {
      outputFrame = thresholdFrame.clone();
    }
  }
  catch (cv::Exception exception) {
    std::cout << exception.what() << std::endl;
  }
}

void ColorDetection::extractObjectsFromThreshold(const cv::Mat& thresholdFrame, std::vector<ImageObject>& imageObjects, int pixelLength, float height) const {
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> heirarchy;

  cv::findContours(thresholdFrame, contours, heirarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

  this->contoursToImageObjects(contours, imageObjects, pixelLength, height);
}

void ColorDetection::contoursToImageObjects(const std::vector<std::vector<cv::Point>>& contours, std::vector<ImageObject>& imageObjects, int pixelLength, float height) const {
  for (std::vector<std::vector<cv::Point>>::const_iterator iter = contours.begin(); iter != contours.end(); ++iter) {
    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(*iter, center, radius);
    if (2.0 * radius > pixelLength) {
      imageObjects.push_back(ImageObject(center, radius, this->mColor));
    }
  }
}
