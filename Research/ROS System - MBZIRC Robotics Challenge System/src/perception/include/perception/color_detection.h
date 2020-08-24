#ifndef COLOR_DETECTION_H
#define COLOR_DETECTION_H

#include <perception/image_object.h>
#include <perception/target.h>

#include <opencv2/core/core.hpp>

class ColorDetection {
public:
  void configureFilter(Target::Color color, unsigned int minHue, unsigned int minSaturation, unsigned int minValue, unsigned int maxHue, unsigned int maxSaturation, unsigned int maxValue);
  void filterByColor(const cv::Mat& rgbFrame, int pixelLength, cv::Mat& outputFrame, bool fitStructure, bool isCircle) const;
  void extractObjectsFromThreshold(const cv::Mat& thresholdFrame, std::vector<ImageObject>& imageObjects, int pixelLength, float height) const;

private:
  void contoursToImageObjects(const std::vector<std::vector<cv::Point>>& contours, std::vector<ImageObject>& imageObjects, int pixelLength, float height) const;

  Target::Color mColor;
  cv::Scalar mMinColor;
  cv::Scalar mMaxColor;
};

#endif
