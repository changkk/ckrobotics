#ifndef IMAGE_OBJECT_H
#define IMAGE_OBJECT_H

#include <perception/target.h>

#include <opencv2/core/core.hpp>

class ImageObject {
public:
  ImageObject();
  ImageObject(cv::Point2i center, int radius, Target::Color color);

  const cv::Point2i& getCenter() const;
  int getRadius() const;
  Target::Color getColor() const;
  bool getIsValid() const;

  void setCenter(const cv::Point2i& center);
  void setRadius(int radius);
  void setColor(Target::Color color);
  void setIsValid(bool isValid);

private:
  Target::Color mColor;
  cv::Point2i mCenter;
  int mRadius;
  bool mIsValid;
};

#endif
