#include <perception/image_object.h>

ImageObject::ImageObject() {
  this->mCenter = cv::Point2i(0, 0);
  this->mRadius = 0;
  this->mColor = Target::Color::NONE;
  this->mIsValid = true;
}

ImageObject::ImageObject(cv::Point2i center, int radius, Target::Color color) {
  this->mCenter = center;
  this->mRadius = radius;
  this->mColor = color;
  this->mIsValid = true;
}

const cv::Point2i& ImageObject::getCenter() const {
  return this->mCenter;
}

int ImageObject::getRadius() const {
  return this->mRadius;
}

Target::Color ImageObject::getColor() const {
  return this->mColor;
}

bool ImageObject::getIsValid() const {
  return this->mIsValid;
}

void ImageObject::setCenter(const cv::Point2i& center) {
  this->mCenter = center;
}

void ImageObject::setRadius(int radius) {
  this->mRadius = radius;
}

void ImageObject::setColor(Target::Color color) {
  this->mColor = color;
}

void ImageObject::setIsValid(bool isValid) {
  this->mIsValid = isValid;
}
