#include <perception/video_processor.h>

#include <iostream>

static const int QUEUE_SIZE = 1;
static const int OCS_IMAGE_FREQUENCY = 2.0;

VideoProcessor::VideoProcessor(
  ros::NodeHandle& rosNode,
  std::string videoTopic,
  unsigned int horizontalPixels,
  unsigned int verticalPixels,
  std::string fisheyeCalibrationFile,
  PerceptionEngine* perceptionEngine,
  void (PerceptionEngine::*perceptionCallback)(cv::Mat rgbFrame)
) : mImageTransport(rosNode),
    HORIZONTAL_PIXELS(horizontalPixels),
    VERTICAL_PIXELS(verticalPixels) {
  this->mNodeHandle = &rosNode;
  this->mCvImagePtr = NULL;
  this->mPerceptionEngine = perceptionEngine;
  this->mPerceptionCallback = perceptionCallback;
  this->mVideoSubscriber = this->mImageTransport.subscribe(videoTopic.c_str(), QUEUE_SIZE, &VideoProcessor::imageCallback, this);
  this->mVideoPublisher = this->mImageTransport.advertise(std::string(videoTopic + "_annotated").c_str(), QUEUE_SIZE);
  this->mDebugPublisher = this->mImageTransport.advertise(std::string(videoTopic + "_debug").c_str(), QUEUE_SIZE);
  this->mOcsPublisher = this->mImageTransport.advertise(std::string(videoTopic + "_ocs").c_str(), QUEUE_SIZE);
  this->mOcsThread = new std::thread(&VideoProcessor::threadLoop, this);
  this->mDoFisheyeCorrection = false;
  if (fisheyeCalibrationFile.size() > 0) {
    this->mFisheyeCorrector = new FisheyeCorrector(fisheyeCalibrationFile, verticalPixels, horizontalPixels);
  }
  else {
    this->mFisheyeCorrector = NULL;
  }
}

VideoProcessor::~VideoProcessor() {
  if (this->mOcsThread != NULL) {
    this->mOcsThread->join();
    delete this->mOcsThread;
  }
  if (this->mFisheyeCorrector != NULL) {
    delete this->mFisheyeCorrector;
  }
}

void VideoProcessor::enableFisheyeCorrection(bool enable) {
  this->mDoFisheyeCorrection = enable;
}

void VideoProcessor::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    this->mCvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    if (this->mCvImagePtr != NULL) {
      if (this->mFisheyeCorrector != NULL && this->mDoFisheyeCorrection) {
        this->mCvImagePtr->image = this->mFisheyeCorrector->undistortImage(this->mCvImagePtr->image);
      }
      (*this->mPerceptionEngine.*this->mPerceptionCallback)(this->mCvImagePtr->image.clone());
    }
    else {
      ROS_ERROR("VideoProcessor::imageCallback mCvImagePtr is NULL.");
    }
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

void VideoProcessor::annotateTargets(const std::vector<ImageObject>& imageObjects) {
  if (this->mCvImagePtr != NULL) {
    cv::Mat rgbFrame = this->mCvImagePtr->image;
    for (std::vector<ImageObject>::const_iterator iter = imageObjects.begin(); iter != imageObjects.end(); ++iter) {
      if (iter->getIsValid()) {
        cv::circle(rgbFrame, iter->getCenter(), iter->getRadius(), cv::Scalar(255, 255, 0), 3);
      }
      else {
        cv::circle(rgbFrame, iter->getCenter(), iter->getRadius(), cv::Scalar(0, 212, 212), 3);
      }
    }
    this->mVideoPublisher.publish(this->mCvImagePtr->toImageMsg());
    this->mMutex.lock();
    this->mAnnotedFrame = rgbFrame.clone();
    this->mMutex.unlock();
  }
  else {
    ROS_ERROR("VideoProcessor::annotateTargets mCvImagePtr is NULL.");
  }
}

void VideoProcessor::annotateTrackedTarget(const ImageObject& imageObject) {
  if (this->mCvImagePtr != NULL) {
    cv::Mat rgbFrame = this->mCvImagePtr->image;
    cv::circle(rgbFrame, imageObject.getCenter(), imageObject.getRadius(), cv::Scalar(255, 0, 255), 8);
    this->mVideoPublisher.publish(this->mCvImagePtr->toImageMsg());
    this->mMutex.lock();
    this->mAnnotedFrame = rgbFrame.clone();
    this->mMutex.unlock();
  }
  else {
    ROS_ERROR("VideoProcessor::annotateTrackedTarget mCvImagePtr is NULL.");
  }
}

void VideoProcessor::drawBoundary(const PositionHandler& positionHandler, const RegionOfInterest& regionOfInterest) const {
  if (this->mCvImagePtr != NULL) {
    std::tuple<float, float> regionVertex0 = regionOfInterest.getVertex(0);
    std::tuple<float, float> regionVertex1 = regionOfInterest.getVertex(1);
    std::tuple<float, float> regionVertex2 = regionOfInterest.getVertex(2);
    std::tuple<float, float> regionVertex3 = regionOfInterest.getVertex(3);

    std::tuple<float, float> cameraVertex0 = positionHandler.getWorldPositionFromImagePosition(std::tuple<unsigned int, unsigned int>(0, VERTICAL_PIXELS));
    std::tuple<float, float> cameraVertex1 = positionHandler.getWorldPositionFromImagePosition(std::tuple<unsigned int, unsigned int>(HORIZONTAL_PIXELS, VERTICAL_PIXELS));
    std::tuple<float, float> cameraVertex2 = positionHandler.getWorldPositionFromImagePosition(std::tuple<unsigned int, unsigned int>(HORIZONTAL_PIXELS, 0));
    std::tuple<float, float> cameraVertex3 = positionHandler.getWorldPositionFromImagePosition(std::tuple<unsigned int, unsigned int>(0, 0));

    std::tuple<float, float> imageRange = positionHandler.getImageRange();
    std::tuple<float, float> pixelsPerMeter;
    std::get<0>(pixelsPerMeter) = HORIZONTAL_PIXELS / std::get<0>(imageRange);
    std::get<1>(pixelsPerMeter) = VERTICAL_PIXELS / std::get<1>(imageRange);

    this->drawBoundaryLine(positionHandler, regionVertex0, regionVertex1, pixelsPerMeter, regionOfInterest.isValidRegion());
    this->drawBoundaryLine(positionHandler, regionVertex1, regionVertex2, pixelsPerMeter, regionOfInterest.isValidRegion());
    this->drawBoundaryLine(positionHandler, regionVertex2, regionVertex3, pixelsPerMeter, regionOfInterest.isValidRegion());
    this->drawBoundaryLine(positionHandler, regionVertex3, regionVertex0, pixelsPerMeter, regionOfInterest.isValidRegion());
  }
  else {
    ROS_WARN("VideoProcessor::drawBoundary mCvImagePtr is NULL.");
  }
}

void VideoProcessor::debug(const cv::Mat& imageFrame) const {
  cv_bridge::CvImage debugMessage;
  debugMessage.encoding = sensor_msgs::image_encodings::MONO8;
  debugMessage.image = imageFrame;
  this->mDebugPublisher.publish(debugMessage.toImageMsg());
}

void VideoProcessor::drawBoundaryLine(const PositionHandler& positionHandler, const std::tuple<float, float>& vertex0, const std::tuple<float, float>& vertex1, const std::tuple<float, float>& pixelsPerMeter, bool isValid) const {
  if (this->mCvImagePtr != NULL) {
    std::tuple<int, int> imageVertex0 = positionHandler.getImagePositionFromWorldPosition(vertex0, pixelsPerMeter);
    std::tuple<int, int> imageVertex1 = positionHandler.getImagePositionFromWorldPosition(vertex1, pixelsPerMeter);
    cv::Point pt1(std::get<0>(imageVertex0), std::get<1>(imageVertex0));
    cv::Point pt2(std::get<0>(imageVertex1), std::get<1>(imageVertex1));
    cv::Mat rgbFrame = this->mCvImagePtr->image;
    cv::Scalar lineColor;
    if (isValid) {
      lineColor = cv::Scalar(0, 255, 0);
    }
    else {
      lineColor = cv::Scalar(0, 0, 255);
    }
    cv::line(rgbFrame, pt1, pt2, lineColor, 3);
  }
  else {
    ROS_ERROR("VideoProcessor::drawBoundaryLine mCvImagePtr is NULL.");
  }
}

void VideoProcessor::threadLoop() {
  ros::Rate rosRate(OCS_IMAGE_FREQUENCY);
  cv_bridge::CvImage imageMessage;
  imageMessage.encoding = sensor_msgs::image_encodings::BGR8;
  while (this->mNodeHandle->ok()) {
    try {
      this->mMutex.lock();
      imageMessage.image = this->mAnnotedFrame;
      this->mMutex.unlock();
      this->mOcsPublisher.publish(imageMessage.toImageMsg());
    }
    catch (std::exception exception) {
      ROS_ERROR("VideoProcessor::threadLoop - %s", exception.what());
    }
    rosRate.sleep();
  }
}
