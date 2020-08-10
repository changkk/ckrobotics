#ifndef VIDEO_PROCESSOR_H
#define VIDEO_PROCESSOR_H

#include <perception/image_object.h>
#include <perception/position_handler.h>
#include <perception/region_of_interest.h>
#include <perception/fisheye_corrector.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <thread>
#include <mutex>

class PerceptionEngine;

class VideoProcessor {
public:
  VideoProcessor(
    ros::NodeHandle& rosNode,
    std::string videoTopic,
    unsigned int horizontalPixels,
    unsigned int verticalPixels,
    std::string fisheyeCalibrationFile,
    PerceptionEngine* perceptionEngine,
    void (PerceptionEngine::*perceptionCallback)(cv::Mat rgbFrame)
  );
  ~VideoProcessor();
  void enableFisheyeCorrection(bool enable);
  void annotateTargets(const std::vector<ImageObject>& imageObjects);
  void annotateTrackedTarget(const ImageObject& imageObject);
  void drawBoundary(const PositionHandler& positionHandler, const RegionOfInterest& regionOfInterest) const;
  void debug(const cv::Mat& imageFrame) const;

private:
  VideoProcessor();
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void drawBoundaryLine(const PositionHandler& positionHandler, const std::tuple<float, float>& vertex0, const std::tuple<float, float>& vertex1, const std::tuple<float, float>& pixelsPerMeter, bool isValid) const;
  void threadLoop();

  std::thread* mOcsThread;
  ros::NodeHandle* mNodeHandle;
  std::mutex mMutex;
  cv_bridge::CvImagePtr mCvImagePtr;
  cv::Mat mAnnotedFrame;
  image_transport::Subscriber mVideoSubscriber;
  image_transport::Publisher mVideoPublisher;
  image_transport::Publisher mDebugPublisher;
  image_transport::Publisher mOcsPublisher;
  image_transport::ImageTransport mImageTransport;
  const unsigned int HORIZONTAL_PIXELS;
  const unsigned int VERTICAL_PIXELS;
  bool mDoFisheyeCorrection;
  FisheyeCorrector* mFisheyeCorrector;
  PerceptionEngine* mPerceptionEngine;
  void (PerceptionEngine::*mPerceptionCallback)(cv::Mat rgbFrame);
};

#endif
