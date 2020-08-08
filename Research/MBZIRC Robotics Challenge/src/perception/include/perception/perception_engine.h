#ifndef PERCEPTION_ENGINE_H
#define PERCEPTION_ENGINE_H

#include <perception/video_processor.h>
#include <perception/color_detection.h>
#include <perception/position_handler.h>
#include <perception/target.h>
#include <perception/target_tracker.h>
#include <perception/region_of_interest.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <string>
#include <thread>
#include <mutex>

class PerceptionEngine {
public:
  PerceptionEngine(
    ros::NodeHandle& rosNode,
    std::string panTiltCameraTopic,
    std::string fisheyeCameraTopic,
    std::string fisheyeCalibrationFile,
    std::string odometryTopic
  );
  ~PerceptionEngine();
  void initialize(ros::Rate rosRate, std::string filtersFilename, std::string roiDirectory);

private:
  PerceptionEngine();
  void pauseCallback(const std_msgs::Bool::ConstPtr& msg);
  void configureColorFilters(std::string filename);
  void configureColorFilter(ColorDetection& colorDetection, Target::Color color, std::string config);
  void loadRegionsOfInterest(std::string dataDirectory);
  void perspectiveImageEvent(cv::Mat rgbFrame);
  void fisheyeImageEvent(cv::Mat rgbFrame);
  void detectObjectsByColor(const VideoProcessor& videoProcessor, const cv::Mat& rgbFrame, std::vector<ImageObject>& imageObjects, bool isFisheye) const;
  void processObservations(VideoProcessor& videoProcessor, std::vector<ImageObject>& imageObjects, bool useObservations);
  int compareClosestToCenter(const ImageObject& object0, const ImageObject& object1) const;
  void publishTargets() const;
  Target generateTarget(const ImageObject& imageObject) const;
  bool isValidTarget(const Target& target) const;
  bool useFisheyeCamera() const;
  void associateTarget(const Target& target);
  std::tuple<unsigned int, unsigned int> cvPointToTuple(const cv::Point2i& point) const;
  int expectedPixelLength(double height, bool isFisheye) const;
  void perspectiveThreadLoop(ros::Rate rosRate);
  void fisheyeThreadLoop(ros::Rate rosRate);
  void publishState() const;
  std::string getNamespace() const;
  void logMessage(std::string message, bool isError = false) const;

  std::thread* mPerspectiveDetectionThread;
  std::thread* mFisheyeDetectionThread;
  std::vector<Target> mTargets;
  std::vector<ImageObject> mPerspectiveImageObjects;
  std::vector<ImageObject> mFisheyeImageObjects;
  std::mutex mPerspectiveMutex;
  std::mutex mFisheyeMutex;
  ros::NodeHandle* mNodeHandle;
  ros::Subscriber mPauseSubscriber;
  ros::Publisher mTargetPublisher;
  ros::Publisher mStatePublisher;
  ros::Publisher mLoggerPublisher;
  cv::Mat mPerspectiveImageFrame;
  cv::Mat mFisheyeImageFrame;
  VideoProcessor mPerspectiveVideoProcessor;
  VideoProcessor mFisheyeVideoProcessor;
  ColorDetection mRedHighColorDetection;
  ColorDetection mRedLowColorDetection;
  ColorDetection mGreenColorDetection;
  ColorDetection mBlueColorDetection;
  ColorDetection mYellowColorDetection;
  ColorDetection mOrangeColorDetection;
  PositionHandler mPositionHandler;
  TargetTracker mTargetTracker;
  std::vector<RegionOfInterest> mRegionsOfInterest;
  bool mIsPaused;
};

#endif
