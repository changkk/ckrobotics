#include <perception/perception_engine.h>
#include <perception/TargetArray.h>
#include <perception/StateMsg.h>

#include <fstream>
#include <sstream>
#include <std_msgs/String.h>
#include <boost/filesystem.hpp>

static const int QUEUE_SIZE = 1;
static const std::string TOPIC_PERCEPTION_TARGETS = "perception/targets";
static const std::string TOPIC_PERCEPTION_STATE = "perception/state";
static const std::string TOPIC_PERCEPTION_PAUSE = "perception/pause";
static const std::string TOPIC_LOGGER = "/ocs/log_message";
const unsigned int HORIZONTAL_PIXELS = 640;
const unsigned int VERTICAL_PIXELS = 480;
const float CAMERA_HORIZONTAL_FOV = 1.0472;
const float CAMERA_VERTICAL_FOV = 0.81727571;
// const float CAMERA_VERTICAL_FOV = 0.785398;
const float CAMERA_HANDOFF_HEIGHT = 2.5;
const float DRAW_BOUNDARY_HEIGHT_THRESHOLD = 2.0;

PerceptionEngine::PerceptionEngine(
  ros::NodeHandle& rosNode,
  std::string panTiltCameraTopic,
  std::string fisheyeCameraTopic,
  std::string fisheyeCalibrationFile,
  std::string odometryTopic
) : mPerspectiveVideoProcessor(rosNode, panTiltCameraTopic, HORIZONTAL_PIXELS, VERTICAL_PIXELS, "", this, &PerceptionEngine::perspectiveImageEvent),
    mFisheyeVideoProcessor(rosNode, fisheyeCameraTopic, HORIZONTAL_PIXELS, VERTICAL_PIXELS, fisheyeCalibrationFile, this, &PerceptionEngine::fisheyeImageEvent),
    mPositionHandler(rosNode, odometryTopic, HORIZONTAL_PIXELS, VERTICAL_PIXELS, CAMERA_HORIZONTAL_FOV, CAMERA_VERTICAL_FOV),
    mTargetTracker(rosNode, HORIZONTAL_PIXELS, VERTICAL_PIXELS) {
  this->mNodeHandle = &rosNode;
  this->mPerspectiveDetectionThread = NULL;
  this->mFisheyeDetectionThread = NULL;
  this->mPauseSubscriber = this->mNodeHandle->subscribe<std_msgs::Bool>(TOPIC_PERCEPTION_PAUSE, QUEUE_SIZE, &PerceptionEngine::pauseCallback, this);
  this->mTargetPublisher = this->mNodeHandle->advertise<perception::TargetArray>(TOPIC_PERCEPTION_TARGETS, QUEUE_SIZE);
  this->mStatePublisher = this->mNodeHandle->advertise<perception::StateMsg>(TOPIC_PERCEPTION_STATE, QUEUE_SIZE);
  this->mLoggerPublisher = this->mNodeHandle->advertise<std_msgs::String>(TOPIC_LOGGER, QUEUE_SIZE);
  this->mIsPaused = false;
}

PerceptionEngine::~PerceptionEngine() {
  if (this->mPerspectiveDetectionThread != NULL) {
    this->mPerspectiveDetectionThread->join();
    delete this->mPerspectiveDetectionThread;
  }
  if (this->mFisheyeDetectionThread != NULL) {
    this->mFisheyeDetectionThread->join();
    delete this->mFisheyeDetectionThread;
  }
}

void PerceptionEngine::initialize(ros::Rate rosRate, std::string filtersFilename, std::string roiDirectory) {
  this->mPerspectiveDetectionThread = new std::thread(&PerceptionEngine::perspectiveThreadLoop, this, rosRate);
  this->mFisheyeDetectionThread = new std::thread(&PerceptionEngine::fisheyeThreadLoop, this, rosRate);
  this->configureColorFilters(filtersFilename);
  this->loadRegionsOfInterest(roiDirectory);
}

void PerceptionEngine::pauseCallback(const std_msgs::Bool::ConstPtr& msg) {
  this->mIsPaused = msg->data;
  std::stringstream ss;
  ss << "Perception pause state: " << this->mIsPaused;
  this->logMessage(ss.str());
}

void PerceptionEngine::configureColorFilters(std::string filename) {
  std::ifstream file(filename);
  std::string line;
  if (file.is_open()) {
    //TODO: Add error checking here.
    std::getline(file, line);
    this->configureColorFilter(this->mRedHighColorDetection, Target::Color::RED, line);
    std::getline(file, line);
    this->configureColorFilter(this->mRedLowColorDetection, Target::Color::RED, line);
    std::getline(file, line);
    this->configureColorFilter(this->mGreenColorDetection, Target::Color::GREEN, line);
    std::getline(file, line);
    this->configureColorFilter(this->mBlueColorDetection, Target::Color::BLUE, line);
    std::getline(file, line);
    this->configureColorFilter(this->mYellowColorDetection, Target::Color::YELLOW, line);
    std::getline(file, line);
    this->configureColorFilter(this->mOrangeColorDetection, Target::Color::ORANGE, line);
    file.close();
  }
}

void PerceptionEngine::configureColorFilter(ColorDetection& colorDetection, Target::Color color, std::string config) {
  std::string colorText;
  unsigned int minHue = 0;
  unsigned int minSaturation = 0;
  unsigned int minValue = 0;
  unsigned int maxHue = 0;
  unsigned int maxSaturation = 0;
  unsigned int maxValue = 0;
  std::stringstream ss(config);
  ss >> colorText >> minHue >> minSaturation >> minValue >> maxHue >> maxSaturation >> maxValue;
  colorDetection.configureFilter(color, minHue, minSaturation, minValue, maxHue, maxSaturation, maxValue);
}

void PerceptionEngine::loadRegionsOfInterest(std::string directory) {
  boost::filesystem::path path(directory);
  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator iter(path); iter != end_itr; ++iter) {
    if (boost::filesystem::is_regular_file(iter->path())) {
      this->mRegionsOfInterest.push_back(RegionOfInterest(iter->path().string()));
    }
  }
}

void PerceptionEngine::perspectiveImageEvent(cv::Mat rgbFrame) {
  mPerspectiveMutex.lock();
  this->mPerspectiveImageFrame = rgbFrame.clone();
  std::vector<ImageObject> imageObjects = this->mPerspectiveImageObjects;
  mPerspectiveMutex.unlock();
  try {
    this->processObservations(this->mPerspectiveVideoProcessor, imageObjects, !(this->useFisheyeCamera()) && !this->mIsPaused);
    this->publishTargets();
  }
  catch (std::exception exception) {
    ROS_ERROR("PerceptionEngine::perspectiveImageEvent - %s", exception.what());
  }
  this->publishState();
}

void PerceptionEngine::fisheyeImageEvent(cv::Mat rgbFrame) {
  mFisheyeMutex.lock();
  this->mFisheyeImageFrame = rgbFrame.clone();
  std::vector<ImageObject> imageObjects = this->mFisheyeImageObjects;
  mFisheyeMutex.unlock();
  try {
    this->mFisheyeVideoProcessor.enableFisheyeCorrection(this->useFisheyeCamera());
    this->processObservations(this->mFisheyeVideoProcessor, imageObjects, this->useFisheyeCamera() && !this->mIsPaused);
    this->publishTargets();
  }
  catch (std::exception exception) {
    ROS_ERROR("PerceptionEngine::fisheyeImageEvent - %s", exception.what());
  }
}

void PerceptionEngine::detectObjectsByColor(const VideoProcessor& videoProcessor, const cv::Mat& rgbFrame, std::vector<ImageObject>& imageObjects, bool isFisheye) const {
  cv::Mat thresholdFrame;
  cv::Mat combinedFrame;
  bool useStructure = this->useFisheyeCamera();
  float height = this->mPositionHandler.getCurrentHeight();
  int pixelLength = this->expectedPixelLength(height, isFisheye);

  try {
    if (rgbFrame.dims > 0) {
      this->mRedHighColorDetection.filterByColor(rgbFrame, pixelLength, thresholdFrame, useStructure, true);
      combinedFrame = thresholdFrame.clone();
      this->mRedLowColorDetection.filterByColor(rgbFrame, pixelLength, thresholdFrame, useStructure, true);
      cv::bitwise_or(thresholdFrame, combinedFrame, combinedFrame);
      this->mRedLowColorDetection.extractObjectsFromThreshold(combinedFrame.clone(), imageObjects, pixelLength, height);

      this->mGreenColorDetection.filterByColor(rgbFrame, pixelLength, thresholdFrame, useStructure, true);
      this->mGreenColorDetection.extractObjectsFromThreshold(thresholdFrame.clone(), imageObjects, pixelLength, height);
      cv::bitwise_or(thresholdFrame, combinedFrame, combinedFrame);

      this->mBlueColorDetection.filterByColor(rgbFrame, pixelLength, thresholdFrame, useStructure, true);
      this->mBlueColorDetection.extractObjectsFromThreshold(thresholdFrame.clone(), imageObjects, pixelLength, height);
      cv::bitwise_or(thresholdFrame, combinedFrame, combinedFrame);

      this->mYellowColorDetection.filterByColor(rgbFrame, pixelLength, thresholdFrame, useStructure, true);
      this->mYellowColorDetection.extractObjectsFromThreshold(thresholdFrame.clone(), imageObjects, pixelLength, height);
      cv::bitwise_or(thresholdFrame, combinedFrame, combinedFrame);

      // this->mOrangeColorDetection.filterByColor(rgbFrame, pixelLength, thresholdFrame, useStructure, false);
      // this->mOrangeColorDetection.extractObjectsFromThreshold(thresholdFrame.clone(), imageObjects, pixelLength, height);
      // cv::bitwise_or(thresholdFrame, combinedFrame, combinedFrame);

      videoProcessor.debug(combinedFrame);
    }
  }
  catch (std::exception exception) {
    ROS_ERROR("PerceptionEngine::detectObjectsByColor - %s", exception.what());
  }
}

// TODO: Refactor this.
void PerceptionEngine::processObservations(VideoProcessor& videoProcessor, std::vector<ImageObject>& imageObjects, bool useObservations) {
  if (useObservations) {
    Target centerMostTarget;
    ImageObject centerMostObject;
    bool wasTargetFound = false;
    for (std::vector<ImageObject>::iterator iter = imageObjects.begin(); iter != imageObjects.end(); ++iter) {
      Target target = this->generateTarget(*iter);
      bool isValidTarget = this->isValidTarget(target);
      iter->setIsValid(isValidTarget);
      if (isValidTarget) {
        this->associateTarget(target);
        // Track target if tracking enabled.
        if (this->mTargetTracker.getStatus()) {
          if (!wasTargetFound || this->compareClosestToCenter(centerMostObject, *iter) < 0) {
            centerMostTarget = target;
            centerMostObject = *iter;
            wasTargetFound = true;
          }
        }
      }
    }
    if (wasTargetFound) {
      this->mTargetTracker.identifyTrackedTarget(centerMostTarget, this->mPositionHandler);
      videoProcessor.annotateTrackedTarget(centerMostObject);
    }
    if (this->mPositionHandler.getCurrentHeight() > DRAW_BOUNDARY_HEIGHT_THRESHOLD) {
      for (std::vector<RegionOfInterest>::const_iterator iter = this->mRegionsOfInterest.begin(); iter != this->mRegionsOfInterest.end(); ++iter) {
        this->mPerspectiveVideoProcessor.drawBoundary(this->mPositionHandler, *iter);
      }
    }
    videoProcessor.annotateTargets(imageObjects);
  }
  else {
    videoProcessor.annotateTargets(std::vector<ImageObject>());
  }
}

int PerceptionEngine::compareClosestToCenter(const ImageObject& object0, const ImageObject& object1) const {
  float distance0 = sqrt(pow(object0.getCenter().x - (int) HORIZONTAL_PIXELS, 2) + pow(object0.getCenter().y - (int) VERTICAL_PIXELS, 2));
  float distance1 = sqrt(pow(object1.getCenter().x - (int) HORIZONTAL_PIXELS, 2) + pow(object1.getCenter().y - (int) VERTICAL_PIXELS, 2));
  return distance0 < distance1 ? 1 : -1;
}

void PerceptionEngine::publishTargets() const {
  perception::TargetArray msg;
  for (std::vector<Target>::const_iterator iter = this->mTargets.begin(); iter != this->mTargets.end(); ++iter) {
    if (iter->confirmedTarget()) {
      msg.targets.push_back(iter->serialize());
    }
  }
  this->mTargetPublisher.publish(msg);
}

Target PerceptionEngine::generateTarget(const ImageObject& imageObject) const {
  Target target;
  target.setPosition(this->mPositionHandler.getWorldPositionFromImagePosition(this->cvPointToTuple(imageObject.getCenter())));
  target.setColor(imageObject.getColor());
  return target;
}

bool PerceptionEngine::isValidTarget(const Target& target) const {
  bool inValidRegion = false;
  for (std::vector<RegionOfInterest>::const_iterator iter = this->mRegionsOfInterest.begin(); iter != this->mRegionsOfInterest.end(); ++iter) {
    if (iter->insideRegion(std::get<0>(target.getPosition()), std::get<1>(target.getPosition()))) {
      if (!iter->isValidRegion()) {
        return false;
      }
      else {
        inValidRegion = true;
      }
    }
  }
  return inValidRegion;
}

bool PerceptionEngine::useFisheyeCamera() const {
  return this->mTargetTracker.getStatus() && (this->mPositionHandler.getCurrentHeight() < CAMERA_HANDOFF_HEIGHT);
}

void PerceptionEngine::associateTarget(const Target& target) {
  for (std::vector<Target>::iterator iter = this->mTargets.begin(); iter != this->mTargets.end(); ++iter) {
    if (*iter == target) {
      iter->setPosition(target.getPosition());
      iter->addObservation(ros::Time::now());
      return;
    }
  }
  this->mTargets.push_back(target);
}

std::tuple<unsigned int, unsigned int> PerceptionEngine::cvPointToTuple(const cv::Point2i& point) const {
  return std::make_tuple((unsigned int)point.x, (unsigned int)point.y);
}

int PerceptionEngine::expectedPixelLength(double height, bool isFisheye) const {
  const int MAX_PIXEL_LENGTH = 100;
  const int MIN_PIXEL_LENGTH = 6;
  int pixelLength = 0;
  if (isFisheye) {
    if (height < 0.3) {
      return MAX_PIXEL_LENGTH;
    }
    pixelLength = (int) (12.0 / height);
  }
  else {
    if (height < 1.0) {
      return MAX_PIXEL_LENGTH;
    }
    pixelLength = (int) (100.0 / height);
    // pixelLength = (int) 77.468 * pow(height, -1.209);
  }
  return std::max(std::min(pixelLength, MAX_PIXEL_LENGTH), MIN_PIXEL_LENGTH);
}

void PerceptionEngine::perspectiveThreadLoop(ros::Rate rosRate) {
  sleep(5);
  while (this->mNodeHandle->ok()) {
    try {
      std::vector<ImageObject> imageObjects;
      mPerspectiveMutex.lock();
      cv::Mat imageFrame = this->mPerspectiveImageFrame.clone();
      mPerspectiveMutex.unlock();
      this->detectObjectsByColor(this->mPerspectiveVideoProcessor, imageFrame, imageObjects, false);
      mPerspectiveMutex.lock();
      this->mPerspectiveImageObjects = imageObjects;
      mPerspectiveMutex.unlock();
    }
    catch (std::exception exception) {
      ROS_ERROR("PerceptionEngine::perspectiveThreadLoop - %s", exception.what());
    }
    ros::spinOnce();
    rosRate.sleep();
  }
}

void PerceptionEngine::fisheyeThreadLoop(ros::Rate rosRate) {
  sleep(5);
  while (this->mNodeHandle->ok()) {
    try {
      std::vector<ImageObject> imageObjects;
      mFisheyeMutex.lock();
      cv::Mat imageFrame = this->mFisheyeImageFrame.clone();
      mFisheyeMutex.unlock();
      this->detectObjectsByColor(this->mFisheyeVideoProcessor, imageFrame, imageObjects, true);
      mFisheyeMutex.lock();
      this->mFisheyeImageObjects = imageObjects;
      mFisheyeMutex.unlock();
    }
    catch (std::exception exception) {
      ROS_ERROR("PerceptionEngine::fisheyeThreadLoop - %s", exception.what());
    }
    ros::spinOnce();
    rosRate.sleep();
  }
}

void PerceptionEngine::publishState() const {
  perception::StateMsg stateMsg;
  stateMsg.camera = this->useFisheyeCamera() ? "FISHEYE" : "PERSPECTIVE";
  stateMsg.is_tracking_target = this->mTargetTracker.getStatus();
  stateMsg.is_paused = this->mIsPaused;
  stateMsg.height_above_ground = this->mPositionHandler.getCurrentHeight();
  stateMsg.expected_pixel_length = this->expectedPixelLength(this->mPositionHandler.getCurrentHeight(), this->useFisheyeCamera());
  this->mStatePublisher.publish(stateMsg);
}

void PerceptionEngine::logMessage(std::string message, bool isError) const {
  std_msgs::String logMessage;
  if (isError) {
    logMessage.data = "[ERROR] ";
  }
  logMessage.data += "[" + this->getNamespace() + "] [PerceptionEngine] " + message;
  this->mLoggerPublisher.publish(logMessage);
}

std::string PerceptionEngine::getNamespace() const {
  std::string ns = this->mNodeHandle->getNamespace();
  ns.erase(0, 2);
  return ns;
}
