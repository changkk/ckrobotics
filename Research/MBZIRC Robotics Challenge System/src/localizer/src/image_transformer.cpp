/*
  Image Transformer class implementation
*/
// ROS dependencies
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

// Project dependencies
#include <localizer/image_transformer.h>

// C++ dependencies
#include <opencv2/calib3d/calib3d.hpp>

#define PI 3.14159265
#define RAD_TO_PI 180/PI

// Initialize static constants
const int ImageTransformer::SUB_IMAGE_QUEUE_SIZE = 1;
const int ImageTransformer::SUB_CAM_INFO_QUEUE_SIZE = 100;
const int ImageTransformer::SUB_CAM_POSE_QUEUE_SIZE = 100;
const int ImageTransformer::PUB_TF_IMAGE_QUEUE_SIZE = 100;

ImageTransformer::ImageTransformer(std::string subImageTopic, std::string subCamInfoTopic, std::string subCamPoseTopic, std::string subImuTopic, std::string pubTfImageTopic, std::string uavModelName, int edgeDtLt, int edgeDtHt, double tfHeight) {
  this->subImageTopic = subImageTopic;
  this->subCamInfoTopic = subCamInfoTopic;
  this->subCamPoseTopic = subCamPoseTopic;
  this->pubTfImageTopic = pubTfImageTopic;
  this->subImuTopic = subImuTopic;
  this->uavModelName = uavModelName;

  this->edgeDtLt = edgeDtLt;
  this->edgeDtHt = edgeDtHt;

  this->tfHeight = tfHeight;

  this->flagP[0] = false;
  this->flagP[1] = false;
  this->flagO[0] = false;
  this->flagO[1] = false;
}

void ImageTransformer::start(ros::NodeHandle nh) {
  /*
    1. Register subscribers
    2. Advertise publishers
  */
  this->imageSubscriber = nh.subscribe(
    this->subImageTopic,
    this->SUB_IMAGE_QUEUE_SIZE,
    &ImageTransformer::imageSubscriberCb,
    this
  );

  this->camInfoSubscriber = nh.subscribe(
    this->subCamInfoTopic,
    this->SUB_CAM_INFO_QUEUE_SIZE,
    &ImageTransformer::camInfoSubscriberCb,
    this
  );

  this->camPoseSubscriber = nh.subscribe(
    this->subCamPoseTopic,
    this->SUB_CAM_POSE_QUEUE_SIZE,
    &ImageTransformer::camPoseSubscriberCb,
    this
  );

  this->tfImagePublisher = nh.advertise<sensor_msgs::Image>(
    this->pubTfImageTopic,
    this->PUB_TF_IMAGE_QUEUE_SIZE
  );

  this->imuSubscriber = nh.subscribe(
    this->subImuTopic,
    100,
    &ImageTransformer::imuSubscriberCb,
    this
  );

  // Initialize booleans to false at the beginning
  this->camInfoReady = false;
  this->heightReady = false;
  this->imuReady = false;

  this->camProjMat = cv::Mat(3,4,CV_64FC1,0.0);
}

void ImageTransformer::imageSubscriberCb(const sensor_msgs::Image::ConstPtr& msg) {
  try {
    this->cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    if (this->heightReady && this->camInfoReady && this->imuReady) {
      this->transformImage();
    }
  } catch (cv_bridge::Exception e) {
    ROS_ERROR("Exception reading camera image at localizer->image_tf %s", e.what());
  }
}

void ImageTransformer::camInfoSubscriberCb(const sensor_msgs::CameraInfo::ConstPtr& msg) {
  // Only do this once
  if (!this->camInfoReady) {
    for(int i=0; i<3; i++) {
      for(int j=0; j<4; j++) {
        this->camProjMat.at<double>(i,j) = msg->P[i*4+j];
      }
    }
    this->camInfoReady = true;
  }

  this->camInfoSubscriber.shutdown();
}

void ImageTransformer::camPoseSubscriberCb(const gazebo_msgs::ModelStates::ConstPtr& msg) {
  /*
    1. Identify the hexacoptor index
    2. Extract data and call method to process
  */
  int hexIdx = -1;

  for (int i=0; i<msg->name.size(); i++) {
    if (this->uavModelName == msg->name[i]) {
      hexIdx = i;
      break;
    }
  }

  if (hexIdx == -1) {
    ROS_WARN("Image Transformer: Hexacoptor model not detected!");
  } else {
    this->flagP[0] = true;
    this->turnP = 1;

    while(this->flagP[1] && this->turnP == 1) {
      // wait
    }

    /*
      Start critical section for pose
      Set height
    */

    this->height = msg->pose[hexIdx].position.z;
    // tf::Quaternion camQuat(
    //   msg->pose[hexIdx].orientation.x,
    //   msg->pose[hexIdx].orientation.y,
    //   msg->pose[hexIdx].orientation.z,
    //   msg->pose[hexIdx].orientation.w
    // );
    // tf::Matrix3x3 m(camQuat);
    // m.getRPY(this->roll, this->pitch, this->yaw);
    
    /*
      End critical section for pose
    */

    this->flagP[0] = false;
    this->heightReady = true;
  }
}

void ImageTransformer::imuSubscriberCb(const sensor_msgs::Imu::ConstPtr& msg) {
  // Extract RPY from Imu

  this->flagO[0] = true;
  this->turnO = 1;

  while(this->flagO[1] && this->turnO == 1) {
    // wait
  }

  /*
    Start critical section for orientation
  */
  tf::Quaternion camQuat(
    msg->orientation.x,
    msg->orientation.y,
    msg->orientation.z,
    msg->orientation.w
  );
  tf::Matrix3x3 m(camQuat);
  m.getRPY(this->roll, this->pitch, this->yaw);

  /*
    Exit critical section for orientation
  */
  this->flagO[0] = false;
  this->imuReady = true;
}

void ImageTransformer::transformImage() {
  // Convert image to grayscale
  cv::cvtColor(this->cvPtr->image, this->grayScaleImg, CV_BGR2GRAY);
  // Extract edges
  cv::Canny(this->grayScaleImg, this->edgeImg, this->edgeDtLt, this->edgeDtHt, 3);

  /*
    Approach :
      1. Take right handed coordinate system which originates at the track right below the UAV with Z axis looking upwards
      2. Calculate the rotation matrices R1 and R2 between world coordinates and camera coordinates for the two camera positions (Current position and the zero RP position at h2)
      3. Pick four points (h1*tan(theta)+-a, +-a)
      4. Calculate 4 vectors for 4 points and the camera coordinate origin (Pw - C)
      5. Using [X,Y,Z,1]^T = R*T*[U,V,W,1]^T, convert points into camera coordinates
      6. Convert points in camera coordinates into image coordinates
      7. With four corresponding points in each image, get homography matrix
      8. Get transformed image using the homography matrix
  */

  this->flagP[1] = true;
  this->turnP = 0;
  this->flagO[1] = true;
  this->turnO = 0;

  while(this->flagP[0] && this->turnP == 0) {
    // waiting
  }
  while(this->flagO[0] && this->turnO == 0) {
    // waiting
  }

  /*
    Start critical section
  */
  // Step 1
  cv::Point3d o1(0,0,this->height);
  cv::Point3d o2(0,0,this->tfHeight);

  // Step 2
  // cv::Mat rotYPR = this->getZRotMat(this->yaw)*this->getYRotMat(this->pitch)*this->getZRotMat(this->roll);
  // double alpha, beta, gamma;
  // alpha = asin(-rotYPR.at<double>(1,2));
  // beta = asin(rotYPR.at<double>(0,2)/cos(alpha));
  
  cv::Mat xPIRot = this->getXRotMat(PI);
  cv::Mat yPitchRot = this->getYRotMat(0);//this->pitch);
  cv::Mat xRollRot = this->getXRotMat(-this->roll);
  // cv::Mat rtt_ = this->getRotationMatrix(this->roll, this->pitch, this->yaw);
  // cv::Mat R1 = xPIRot *rtt_;
  cv::Mat R1 = xPIRot;// * xRollRot * yPitchRot;
  cv::Mat R2 = xPIRot;

  // Step 3, a=1
  cv::Point3d p1(this->height*tan(this->pitch)+1, 1, 0);
  cv::Point3d p2(this->height*tan(this->pitch)-1, 1, 0);
  cv::Point3d p3(this->height*tan(this->pitch)+1, -1, 0);
  cv::Point3d p4(this->height*tan(this->pitch)-1, -1, 0);
  
  std::cout << this->roll*RAD_TO_PI << "     ," << this->pitch*RAD_TO_PI << "     ," << this->yaw*RAD_TO_PI << std::endl;
  // std::cout << this->roll*RAD_TO_PI << std::endl;
  /*
    Exit critical section
  */
  this->flagP[1] = false;
  this->flagO[1] = false;

  // Step 4
  cv::Vec3d p1V1 = p1 - o1;
  cv::Vec3d p2V1 = p2 - o1;
  cv::Vec3d p3V1 = p3 - o1;
  cv::Vec3d p4V1 = p4 - o1;
  
  cv::Vec3d p1V2 = p1 - o2;
  cv::Vec3d p2V2 = p2 - o2;
  cv::Vec3d p3V2 = p3 - o2;
  cv::Vec3d p4V2 = p4 - o2;

  // Step 5
  cv::Mat hmgnsRotMatR1 = this->getHmgnsRotMat(R1);
  cv::Mat hmgnsRotMatR2 = this->getHmgnsRotMat(R2);
  cv::Mat hmgnsTfMatP1V1 = this->getHmgnsTfMat(p1V1);
  cv::Mat hmgnsTfMatP2V1 = this->getHmgnsTfMat(p2V1);
  cv::Mat hmgnsTfMatP3V1 = this->getHmgnsTfMat(p3V1);
  cv::Mat hmgnsTfMatP4V1 = this->getHmgnsTfMat(p4V1);
  cv::Mat hmgnsTfMatP1V2 = this->getHmgnsTfMat(p1V2);
  cv::Mat hmgnsTfMatP2V2 = this->getHmgnsTfMat(p2V2);
  cv::Mat hmgnsTfMatP3V2 = this->getHmgnsTfMat(p3V2);
  cv::Mat hmgnsTfMatP4V2 = this->getHmgnsTfMat(p4V2);
  cv::Mat hmgnsCdtMatP1 = this->getHmgnsCdts(p1);
  cv::Mat hmgnsCdtMatP2 = this->getHmgnsCdts(p2);
  cv::Mat hmgnsCdtMatP3 = this->getHmgnsCdts(p3);
  cv::Mat hmgnsCdtMatP4 = this->getHmgnsCdts(p4);

  cv::Mat hmgnsp1C1 = hmgnsRotMatR1 * hmgnsTfMatP1V1 * hmgnsCdtMatP1;
  cv::Mat hmgnsp2C1 = hmgnsRotMatR1 * hmgnsTfMatP2V1 * hmgnsCdtMatP2;
  cv::Mat hmgnsp3C1 = hmgnsRotMatR1 * hmgnsTfMatP3V1 * hmgnsCdtMatP3;
  cv::Mat hmgnsp4C1 = hmgnsRotMatR1 * hmgnsTfMatP4V1 * hmgnsCdtMatP4;
  cv::Mat hmgnsp1C2 = hmgnsRotMatR2 * hmgnsTfMatP1V2 * hmgnsCdtMatP1;
  cv::Mat hmgnsp2C2 = hmgnsRotMatR2 * hmgnsTfMatP2V2 * hmgnsCdtMatP2;
  cv::Mat hmgnsp3C2 = hmgnsRotMatR2 * hmgnsTfMatP3V2 * hmgnsCdtMatP3;
  cv::Mat hmgnsp4C2 = hmgnsRotMatR2 * hmgnsTfMatP4V2 * hmgnsCdtMatP4;

  // Step 6
  cv::Mat p1I1 = camProjMat * hmgnsp1C1;
  cv::Mat p2I1 = camProjMat * hmgnsp2C1;
  cv::Mat p3I1 = camProjMat * hmgnsp3C1;
  cv::Mat p4I1 = camProjMat * hmgnsp4C1;
  cv::Mat p1I2 = camProjMat * hmgnsp1C2;
  cv::Mat p2I2 = camProjMat * hmgnsp2C2;
  cv::Mat p3I2 = camProjMat * hmgnsp3C2;
  cv::Mat p4I2 = camProjMat * hmgnsp4C2;

  // Step 7
  std::vector<cv::Point2f> src, dst;
  src = getPixelVector(p1I1, p2I1, p3I1, p4I1);
  dst = getPixelVector(p1I2, p2I2, p3I2, p4I2);

  try {
    cv::Mat homographyMat = cv::findHomography(src, dst, 0, 3, cv::noArray());
    
    // Step 8
    this->tfImg = cv::Mat(this->grayScaleImg.size(), this->grayScaleImg.type());
    cv::warpPerspective(this->grayScaleImg, this->tfImg, homographyMat, tfImg.size());
    
    // Publish edge image to sensor_msgs::Image
    sensor_msgs::ImagePtr pubImage = cv_bridge::CvImage(std_msgs::Header(), "mono8", this->tfImg).toImageMsg();
    pubImage->header.stamp = ros::Time::now();

    tfImagePublisher.publish(pubImage);
  } catch (std::exception e) {
    ROS_ERROR("Homography transformation error: %s", e.what());
  }
}

// Get rotation matrix about X axis
cv::Mat ImageTransformer::getXRotMat(double theta) {
  cv::Mat xRot = cv::Mat(3,3,CV_64FC1);
  
  xRot.at<double>(0,0) = 1.0;
  xRot.at<double>(0,1) = 0.0;
  xRot.at<double>(0,2) = 0.0;
  xRot.at<double>(1,0) = 0.0;
  xRot.at<double>(1,1) = cos(theta);
  xRot.at<double>(1,2) = -sin(theta);
  xRot.at<double>(2,0) = 0.0;
  xRot.at<double>(2,1) = sin(theta);
  xRot.at<double>(2,2) = cos(theta);
  
  return xRot;
}

// Get rotation matrix about Y axis
cv::Mat ImageTransformer::getYRotMat(double psi) {
  cv::Mat yRot = cv::Mat(3,3,CV_64FC1);
  
  yRot.at<double>(0,0) = cos(psi);
  yRot.at<double>(0,1) = 0.0;
  yRot.at<double>(0,2) = sin(psi);
  yRot.at<double>(1,0) = 0.0;
  yRot.at<double>(1,1) = 1.0;
  yRot.at<double>(1,2) = 0.0;
  yRot.at<double>(2,0) = -sin(psi);
  yRot.at<double>(2,1) = 0.0;
  yRot.at<double>(2,2) = cos(psi);
  
  return yRot;
}

// Get rotation matrix about Z axis
cv::Mat ImageTransformer::getZRotMat(double phi) {
  cv::Mat zRot = cv::Mat(3,3,CV_64FC1);
  
  zRot.at<double>(0,0) = cos(phi);
  zRot.at<double>(0,1) = -sin(phi);
  zRot.at<double>(0,2) = 0;
  zRot.at<double>(1,0) = sin(phi);
  zRot.at<double>(1,1) = cos(phi);
  zRot.at<double>(1,2) = 0;
  zRot.at<double>(2,0) = 0;
  zRot.at<double>(2,1) = 0;
  zRot.at<double>(2,2) = 1;

  return zRot;
}

// Get homogenous matrix for rotation
cv::Mat ImageTransformer::getHmgnsRotMat(cv::Mat R) {
  cv::Mat hmgnsRotMat = cv::Mat(4,4,CV_64FC1);
  hmgnsRotMat.at<double>(0,0) = R.at<double>(0,0);
  hmgnsRotMat.at<double>(0,1) = R.at<double>(0,1);
  hmgnsRotMat.at<double>(0,2) = R.at<double>(0,2);
  hmgnsRotMat.at<double>(0,3) = 0.0;
  hmgnsRotMat.at<double>(1,0) = R.at<double>(1,0);
  hmgnsRotMat.at<double>(1,1) = R.at<double>(1,1);
  hmgnsRotMat.at<double>(1,2) = R.at<double>(1,2);
  hmgnsRotMat.at<double>(1,3) = 0.0;
  hmgnsRotMat.at<double>(2,0) = R.at<double>(2,0);
  hmgnsRotMat.at<double>(2,1) = R.at<double>(2,1);
  hmgnsRotMat.at<double>(2,2) = R.at<double>(2,2);
  hmgnsRotMat.at<double>(2,3) = 0.0;
  hmgnsRotMat.at<double>(3,0) = 0.0;
  hmgnsRotMat.at<double>(3,1) = 0.0;
  hmgnsRotMat.at<double>(3,2) = 0.0;
  hmgnsRotMat.at<double>(3,3) = 1.0;

  return hmgnsRotMat;
}

// Get homogenous matrix for translation
cv::Mat ImageTransformer::getHmgnsTfMat(cv::Vec3d tf) {
  cv::Mat hmgnsTfMat = cv::Mat(4,4,CV_64FC1);
  hmgnsTfMat.at<double>(0,0) = 1;
  hmgnsTfMat.at<double>(0,1) = 0;
  hmgnsTfMat.at<double>(0,2) = 0;
  hmgnsTfMat.at<double>(0,3) = tf[0];
  hmgnsTfMat.at<double>(1,0) = 0;
  hmgnsTfMat.at<double>(1,1) = 1;
  hmgnsTfMat.at<double>(1,2) = 0;
  hmgnsTfMat.at<double>(1,3) = tf[1];
  hmgnsTfMat.at<double>(2,0) = 0;
  hmgnsTfMat.at<double>(2,1) = 0;
  hmgnsTfMat.at<double>(2,2) = 1;
  hmgnsTfMat.at<double>(2,3) = tf[2];
  hmgnsTfMat.at<double>(3,0) = 0;
  hmgnsTfMat.at<double>(3,1) = 0;
  hmgnsTfMat.at<double>(3,2) = 0;
  hmgnsTfMat.at<double>(3,3) = 1;

  return hmgnsTfMat;
}

// Get homogenous coordinates of a 3D point
cv::Mat ImageTransformer::getHmgnsCdts(cv::Point3d p) {
  cv::Mat hmgnsCdtMat = cv::Mat(4,1,CV_64FC1);
  hmgnsCdtMat.at<double>(0,0) = p.x;
  hmgnsCdtMat.at<double>(1,0) = p.y;
  hmgnsCdtMat.at<double>(2,0) = p.z;
  hmgnsCdtMat.at<double>(3,0) = 1;

  return hmgnsCdtMat;
}

std::vector<cv::Point2f> ImageTransformer::getPixelVector(cv::Mat m1, cv::Mat m2, cv::Mat m3, cv::Mat m4) {
  std::vector<cv::Point2f> pointVector;
  cv::Point2f p1,p2,p3,p4;

  p1.x = m1.at<double>(0,0)/m1.at<double>(2,0);
  p1.y = m1.at<double>(1,0)/m1.at<double>(2,0);
  p2.x = m2.at<double>(0,0)/m2.at<double>(2,0);
  p2.y = m2.at<double>(1,0)/m2.at<double>(2,0);
  p3.x = m3.at<double>(0,0)/m3.at<double>(2,0);
  p3.y = m3.at<double>(1,0)/m3.at<double>(2,0);
  p4.x = m4.at<double>(0,0)/m4.at<double>(2,0);
  p4.y = m4.at<double>(1,0)/m4.at<double>(2,0);

  pointVector.push_back(p1);
  pointVector.push_back(p2);
  pointVector.push_back(p3);
  pointVector.push_back(p4);

  return pointVector;
}

cv::Mat ImageTransformer::getRotationMatrix (double roll, double pitch, double yaw) {
  cv::Mat rotMat = cv::Mat(3,3,CV_64FC1);

  rotMat.at<double>(0,0) = cos(yaw)*cos(pitch);
  rotMat.at<double>(0,1) = cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll);
  rotMat.at<double>(0,2) = cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll);
  rotMat.at<double>(1,0) = sin(yaw)*cos(pitch);
  rotMat.at<double>(1,1) = sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll);
  rotMat.at<double>(1,2) = sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll);
  rotMat.at<double>(2,0) = -1*sin(pitch);
  rotMat.at<double>(2,1) = cos(pitch)*sin(roll);
  rotMat.at<double>(2,2) = cos(pitch)*cos(roll);

  return rotMat;
}
