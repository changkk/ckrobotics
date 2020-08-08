/*
  Fisheye image undistortion implementation
*/

// ROS dependencies
#include <ros/ros.h>

// Project dependencies

#include <localizer/fisheye_corrector.hpp>
#include <opencv2/opencv.hpp>


//Initialise FisheyeCorrector object states 
const uint8_t FisheyeCorrector::FISHEYECORR_STATE_INITIALISED = 0x00;
const uint8_t FisheyeCorrector::FISHEYECORR_STATE_FISHEYE_CAMERAINFO = 0x01;
const uint8_t FisheyeCorrector::FISHEYECORR_STATE_PANTILT_CAMERAINFO = 0x02;
const uint8_t FisheyeCorrector::FISHEYECORR_STATE_FISHEYEPANTILT_CAMERAINFO = 0x03;
const uint8_t FisheyeCorrector::FISHEYECORR_STATE_READY = 0x07;
const uint8_t FisheyeCorrector::FISHEYECORR_STATE_BUSY = 0x0F;

// Initialize static constants
const int FisheyeCorrector::SUB_IMAGE_QUEUE_SIZE = 100;
const int FisheyeCorrector::PUB_IMAGE_QUEUE_SIZE = 100;
const double FisheyeCorrector::MAP_RESOLUTION = 0.05;
// const double FisheyeCorrector::MAP_RESOLUTION = 0.1255;

//Dummy functions
void connected(const ros::SingleSubscriberPublisher &) {}
void disconnected(const ros::SingleSubscriberPublisher &) {}

//FisheyeCorrector class Constructor
FisheyeCorrector::FisheyeCorrector(FisheyeCorrector_Topics *_topics, std::string calibFileName, int imgH, int imgW, bool enFisheyeCorrection)
{
  this->_topic_base = _topics;
  this->sync_ = 0;
  this->exitflag_ = false;

  //Initialise status flag
  this->statusFlag = FISHEYECORR_STATE_INITIALISED;

  this->imgH = imgH;
  this->imgW = imgW;
  this->calibFileName = calibFileName;
  
  get_ocam_model(&(this->ocamModel), (char*)(this->calibFileName.c_str()));
   
  ROS_INFO_STREAM("FisheyeCorrector object initialised");
}

void FisheyeCorrector::start(ros::NodeHandle nh)
{
  /*
    1. Set flags
    2. Register subscibers
    3. Advertise publishers
    4. Start the CV thread
  */

  this->newImgMtx.lock();
  this->newImage_ = false;
  this->newImgMtx.unlock();

  this->skippedImgCnt = 0;

  //Store the ROS handle
  nh_ = &nh;

//Ravi: Code has been modified to listen to bothe image and the UAV pose updates in
//      a synchronize manner.
#if 0
  this->imageSubscriber = nh.subscribe(
    this->subImageTopic,
    this->SUB_IMAGE_QUEUE_SIZE,
    &FisheyeCorrector::imageSubscriberCb,
    this
  );

  this->imagePublisher = nh.advertise<sensor_msgs::Image>(
    this->pubImageTopic,
    this->PUB_IMAGE_QUEUE_SIZE
  );
#else
  //Initialize the sequence number generator.
  this->msg_cnt_ = 0;

  //Subscribe to FishEye camera info
  fisheye_camera_info_sub = nh_->subscribe(this->_topic_base->subFECamInfo , 1, &FisheyeCorrector::fisheye_camera_info_cb,this);
  pt_camera_info_sub = nh_->subscribe(this->_topic_base->subPTCamInfo , 1, &FisheyeCorrector::pt_camera_info_cb,this);
  
  ros::Subscriber gps_sub = nh_->subscribe(this->_topic_base->subUAVGPSPoseTopic, 1, &FisheyeCorrector::gpsSubscriberCb, this);

  scanService_    = nh_->advertiseService(this->_topic_base->srvTopic,    &FisheyeCorrector::serviceInterface,    this);

  //This is to avoid roscore to chnage the header as the message is passed through the ROS queues
  ros::AdvertiseOptions op = ros::AdvertiseOptions::create<clog_msgs::ScanPose>(this->_topic_base->laserTopic, 1, &connected, &disconnected, ros::VoidPtr(), NULL);
  op.has_header = false;
  this->scanPublisher = nh_->advertise(op);

#endif

  this->cvThread = new boost::thread(boost::bind(&FisheyeCorrector::cvProcess, this));

  ros::spin();
}

FisheyeCorrector::~FisheyeCorrector()
{
  std::cout << "Destructor on FisheyeCorrector called..." << std::endl;
  {
    //boost::mutex::scoped_lock lock(this->newImgMtx);
    this->newImgMtx.lock();
    this->exitflag_ = true;
    this->newImgMtx.unlock();
    this->newImgCondition_.notify_all();
  }

  this->cvThread->interrupt();
  this->cvThread->join();
  delete this->cvThread;
  delete this->sync_;
  
  // cv::destroyWindow("FisheyeImage");
}

bool FisheyeCorrector::serviceInterface(clog_msgs::imgForward::Request& req, clog_msgs::imgForward::Response& resp)
{
  this->newImgMtx.lock();
  bool exitflag = this->exitflag_;
  this->newImgMtx.unlock();

  if(exitflag){
    resp.ret = 0;
    return true;
  }

  std::cout << "FisheyeCorrector serviceInterface " << (int)req.cmd << "  " << (int)req.type << std::endl << std::flush;

  if ((uint8_t)req.cmd == 1)
  {
    this->pose_sub_.subscribe(*nh_, this->_topic_base->subUAVPoseTopic, 10);
    if ((uint8_t)req.type == 0) //FishEye camera
    {
      this->img_sub_.subscribe(*nh_, this->_topic_base->subFEImageTopic, 10);
      cameratype_ = 0;
      this->enFisheyeCorrection = true;
    }
    else{ //Pan-tilt camera
      this->img_sub_.subscribe(*nh_, this->_topic_base->subPTImageTopic, 10);      
      cameratype_ = 1;
      this->enFisheyeCorrection = false;
    }

    this->sync_ = new message_filters::Synchronizer<ImgPoseSyncPolicy>(ImgPoseSyncPolicy(10), img_sub_, pose_sub_);
    this->sync_->registerCallback(boost::bind(&FisheyeCorrector::imgposeSubscriberCb, this, _1, _2));

    this->statusMtx.lock();
    this->statusFlag = FISHEYECORR_STATE_READY;
    std::bitset<8> tmp(this->statusFlag);
    this->statusMtx.unlock();
    //std::cout << "Status= " << tmp << std::endl << std::flush;
  }
  else if ((uint8_t)req.cmd == 2)   {
    this->statusMtx.lock();
    this->statusFlag = FISHEYECORR_STATE_READY;
    std::bitset<8> tmp(this->statusFlag);
    this->statusMtx.unlock();
    //std::cout << "Status= " << tmp << std::endl << std::flush; 
  } else if ((uint8_t)req.cmd == 3)   {
    this->statusMtx.lock();
    this->img_sub_.unsubscribe();
    this->pose_sub_.unsubscribe();
    this->statusFlag = FISHEYECORR_STATE_BUSY;
    std::bitset<8> tmp(this->statusFlag);
    this->statusMtx.unlock();
    //std::cout << "Status= " << tmp << std::endl << std::flush; 
    
    delete this->sync_;
    this->sync_ = 0;
  }
  resp.ret = 1;
  return true;
}

void FisheyeCorrector::fisheye_camera_info_cb(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    std::cout << "FishEye camera info has been received" << std::endl << std::flush;

    this->statusMtx.lock();
    this->statusFlag = this->statusFlag | FISHEYECORR_STATE_FISHEYE_CAMERAINFO;
    std::bitset<8> tmp(this->statusFlag);
    this->statusMtx.unlock();

    fisheye_camera_info_ = *msg;
    std::cout << "Status= " << tmp << std::endl << std::flush;
    fisheye_camera_info_sub.shutdown();
}

void FisheyeCorrector::pt_camera_info_cb(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    std::cout << "PanTilt camera info has been received" << std::endl << std::flush;

    this->statusMtx.lock();
    this->statusFlag = this->statusFlag | FISHEYECORR_STATE_PANTILT_CAMERAINFO;
    std::bitset<8> tmp(this->statusFlag);
    this->statusMtx.unlock();

    pantilt_camera_info_ = *msg;
    
    std::cout << "Status= " << tmp << std::endl << std::flush;
    pt_camera_info_sub.shutdown();
}

void FisheyeCorrector::gpsSubscriberCb(const nav_msgs::OdometryConstPtr& msg)
{
  gpspose_.header = msg->header;
  gpspose_.pose = msg->pose;
}

void FisheyeCorrector::imgposeSubscriberCb(const sensor_msgs::Image::ConstPtr &img, const nav_msgs::OdometryConstPtr &odom)
{
  // std::cout << "Synchronized msg imgtime= " << img->header.stamp << " odomtime= " << odom->header.stamp << std::endl
  //           << std::flush;

  bool cvBusy = true;

  this->statusMtx.lock();
  if (this->statusFlag == FISHEYECORR_STATE_READY)
  {
    cvBusy = false;
  }
  this->statusMtx.unlock();

  if (cvBusy)
  {
    this->skippedImgCnt++;
  }
  else
  {

    if (this->skippedImgCnt > 0)
    {
      std::cout << this->skippedImgCnt << " images skipped" << std::endl;
    }
    this->skippedImgCnt = 0;


    {
      //boost::unique_lock<boost::mutex> lock(this->newImgMtx);
      this->newImgMtx.lock();
      this->cvPtr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
      this->out_msg_.scan.ranges.clear();
      this->out_msg_.scan.intensities.clear();

      this->out_msg_.pose = *odom;
      if (this->cameratype_ == 0)
      {
        this->out_msg_.cinfo = fisheye_camera_info_;
        this->out_msg_.imgType = 0;
      }
      else{
        this->out_msg_.cinfo = pantilt_camera_info_;
        this->out_msg_.imgType = 1;
      }
      this->out_msg_.header.frame_id = img->header.frame_id;
      this->newImage_ = true;
      this->newImgMtx.unlock();
      this->newImgCondition_.notify_all();
    }

  }
}

void FisheyeCorrector::imageSubscriberCb(const sensor_msgs::Image::ConstPtr &msg)
{
  bool cvBusy;

  this->statusMtx.lock();
  if (this->statusFlag == FISHEYECORR_STATE_READY)
  {
    cvBusy = false;
  }
  this->statusMtx.unlock();

  if (cvBusy)
  {
    this->skippedImgCnt++;
  }
  else
  {
    this->newImgMtx.lock();
    this->cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    this->newImage_ = true;
    this->newImgMtx.unlock();

    std::cout << this->skippedImgCnt << " images skipped" << std::endl;
    this->skippedImgCnt = 0;
  }
}

void FisheyeCorrector::cvProcess()
{
  bool newImage;
  cv::Mat src, rectifiedDst, mapXP, mapYP;
  CvMat mapx, mapy;
  float sf = 4;

  mapXP = cv::Mat(this->imgH, this->imgW, CV_32FC1);
  mapYP = cv::Mat(this->imgH, this->imgW, CV_32FC1);

  mapx = mapXP;
  mapy = mapYP;

  //Opencv Initialization config
  // static boost::once_flag cv_thread_flag = BOOST_ONCE_INIT;
  // boost::call_once(cv_thread_flag, &cv::startWindowThread);
  // cv::namedWindow("FisheyeImage", CV_WINDOW_NORMAL);

  create_perspecive_undistortion_LUT(&mapx, &mapy, &(this->ocamModel), sf);


  while (true)
  {
    {
      boost::unique_lock<boost::mutex> lock(this->newImgMtx);
      //this->newImgMtx.lock();
      while (!this->newImage_) {
        this->newImgCondition_.wait(lock);
      }
      //this->newImgMtx.unlock();
      this->statusMtx.lock();
      this->statusFlag = FISHEYECORR_STATE_BUSY;
      this->statusMtx.unlock();
    }


    //ROS_INFO("Notification recieved");

    if(!this->exitflag_)
    {
      /*
        1. Set flags
        2. Rectify image
        3. Publish image
        4. Set flags
      */
      //std::cout << "Enter 1" << "\n";
      
      this->newImage_ = false;
      src = this->cvPtr->image;


#if 0
      cv::remap(src, rectifiedDst, mapXP, mapYP, cv::INTER_LINEAR);

      sensor_msgs::ImagePtr pubImage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rectifiedDst).toImageMsg();
      pubImage->header.stamp = ros::Time::now();

      this->imagePublisher.publish(pubImage);
#else
      // cv::imshow ( "FisheyeImage",src);
      // cv::waitKey(1);

      //Step 1: Extract edges - Run the Canny filter.
      //ROS_INFO("Processing started");

      cv::Mat gdst, dst, src_gray;

      // cvtColor( src, src_gray, cv::COLOR_RGB2GRAY );
      // cv::GaussianBlur( src_gray, gdst, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );
      cv::GaussianBlur(src, gdst, cv::Size(9, 9), 10.0);
      // cv::adaptiveBilateralFilter(src, gdst, cv::Size(9, 9), 15.0);
      cv::Canny(gdst, dst, 15, 50, 3);
      // cv::Canny(gdst, dst, 50, 200, 3, false);
      /// Remove noise by blurring with a Gaussian filter
      //cv::GaussianBlur( src, gdst, cv::Size(9, 9), 10.0, 0, cv::BORDER_DEFAULT );
      //cvtColor( src, gray, CV_RGB2GRAY );
 
      /// Apply Laplace function
      //cv::Laplacian( gdst, ldst, CV_16S, 3, 1, 0, cv::BORDER_DEFAULT );
      //cv::convertScaleAbs( ldst, dst );

      if (enFisheyeCorrection)
      {
        //Step 2: Now we need to apply the distortion correction on the edge image.
        cv::remap(dst, rectifiedDst, mapXP, mapYP, cv::INTER_LINEAR);
      }
      else
      {
        rectifiedDst = dst;
      }

      // cv::imshow("FisheyeImage", rectifiedDst);
      // cv::waitKey(1);
      int count =0 ;

      //Step 3: Convert the extracted edge pixels to a laser scan.
      for (int row = 0; row < rectifiedDst.rows; row++)
      {
        for (int col = 0; col < rectifiedDst.cols; col++)
        {
          //printf("Pixel value: %d\n", rectifiedDst.at<uchar>(cv::Point(col, row)));
          if (rectifiedDst.at<uchar>(cv::Point(col, row))>240)
          { 
            count++;
            if (count % 2 == 0)
               continue;

            double range = sqrt(pow(row - rectifiedDst.rows / 2, 2) + pow(col - rectifiedDst.cols / 2, 2)) * this->MAP_RESOLUTION;
            //if (range >200* this->MAP_RESOLUTION)
            //  continue;
            out_msg_.scan.ranges.push_back(range);

            double bearing = atan2(row - rectifiedDst.rows / 2, col - rectifiedDst.cols / 2);
            out_msg_.scan.intensities.push_back(bearing);
            
          }
        }
      }

      //Fill in the header and scan details and publish the topic.
      out_msg_.header.stamp = out_msg_.pose.header.stamp;
      out_msg_.header.seq = ++msg_cnt_;

      out_msg_.gps = gpspose_;
      //Copy the generated laser scan points

      //std::cout << "Publishing msg imgtime= " << out_msg_.header.stamp << std::endl
      //          << std::flush;
      //Publish the frame into the ROS space
      this->scanPublisher.publish(out_msg_);
#endif
    }
    boost::this_thread::interruption_point();
  }
}
