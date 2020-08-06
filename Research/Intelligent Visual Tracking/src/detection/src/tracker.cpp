#include <string.h>
#include <iostream>
#include "ros/ros.h"
#include <sstream>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Point32.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <opencv2/opencv.hpp>
#include "opencv2/core/utility.hpp"
#include "opencv2/core.hpp"
#include <opencv2/core/ocl.hpp>
#include "opencv2/core/core.hpp"
#include <opencv2//highgui/highgui.hpp>
#include "opencv2/tracking.hpp"
#include "opencv2/tracking/tracking.hpp"
#include "opencv2/tracking/tracker.hpp"
#include <opencv2/video/tracking.hpp>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <cstring>
#include <fstream>
// #include "samples_utility.hpp"


using namespace std;
// using namespace tf2;
using namespace cv;

cv_bridge::CvImagePtr cv_ptr;
//Size size(1280, 960);
float scl = 0.25;
//int w = round(1280*scl), l = round(960*scl);
int w = 320, l = 240;
Size size(w, l);
Mat frame  = Mat::zeros(size, CV_8UC3);
Mat frame_n = Mat::zeros(size, CV_8UC3);
bool central_detection = false;
int no_detection = 0;
geometry_msgs::Point32 box_pos, img_size, box_size;
geometry_msgs::Point32 tracker_box_pos, tracker_box_size;


class ImageConverter
{



  public:

    ros::NodeHandle n;
    image_transport::Publisher image_pub_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;


    ImageConverter()
        : it_(n)
    {
        // Subscribe to input video feed and publish output video feed
        cout<< "what"<<endl;
        image_transport::TransportHints hints("compressed", ros::TransportHints());
        image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this, hints);
        image_pub_ = it_.advertise("/tracker_image", 1);
    }

    ~ImageConverter()
    {
        // destroyWindow(OPENCV_WINDOW);
    }




    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            frame = cv_ptr->image;
            frame.copyTo(frame_n);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        frame = cv_ptr->image;
	resize(frame, frame, size);	

        frame.copyTo(frame_n);
    }

};

    //Callback for peripheral camera detection
    void objdet_cb(const geometry_msgs::PolygonStamped msg)
    {
        // Update detection box
        box_pos = msg.polygon.points[0]; //Check if this was position of top left corner or center of box
        img_size = msg.polygon.points[1];
        box_size = msg.polygon.points[2];
        cout << box_pos.x << "  " << box_pos.y << endl;

	img_size.x = scl*img_size.x;
	img_size.y = scl*img_size.y;
	box_size.x = scl*box_size.x;
	box_size.y = scl*box_size.y;
	box_pos.x = scl*box_pos.x;
	box_pos.y = scl*box_pos.y;

        // If a message shows up here, set detection flag high
        central_detection = true;
        no_detection = 0;

        // std_msgs::Float64 tmp_msg;

        // // Set setpoint to half of image widht/height (center)
        // tmp_msg.data = img_size.x/2;
        // yaw_sp_pub.publish(tmp_msg);
        // tmp_msg.data = img_size.y/2;
        // pitch_sp_pub.publish(tmp_msg);

        // // Set state to filter input
        // tmp_msg.data = box_pos.x;
        // yaw_st_pub.publish(tmp_msg);
        // tmp_msg.data = box_pos.y;
        // pitch_st_pub.publish(tmp_msg);

        // //Publish twiststamped message to gimbal speed topic (pitch_ce/yaw_ce normalized)
        // usleep(15000);
        // gm_speed_msg.header.stamp = ros::Time::now();
        // gm_speed_msg.twist.angular.x = 0;
        // gm_speed_msg.twist.angular.y = pitch_ce*max_ang_spd;
        // gm_speed_msg.twist.angular.z = yaw_ce*max_ang_spd;
    }

    int main(int argc, char **argv)
    {
        ros::init(argc, argv, "tracker_node");

        // ros::NodeHandle n;

        ImageConverter ic;

        ros::Subscriber central_det = ic.n.subscribe("yolo_detection_box", 1, objdet_cb);
        ros::Publisher tracker_box_pub = ic.n.advertise<geometry_msgs::PolygonStamped>("tracker_box", 1);

        // ros::Subscriber central_det = n.subscribe("yolo_detection_box", 1, objdet_cb);
        // ros::Publisher tracker_info = n.advertise<geometry_msgs::PolygonStamped>("tracker_info", 1);
        // ros::Publisher image_pub_ = n.advertise<sensor_msgs::Image>("/tracker_image", 1);

        ros::Rate loop_rate(30);
        
        ///////////////////////////////////////////////////////////////////////
        
        bool tracking = false;
        Rect2d roi_detect, roi_tracked;
        int timer;

        // TrackerKCF::Params param;
        // param.desc_pca = TrackerKCF::GRAY | TrackerKCF::CN;
        // param.desc_npca = 0;
        // param.compress_feature = true;
        // param.compressed_size = 2;

        // cout << "Foo2" << endl;

        Ptr<TrackerKCF> tracker = TrackerKCF::create();
        // Ptr<TrackerKCF> tracker = TrackerKCF::create();
        Ptr<TrackerKCF> tracker = TrackerKCF::create();
        // Ptr<TrackerCSRT> tracker = TrackerCSRT::create();
        // Ptr<Tracker> tracker;

        // cout << "Foo2" << endl;
        
        // tracker = TrackerCSRT::create();
        // tracker = TrackerMIL::create();
        // Ptr<TrackerCSRT>

            // tracker = TrackerBoosting::create();
            // tracker = TrackerMIL::create();
            // tracker = TrackerKCF::create();
            // tracker = TrackerTLD::create();
            // tracker = TrackerMedianFlow::create();
            // tracker = TrackerGOTURN::create();
            // tracker = TrackerMOSSE::create();
            // tracker = TrackerCSRT::create();


        // tracker->setFeatureExtractor(sobelExtractor);

	    cout << w << "	" << l << endl;

        while (ros::ok())
        {
            
            cout << "Status" << endl << "tracking = " << tracking << "   central_detection = " << central_detection << endl;
            
            
            
            // timer = getTickCount();
            

            if (!tracking)
            {
                //Check if object detected and update tracker init appropriately
                if (central_detection && !tracking)
                {
                    cout << "Foo2" << endl;
                    roi_detect.x = round(box_pos.x - box_size.x/2);
                    roi_detect.y = round(box_pos.y - box_size.y/2);
                    roi_detect.width = box_size.x;
                    roi_detect.height = box_size.y;

		            cout << roi_detect.x << "	" << roi_detect.y << "	" << roi_detect.width << "	" << roi_detect.height << endl;

                    // Clear tracker state
                    tracker->clear();


                    // cv::Rect aaRect = cv::Rect(box_pos.x - box_size.x/2, box_pos.y - box_size.y/2, box_size.x, box_size.y);

                    // Mat mask = Mat::zeros(aaRect.size(), CV_8UC1);
                    // const int n = 4;
                    // std::vector<cv::Point> poly_points(n);
                    // //Translate x and y to rects start position
                    // int sx = aaRect.x;
                    // int sy = aaRect.y;
                    // for (int i = 0; i < n; ++i) {
                    //     poly_points[i] = Point(elements[2 * i] - sx, elements[2 * i + 1] - sy);
                    // }
                    // cv::fillConvexPoly(mask, poly_points, Scalar(1.0), 8);

                    // mask.convertTo(mask, CV_32FC1);
                    // tracker->setInitialMask(mask);

                    // tracker = TrackerCSRT::create();

                    // "tracker_algorithm can be: MIL, BOOSTING, MEDIANFLOW, TLD, KCF, GOTURN, MOSSE.\n"
                    Ptr<TrackerKCF> tracker = TrackerKCF::create();
                    // Ptr<TrackerCSRT> tracker = TrackerCSRT::create();
                    // cout << getBuildInformation() << endl;

                    // initialize the tracker
                    // imshow("Show",frame_n);
                    cout << roi_detect << endl;
                    tracker->init(frame_n, roi_detect);
                    cout << "Foo2.1" << endl;
                            
                    // Update tracker
                    tracking = tracker->update(frame_n, roi_tracked);
                    // tracking = true;
                    cout << "Foo2.2 tracking = " << tracking << endl;

                    // draw the tracked object
                    rectangle(frame_n, roi_tracked, Scalar(255, 0, 0), 2, 1);

                    // cv_bridge::CvImage out_msg;
                    // out_msg.header = cv_ptr->header; // Same timestamp and tf frame as input image
                    // out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3; // Or whatever/
                    // out_msg.image = frame_n; // Your cv::Mat

                }
            }
            else
            {
                cout << "Foo34" << endl;

                // Update tracker
                tracking = tracker->update(frame_n, roi_tracked);

                // draw the tracked object
                rectangle(frame_n, roi_tracked, Scalar(255, 0, 0), 2, 1);
                cout << "Trcker Out:" << roi_tracked.x << "  " << roi_tracked.y << endl;


                // Check offset between detection and tracking
                if (central_detection)
                {
		            roi_detect.x = round(box_pos.x - box_size.x/2);
                    roi_detect.y = round(box_pos.y - box_size.y/2);
                    roi_detect.width = box_size.x;
                    roi_detect.height = box_size.y;
                    
		            int delta_x = abs(roi_detect.x - roi_tracked.x);
                    int delta_y = abs(roi_detect.y - roi_tracked.y);

                    cout << "Delta x and y: " << delta_x << "   " << delta_y << endl;
                    
                    //If offset is greater than 4% of image dimensions, assume tracking failure. (We trust our detector more)
                    if ( (delta_x > img_size.x*0.1) || (delta_y > img_size.y*0.1) )
                    {
                        tracking = false;
                    }

                }

                tracker_box_size.x = roi_detect.width;
                tracker_box_size.y = roi_detect.height;
                tracker_box_pos.x = roi_detect.x + tracker_box_size.x/2;
                tracker_box_pos.y = roi_detect.y + tracker_box_size.y/2;

                geometry_msgs::PolygonStamped tracker_box;
                tracker_box.polygon.points.reserve(3);
                tracker_box.polygon.points.push_back(tracker_box_pos);
                tracker_box.polygon.points.push_back(img_size);
                tracker_box.polygon.points.push_back(tracker_box_size);
                tracker_box_pub.publish(tracker_box);

            }

                    sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_n).toImageMsg();
                    ic.image_pub_.publish(out_msg);


            no_detection++;
            no_detection = (no_detection < 30) ? no_detection:30;

            
            if (no_detection > 10)
            {
                central_detection = false;
            }

            // int fps = getTickFrequency() / (getTickCount() - timer);

            // stringstream s;
            // s.str("");
            // s  << "FPS = " << fps;
            // cv::putText(frame_n, s.str(), Point(10, 50), FONT_HERSHEY_SIMPLEX, 1.25, Scalar(100, 0, 255), 4);


            // imshow("tracking", frame_n);
            // waitKey(3);
            

            ros::spinOnce();
            loop_rate.sleep();
        }
        

        return 0;
    }
