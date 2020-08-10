/*
  Node that publishes static images for debug purposes
*/

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

Mat tmplt(int);

int main(int argc, char** argv) {
  ros::init(argc, argv, "debug_node");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<sensor_msgs::Image>(
    "/localizer/tf_image",
    1,
    false
  );

  Mat tmpl;
  sensor_msgs::ImagePtr pubImage;

  for (int i=120; i<950; i++) {
    tmpl = tmplt(i);
    if (tmpl.empty()) {
      continue;
    }

    pubImage = cv_bridge::CvImage(std_msgs::Header(), "mono8", tmpl).toImageMsg();
    pubImage->header.stamp = ros::Time::now();

    pub.publish(pubImage);

    cout << "sleeping for 2 secs" << endl;
    ros::Duration(2).sleep();

  }
  
  return 0;
}

Mat tmplt ( int index )
{
//     float rz_factor =  .5;

  float rz_factor = .25 * .75;

  stringstream fname;

//     fname << "LaserMatching/Laser-im/image_" << index << ".png" ;
  fname << "datasets/alt-30-quad/ardrone-bottom-image_raw-Image_" 
    << setfill ( '0' ) << setw ( 4 ) << index << ".png" ;

  Mat src = imread ( fname.str(), 0 );

  if ( src.empty() )
  {

    cout << "can not open " << fname.str() << endl;
    //   return NULL;

    return src;
  }


  imshow ( "input",src );

  Mat dst, cdst, gdst, rsrc;

  resize ( src, rsrc, Size ( round ( rz_factor*src.cols ),round ( rz_factor*src.rows ) ) );

//     dst=rsrc;

// Add a gaussian blur to reduce noise
  GaussianBlur ( rsrc, gdst, Size ( 3, 3 ), 0.95 );

  Canny ( gdst, dst, 15, 50, 3 );

  return dst;

}
