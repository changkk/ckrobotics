

// ROS dependencies
#include <ros/ros.h>

// Project dependencies

#include "perception/quick_drop.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;
using namespace std;

//test_fisheye class Constructor
test_challenge1::test_challenge1() : m_state(CHAL1_INIT), exitflag_(false), laser_flag(true),
                                     start_process(false), armed(false), guided(false), distance_totruck(0.0)
{
  std::cout << "test_challenge1 constructor" << std::endl << std::flush;

  monitor_pose.x = 0.0;
  monitor_pose.y = 0.0;
  monitor_pose.z = 2.0;

  disx_err=1.5;
  disy_err=1.5;
  disz_err=1.5;

}

void test_challenge1::start(ros::NodeHandle nh)
{
  std::cout << "test_challenge1 start" << std::endl << std::flush;
  //Store the ROS handle
  nh_ = &nh;

  //Subscribers
  start_sub = nh.subscribe("/start_challenge1", 1, &test_challenge1::start_callback, this);
  pose_sub = nh.subscribe("/hexacopter/localizer/global_odom", 1, &test_challenge1::chatterCallback, this);
  laser_sub = nh.subscribe("/rrbot/laser/scan", 1, &test_challenge1::laser_callback, this); 
  mavros_sub = nh.subscribe("/hexacopter/mavros/state", 1, &test_challenge1::MavrosStateCallback, this);
  //lidar_sub = nh.subscribe("hexacopter/rangefinder", 1, &test_challenge1::lidar_callback, this);

  //Publishers
  landing_pub = nh.advertise<std_msgs::Empty>("/hexacopter/uav_control/land", 1);
  takeoff_pub = nh.advertise<std_msgs::Empty>("/hexacopter/uav_control/takeoff", 1);
  point_pub = nh.advertise<geometry_msgs::Pose>("/hexacopter/uav_control/waypoint", 1);
  vel_pub = nh.advertise<geometry_msgs::Twist>("/hexacopter/uav_control/velocity", 1);

  //Thread1 = new boost::thread(boost::bind(&test_challenge1::cvProcess1, this));
  Thread1 = new boost::thread(boost::bind(&test_challenge1::cvProcess2, this));

  ros::spin();
  {
    processMtx1.lock();
    exitflag_ = true;
    processMtx1.unlock();
  }
}

test_challenge1::~test_challenge1()
{
  std::cout << "Destructor on test_challenge1 called..." << std::endl << std::flush;
  Thread1->interrupt();
  Thread1->join();
  delete Thread1;
}


void test_challenge1::chatterCallback(const nav_msgs::Odometry& msg)
{    
   //d::cout << "test_challenge1::chatterCallback" << std::endl << std::flush;
   geometry_msgs::Point coord = msg.pose.pose.position;
   
   {
     //Get the current poses
     processMtx1.lock();
	   uav_pose.x = coord.x;
	   uav_pose.y = coord.y;
	   uav_pose.z = coord.z;
     disx_err = abs(uav_pose.x-monitor_pose.x);
     disy_err = abs(uav_pose.y-monitor_pose.y);
     disz_err = abs(uav_pose.z-monitor_pose.z);
     processMtx1.unlock();
     //std::cout << "uav_pose.x= " << uav_pose.x << ", disx_err= " << disx_err << std::endl << std::flush;
   }

}

/* We start the state machine change on start_challenge1 message */
void test_challenge1::start_callback(const std_msgs::Empty& msg)
{
  
  processMtx1.lock();
  start_process=true;
  processMtx1.unlock();
  std::cout << "start_callback start_process= " << start_process << std::endl << std::flush;
  return;
}

void test_challenge1::MavrosStateCallback(const mavros_msgs::State msg)
{
  processMtx1.lock();
	armed = msg.armed;
	guided = msg.guided;
	flightmode = msg.mode;
  processMtx1.unlock();
}

/* Main state machine: written assuming that we cannot run threads as we haven't tested that */
void test_challenge1::cvProcess2()
{
  bool slanding_, exit_, armed_;

  usleep(10000);

  std::cout << "test_challenge1::cvProcess2 thread" << std::endl << std::flush;
  
  tStart = ros::Time::now();
  
  while(true)
  {
    //Check if we are ready to start
    processMtx1.lock();
    slanding_ = start_process;
    exit_ = exitflag_;
    armed_ = armed;
    processMtx1.unlock();
    while ((!slanding_) && (!exit_) && (!armed_))
    {
      //std::cout << "1 startLanding= " << startLanding << ", exitflag_= " << exitflag_ << std::endl << std::flush;
           
      //sleep for 100ms and try
      usleep(10000);
      //std::cout << "woke up\n";
      processMtx1.lock();
      slanding_ = start_process;
      exit_ = exitflag_;
      armed_ = armed;
      processMtx1.unlock();
      //std::cout << "2 startLanding= " << startLanding << ", exitflag_= " << exitflag_ << std::endl << std::flush;
    }
    m_state=CHAL1_MONIPREPARE;

    
    if(exitflag_)
    {
      break;
    }

    std::cout << "Preparing m_state= " << m_state << std::endl << std::flush;
    
    //Now start moving to the monitoring target
    if(m_state==CHAL1_MONIPREPARE)
    {
      cout<<"takeoff Start 1 (Top right to bottom left)"<<endl;

	    int ini_alt = uav_pose.z; //d_z;
	    int count = 0;

      //Issue the take off command
		  while(uav_pose.z < (ini_alt + 1.5))
		  {
			  count++;
			  if (!armed)
		    {
				  takeoff_pub.publish(std_msgs::Empty());
			  }
			  ros::Duration(10).sleep();		
		  }

	    cout<<"go to start point"<<endl;
	    geometry_msgs::Pose c;
			c.position.x=monitor_pose.x;
			c.position.y=monitor_pose.y;
			c.position.z=monitor_pose.z;
     	point_pub.publish(c);
			ros::Duration(8).sleep();	
      
      std::cout << "published x= " << c.position.x << ", y= " << c.position.y << ", z= " << c.position.z << std::endl << std::flush;
    }
    else
    {
      std::cout << "continue" << std::endl << std::flush;
      continue;
    }

    if(exitflag_)
    {
      break;
    }
    
    //Now that we've asked the pixhawk to move to the centre of the areana. Let's check that
    {
         processMtx1.lock();
         while (((disx_err > 1.0) || (disy_err > 1.0) || (disz_err > 0.5)) && (!exitflag_))
         {

           processMtx1.unlock();
           //sleep for 100ms and try
           usleep(5000);
           processMtx1.lock();
           //std::cout << "Woke up m_state= " << m_state << ", disx_err= " << disx_err << ", disy_err= " << disy_err << std::endl << std::flush;
         }
         m_state=CHAL1_MONIREADY;
         processMtx1.unlock();
    } 
 
    if(exitflag_)
    {
      break;
    }

    std::cout << "Monitoring m_state= " << m_state << ", x_err= " << disx_err << ", y_err= " << disy_err << ", z= " << uav_pose.z << std::endl << std::flush;

    {
         processMtx1.lock();
         while ((m_state < CHAL1_TRUCKDETECTED_STAGE1) && (!exitflag_))
         {
           processMtx1.unlock();
           //sleep for 10ms and try
           usleep(5000);
           processMtx1.lock();
           //std::cout << "Woke up m_state= " << m_state << ", disx_err= " << disx_err << ", disy_err= " << disy_err << std::endl << std::flush;
         }
         processMtx1.unlock();
    } 

    if(exitflag_)
    {
      break;
    }

    std::cout << "Detect-stage 1 m_state= " << m_state << ", x_err= " << disx_err << ", y_err= " << disy_err << ", z= " << uav_pose.z << std::endl << std::flush;

    //we can use distance_totruck to calculate any timing
    ros::Duration(2).sleep();	
    processMtx1.lock();
    laser_flag = true;
    processMtx1.unlock();

    
    {
         processMtx1.lock();
         while ((m_state < CHAL1_TRUCKDETECTED_STAGE2) && (!exitflag_))
         {
           processMtx1.unlock();
           //sleep for 10ms and try
           usleep(5000);
           processMtx1.lock();
           //std::cout << "Woke up m_state= " << m_state << ", disx_err= " << disx_err << ", disy_err= " << disy_err << std::endl << std::flush;
         }
         processMtx1.unlock();
    } 
    
    if(exitflag_)
    {
      break;
    }
    
    std::cout << "Detect-stage 2 m_state= " << m_state << ", x_err= " << disx_err << ", y_err= " << disy_err << std::endl << std::flush;


    //Now send the command to land

	geometry_msgs::Twist drop;
	drop.linear.x = 0;
	drop.linear.y = 0;
	drop.linear.z = -10;
	vel_pub.publish(drop);

    while(!exitflag_)
    {
      usleep(200000);
    }
    boost::this_thread::interruption_point();
  }
}

void test_challenge1::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) // For Simulation
{
  challenge1State tmpstate;

  processMtx1.lock();
  tmpstate = m_state;
  processMtx1.unlock();

  if(!laser_flag)
  {
    return;
  }

  if ((tmpstate < CHAL1_MONIREADY)||(tmpstate > CHAL1_TRUCKDETECTED_STAGE2))
  {
    return;
  }

  std::vector<float> laser = msg->ranges;

  int size_laser = laser.size();
  float min_range = 2000.0, max_range=-2.0;
  int index_min, index_max;
  for (int i=0;i<size_laser;i++)
  {
    if (laser[i] < min_range)
    {
      min_range = laser[i];
      index_min = i;
    }
    if (laser[i] > max_range)
    {
      max_range = laser[i];
      index_max = i;
    }
  }

  float diff = uav_pose.z - min_range;
  //std::cout << "altitude= " << uav_pose.z << "min_range= " << min_range << ", diff= " << diff << std::endl << std::flush;
  if(tmpstate == CHAL1_MONIREADY)
  {
    if(diff > 1.0)
    {
      processMtx1.lock();
      m_state = CHAL1_TRUCKDETECTED_STAGE1;
      distance_totruck = min_range;
      laser_flag = false;
      processMtx1.unlock();
      std::cout << "altitude= " << uav_pose.z << "min_range= " << min_range << ", diff= " << diff << std::endl << std::flush;
    }
  }
  else if(tmpstate == CHAL1_TRUCKDETECTED_STAGE1)
  {
    if(diff > 1.0)
    {
      processMtx1.lock();
      m_state = CHAL1_TRUCKDETECTED_STAGE2;
      distance_totruck = min_range;
      processMtx1.unlock();
      std::cout << "altitude= " << uav_pose.z << "min_range= " << min_range << ", diff= " << diff << std::endl << std::flush;
    }
  }
}

void test_challenge1::lidar_callback(const std_msgs::Float64::ConstPtr& msg)
{
  challenge1State tmpstate;

  processMtx1.lock();
  tmpstate = m_state;
  processMtx1.unlock();

  if(!laser_flag)
  {
    return;
  }

  if ((tmpstate < CHAL1_MONIREADY)||(tmpstate > CHAL1_TRUCKDETECTED_STAGE2))
  {
    return;
  }

  float range_value = msg->data;
  if (range_value < 0.05)
  {
    return;
  }
  
  float diff = uav_pose.z - range_value;

  if(tmpstate == CHAL1_MONIREADY)
  {
    if(diff > 1.0)
    {
      processMtx1.lock();
      m_state = CHAL1_TRUCKDETECTED_STAGE1;
      distance_totruck = range_value;
      laser_flag = false;
      processMtx1.unlock();
      std::cout << "altitude= " << uav_pose.z << "min_range= " << range_value << ", diff= " << diff << std::endl << std::flush;
    }
  }
  else if(tmpstate == CHAL1_TRUCKDETECTED_STAGE1)
  {
    if(diff > 1.0)
    {
      processMtx1.lock();
      m_state = CHAL1_TRUCKDETECTED_STAGE2;
      distance_totruck = range_value;
      processMtx1.unlock();
      std::cout << "altitude= " << uav_pose.z << "min_range= " << range_value << ", diff= " << diff << std::endl << std::flush;
    }
  } 
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "test_fisheye");
  ros::NodeHandle nh;
  ros::NodeHandle privatenh("~");

  std::cout << "Instantiating fisheyeCorrector object" << std::endl << std::flush;
  test_challenge1 challenge1;
  challenge1.start(nh);

  return 0;
}
