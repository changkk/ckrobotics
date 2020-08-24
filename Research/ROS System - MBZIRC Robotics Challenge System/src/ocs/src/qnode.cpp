/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/ocs/qnode.hpp"
#include <boost/lexical_cast.hpp>

static const std::string GAZEBO_MODEL_TOPIC = "/gazebo/model_states";
static const std::string OCS_LOG_MESSAGE_TOPIC = "/ocs/log_message";
static const std::string HEX_0_MODEL = "hexacoptor";
static const std::string TRUCK_MODEL = "truck";

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ocs {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"ocs");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	
	// Add your ros communications here.

	// Gazebo model subscription
    gazeboModelSubscriber = n.subscribe(GAZEBO_MODEL_TOPIC, 1, &QNode::gazeboModelStatesCb, this);
	
	// Log message subscription
	logMsgSubscriber = n.subscribe(OCS_LOG_MESSAGE_TOPIC, 100, &QNode::logMsgCb, this);

    
	std::cout << "registered cb. starting..." << std::endl;
    start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"ocs");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.

    start();
	return true;
}

void QNode::run() {
    ros::spin();
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::gazeboModelStatesCb(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    // std::cout << "message received" << std::endl;
    for (int i=0; i<msg->name.size(); i++) {
        if (HEX_0_MODEL == msg->name[i]) {
            writePoseLog(HEX_0_MODEL, msg->pose[i]);
        } else if (TRUCK_MODEL == msg->name[i]) {
            writePoseLog(TRUCK_MODEL, msg->pose[i]);
        }
    }
}

void QNode::writePoseLog(std::string model, geometry_msgs::Pose pose) {
    ros::Rate loop_rate(1);
    std::string logMsg = model + " : <";
    logMsg += boost::lexical_cast<std::string>(pose.position.x) + ",";
    logMsg += boost::lexical_cast<std::string>(pose.position.y) + ",";
    logMsg += boost::lexical_cast<std::string>(pose.position.z) + ",";
    logMsg += ">";

    log(Info, logMsg);
    loop_rate.sleep();
}

void QNode::logMsgCb(const std_msgs::String::ConstPtr& msg) {
	log(Info, msg->data);
}

}  // namespace ocs
