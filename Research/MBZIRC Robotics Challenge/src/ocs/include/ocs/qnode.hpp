/**
 * @file /include/ocs/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ocs_QNODE_HPP_
#define ocs_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/String.h>
#endif

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ocs {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

	/*
		OCS Start
	*/
	void gazeboModelStatesCb(const gazebo_msgs::ModelStates::ConstPtr&);
	void writePoseLog(std::string model, geometry_msgs::Pose pose);
	void logMsgCb(const std_msgs::String::ConstPtr&);
	/*
		OCS End
	*/


Q_SIGNALS:
	void loggingUpdated();
  void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	QStringListModel logging_model;

	/*
		OCS Start
	*/
	ros::Subscriber gazeboModelSubscriber;
	ros::Subscriber logMsgSubscriber;
	/*
		OCS End
	*/
};

}  // namespace ocs

#endif /* ocs_QNODE_HPP_ */
