#ifndef map_marker_QNODE_HPP_
#define map_marker_QNODE_HPP_

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"

namespace map_marker {

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
	geometry_msgs::Pose GetRobotPosition();
	void MoveRobotToPose(geometry_msgs::Pose pos);

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

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher marker_publisher;
    QStringListModel logging_model;
    geometry_msgs::Pose pose;
};

}

#endif
