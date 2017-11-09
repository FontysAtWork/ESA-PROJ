#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "../include/map_marker/qnode.hpp"

namespace map_marker {

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
	ros::init(init_argc,init_argv,"map_marker");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start();
	ros::NodeHandle n;
	marker_publisher = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 100);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"map_marker");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start();
	ros::NodeHandle n;
	marker_publisher = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 100);
	start();
	return true;
}

void QNode::run() {
	ros::Rate rate(1);
	int count = 0;

	tf::TransformListener tflistener;

	while (ros::ok()) {
		tf::StampedTransform transform;
		try {
			geometry_msgs::Quaternion q;
			tflistener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
			pose.position.x = transform.getOrigin().x();
			pose.position.y = transform.getOrigin().y();
			pose.position.z = transform.getOrigin().z();
			tf::quaternionTFToMsg(transform.getRotation(), q);
			pose.orientation = q;

			std::cout << "x: " << transform.getOrigin().x() << ", y:" << transform.getOrigin().y() << ", z: "<< transform.getOrigin().y() << std::endl;
			std::cout << "x: " << q.x << ", y:" << q.y << ", z: " << q.z << ", w: " << q.w << std::endl << std::endl;

		}
		catch (tf::TransformException ex){
		  //ROS_ERROR("%s",ex.what());
		  ros::Duration(1.0).sleep();
		}
		ros::spinOnce();
		rate.sleep();
	}
	
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

geometry_msgs::Pose QNode::GetRobotPosition() {
	// TODO: IMPLEMENTATION

	geometry_msgs::Pose p;
	p.position.x = 5;
	p.position.y = 8;
	p.position.z = 0;
	p.orientation.z = 1;
	return p;
}

void QNode::MoveRobotToPose(geometry_msgs::Pose pos) {
	// TODO: IMPLEMENTATION

	ROS_INFO("xpos: %f", pos.position.x);

	geometry_msgs::PoseStamped p;
	p.pose = pos;

	marker_publisher.publish(p);
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

}  // namespace map_marker
