#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <actionlib_msgs/GoalID.h>
#include "qnode.hpp"

namespace map_marker {

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{
	}

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
	pubPose = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 100);
	pubVelCmd = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
	pubActionLibCancel = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 100);
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
	pubPose = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 100);
	pubVelCmd = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
	pubActionLibCancel = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 100);
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
			tflistener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
			pose.position.x = transform.getOrigin().x();
			pose.position.y = transform.getOrigin().y();
			pose.position.z = transform.getOrigin().z();
			tf::quaternionTFToMsg(transform.getRotation(), q);
			pose.orientation = q;
		}
		catch (tf::TransformException ex){
		  ROS_ERROR("%s",ex.what());
		  ros::Duration(1.0).sleep();
		}
		ros::spinOnce();
		rate.sleep();
	}

	ROS_INFO("Ros shutdown, proceeding to close the gui.");
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

geometry_msgs::Pose QNode::GetRobotPosition() {
	return pose;
}

void QNode::Panic() {
	actionlib_msgs::GoalID kill_msg;
    geometry_msgs::Twist vel_msg;        

	pubActionLibCancel.publish(kill_msg);
	pubVelCmd.publish(vel_msg);
}

void QNode::MoveRobotToPose(geometry_msgs::Pose pos) {
	ROS_INFO("Moving robot to x:%f, y: %f", pos.position.x, pos.position.y);
	geometry_msgs::PoseStamped p;
	p.header.stamp = ros::Time::now();
	p.header.frame_id = "map";
	p.pose = pos;

	pubPose.publish(p);
}

}  // namespace map_marker
