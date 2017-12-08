#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/Bool.h>
#include <sstream>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "Qnode.hpp"
#include "keepout_map_server_msg/Line.h"

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

bool QNode::Init() {
	ros::init(init_argc,init_argv,"map_marker");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start();
	ros::NodeHandle n;
	subMap = n.subscribe("keepout/map", 100, &QNode::keepoutMapCallback, this);
	pubEmergency = n.advertise<std_msgs::Bool>("emergency_stop", 100);
	pubPose = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 100);
	pubNoGoLine = n.advertise<keepout_map_server_msg::Line>("/keepout/drawline", 100);
	start();
	return true;
}

bool QNode::Init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__ip"] = host_url;
	ros::init(remappings,"map_marker");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start();
	ros::NodeHandle n;
	subMap = n.subscribe("keepout/map", 100, &QNode::keepoutMapCallback, this);
	pubEmergency = n.advertise<std_msgs::Bool>("emergency_stop", 100);
	pubPose = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 100);
	pubNoGoLine = n.advertise<keepout_map_server_msg::Line>("/keepout/drawline", 100);
	start();
	return true;
}

void QNode::run() {
	ros::Rate rate(10);
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
			tf::Quaternion tfq = transform.getRotation();
			tfq.normalize();
			tf::quaternionTFToMsg(tfq, q);
			pose.orientation = q;
			Q_EMIT QNode::RobotPosUpdated();
		}
		catch (tf::TransformException ex){
		  ROS_ERROR("%s",ex.what());
		  ros::Duration(1.0).sleep();
		}
		ros::spinOnce();
		rate.sleep();
	}

	ROS_INFO("Ros shutdown, proceeding to close the gui.");
	Q_EMIT RosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

geometry_msgs::Pose QNode::GetRobotPosition() {
	return pose;
}

void QNode::Panic() {
    std_msgs::Bool emergency_msg; 
    emergency_msg.data = true;      
	pubEmergency.publish(emergency_msg);
}

void QNode::MoveRobotToPose(geometry_msgs::Pose pos) {
	ROS_INFO("Moving robot to x:%f, y: %f", pos.position.x, pos.position.y);
	geometry_msgs::PoseStamped p;
	p.header.stamp = ros::Time::now();
	p.header.frame_id = "map";
	p.pose = pos;

	pubPose.publish(p);
}

void QNode::DrawLine(const int x1, const int y1, const int x2, const int y2)
{
	ROS_INFO("Sending line");

	keepout_map_server_msg::Line line;
	line.x1 = x1;
	line.x2 = x2;
	line.y1 = y1;
	line.y2 = y2;
	line.gradient = 100;

	pubNoGoLine.publish(line);

}


void QNode::keepoutMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid)
{
	keepoutMap.data = grid->data;
	keepoutMap.info = grid->info;
	keepoutMap.header = grid->header;

}

nav_msgs::OccupancyGrid QNode::GetKeepOutMap()
{
	return keepoutMap;

} 


}  // namespace map_marker
