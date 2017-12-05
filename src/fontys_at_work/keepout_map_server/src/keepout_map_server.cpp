#include "ros/ros.h"
#include <ros/timer.h>

#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Empty.h>
#include <keepout_map_server/Line.h>
#include <keepout_map_server/Square.h>

#include "map.hpp"
#include "map_functions.hpp"

ros::Publisher mapPublisher;
faw::Map m;

void cbClearMap(const std_msgs::Empty::ConstPtr& msg)
{
	m.clear();
}

void cbDrawLine(const keepout_map_server::Line::ConstPtr& msg)
{
	faw::mapFunctions::drawLine(m, msg->x1, msg->y1, msg->x2, msg->y2, msg->gradient);
}

void cbDrawSquare(const keepout_map_server::Square::ConstPtr& msg)
{
	faw::mapFunctions::drawSquare(m, msg->x, msg->y, msg->width, msg->height, msg->gradient);
}

void cbPublishMap(const ros::TimerEvent&)
{
	mapPublisher.publish(m.toMessage());
}

int main(int argc, char **argv)
{
	int mapWidth;
	int mapHeight;
	float mapResolution;
	float publishInterval;
	
	ros::init(argc, argv, "keepout_map_server");
	ros::NodeHandle n;
	
	ros::param::param<int>(ros::this_node::getName() + "/map_width", mapWidth, 1000);
	ros::param::param<int>(ros::this_node::getName() + "/map_height", mapHeight, 1000);
	ros::param::param<float>(ros::this_node::getName() + "/map_resolution", mapResolution, 0.01);
	ros::param::param<float>(ros::this_node::getName() + "/publish_interval", publishInterval, 1);
	
	//std::cout << mapWidth << " " << mapHeight << " " << mapResolution << " " << publishInterval << std::endl;
	
	m = faw::Map(mapWidth, mapHeight, mapResolution);
	
	ros::Subscriber subClearMap = n.subscribe("clear", 1000, cbClearMap);
	ros::Subscriber subDrawLine = n.subscribe("drawline", 1000, cbDrawLine);
	ros::Subscriber subDrawSquare = n.subscribe("drawsquare", 1000, cbDrawSquare);
	
	mapPublisher = n.advertise<nav_msgs::OccupancyGrid>("map", 10);
	
	ros::Timer mapPubTimer = n.createTimer(ros::Duration(publishInterval), cbPublishMap);

	ros::spin();

	return 0;
}
