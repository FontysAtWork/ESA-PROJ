#include "ros/ros.h"
#include "nav_msgs/MapMetaData.h"
#include <cassert>
#include "map.hpp"

namespace faw {
	
	Map::Map(int width, int height, float resolution)
		: _width(width), _height(height), _res(resolution)
	{	
		_mapData = std::vector<int8_t>(width * height, -1);
	}	

	int Map::width(void)
	{
		return _width;
	}

	int Map::height(void)
	{
		return _height;
	}

	int8_t Map::getPixel(const int x, const int y)
	{
		assert(x >= 0 && x <= _width);
		assert(y >= 0 && y <= _height);
		
		return _mapData.at( (y * _height) + x );
	}
	
	void Map::setPixel(const int x, const int y, const int8_t val)
	{
		assert(x >= 0 && x <= _width);
		assert(y >= 0 && y <= _height);
		
		_mapData[(y * _height) + x] = val;
	}
	
	void Map::clear(void)
	{
		std::fill(_mapData.begin(), _mapData.end(), -1);
	}

	nav_msgs::OccupancyGrid Map::toMessage(void)
	{
		nav_msgs::OccupancyGrid msg;
		msg.header.stamp = ros::Time::now();
		
		//Map metadata (nav_msgs/MapMetaData.msg)
		msg.info.map_load_time = msg.header.stamp;
		msg.info.resolution = _res;
		msg.info.width = _width;
		msg.info.height = _height;
		

		msg.data = _mapData;
		
		return msg;
	}

}
