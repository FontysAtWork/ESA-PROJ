#pragma once

#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>

namespace faw {

	class Map
	{
		public:
			Map() = default;
			Map(int width, int height, float resolution);
			int width(void);
			int height(void);
			int8_t getPixel(const int x, const int y);
			void setPixel(const int x, const int y, const int8_t val);
			void clear(void);
			nav_msgs::OccupancyGrid toMessage(void);
			map_msgs::OccupancyGridUpdate toUpdateMessage(void);
		
		private:
			int _width;
			int _height;
			float _res;
			std::vector<int8_t> _mapData;
	};

}
	
