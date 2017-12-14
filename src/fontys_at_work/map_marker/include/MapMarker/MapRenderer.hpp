#ifndef MAP_RENDERER_H
#define MAP_RENDERER_H

#include "nav_msgs/OccupancyGrid.h"
#include <QImage>

#define FREE_THRESH 0.195
#define OCCUPIED_THRESH 0.65


class MapRenderer{

public:
	MapRenderer();
	void reset(int width, int height);
	void drawOccupancyGrid(nav_msgs::OccupancyGrid grid);
	QImage& getImage();


private:
	int width;
	int height;
	QImage image;



};





#endif // MAP_RENDERER_H