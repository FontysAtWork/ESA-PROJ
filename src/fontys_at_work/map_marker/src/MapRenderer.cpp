#include "MapRenderer.hpp"
#include "ros/ros.h"

MapRenderer::MapRenderer()
{
	width = 1;
	height = 1;
	image = QImage(width, height, QImage::Format_RGB888);
	image.fill(Qt::lightGray);
}

void MapRenderer::reset(int w, int h)
{
	if(w != width || h != height)
	{
		width = w;
		height = h;
		image = image.scaled(width, height);
	}

	image.fill(Qt::lightGray);
}


void MapRenderer::drawOccupancyGrid(nav_msgs::OccupancyGrid grid)
{
	for (int i = 0; i < width; ++i)
	{
		for (int j = 0; j < height; ++j)
		{
			int currentindex = i * width + j;

			if(grid.data[currentindex] >= 0 && grid.data[currentindex] < FREE_THRESH)
			{
				image.setPixel(i, j, Qt::white);
			}
			else if(grid.data[currentindex] > OCCUPIED_THRESH)
			{
				image.setPixel(i, j, Qt::black);
			}
		}
	}
}


QImage& MapRenderer::getImage()
{
	return image;


}
