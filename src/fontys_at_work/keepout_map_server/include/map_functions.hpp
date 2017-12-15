#include "ros/ros.h"
#include <cassert>
#include "map.hpp"

namespace faw {
namespace mapFunctions
{
	/*
	 * Function retreived from: https://stackoverflow.com/a/16405254
	 */
	void drawLine(faw::Map &src, float  x0, float  y0, float  x1, float  y1, const int8_t gradient)
	{
		if(x0 < 1 || x0 > src.width()  - 1 ||
		   y0 < 1 || y0 > src.height() - 1 ||
		   x1 < 1 || x1 > src.width()  - 1 ||
		   y1 < 1 || y1 > src.height() - 1 )
	   {
		   return;
	   }
		
		float x{x1 - x0}, y{y1 - y0};
		const float max{std::max(std::fabs(x), std::fabs(y))};
		x /= max; y /= max;
		for (float n{0}; n < max; ++n)
		{
			src.setPixel(x0+1, y0, gradient);
			src.setPixel(x0, y0+1, gradient);
			src.setPixel(x0, y0-1, gradient);
			src.setPixel(x0-1, y0, gradient);
			
			src.setPixel(x0, y0, gradient);
			x0 += x; y0 += y;
		}
	}
	
	void drawSquare(faw::Map &src, const int x, const int y, const int width, const int height, const int8_t gradient)
	{
		drawLine(src, x, y, x + width, y, gradient);
		drawLine(src, x + width, y, x + width, y + height, gradient);
		drawLine(src, x + width, y + height, x, y + height, gradient);
		drawLine(src, x, y + height, x, y, gradient);
	}
}
}
