#include "NoGoLine.hpp"

NoGoLine::NoGoLine()
{
	state = Idle;

}

void NoGoLine::SetPoint(const int x, const int y, map_marker::QNode& qnode)
{
	switch(state)
	{
		case FirstPoint:
		{
			x1 = x;
			y1 = y;
			state = SecondPoint;
			break;
		}
		case SecondPoint:
		{
			x2 = x;
			y2 = y;
			qnode.DrawLine(x1, y1, x2, y2);
			//qnode
			state = Idle;
			break;
		}

		default:
		break;
	}
}

void NoGoLine::ButtonClicked()
{
	state = FirstPoint;

}