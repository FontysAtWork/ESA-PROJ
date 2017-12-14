#include "NoGoLine.hpp"

NoGoLine::NoGoLine(double x1, double y1, double x2, double y2, std::string name)
{
	coordX1 = x1;
    coordY1 = y1;
    coordX2 = x2;
    coordY2 = y2;
    lineName = name;

}

double NoGoLine::GetX1()
{
	return coordX1;
}

double NoGoLine::GetX2()
{
	return coordX2;
}

double NoGoLine::GetY1()
{
	return coordY1;
}

double NoGoLine::GetY2()
{
	return coordY2;
}

std::string NoGoLine::GetName()
{
	return lineName;
}

/*void NoGoLine::SetPoint(const int x, const int y, map_marker::QNode& qnode)
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

}*/
