#ifndef NO_GO_LINE_H
#define NO_GO_LINE_H

#include <string>
//#include "Qnode.hpp"

/*enum LineState 
{
    Idle = 0,
    FirstPoint = 1,
    SecondPoint = 2
};*/

class NoGoLine
{
private:
    double coordX1;
    double coordY1;
    double coordX2;
    double coordY2;
    std::string lineName;
    //LineState state;

public:
    NoGoLine(double x1, double y1, double x2, double y2, std::string name);
    double GetX1();
    double GetX2();
    double GetY1();
    double GetY2();
    std::string GetName();
    //void SetPoint(const int x, const int y, map_marker::QNode& qnode);
    //void ButtonClicked();

};

#endif // NO_GO_LINE_H
