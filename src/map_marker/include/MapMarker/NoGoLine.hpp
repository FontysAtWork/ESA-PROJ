#ifndef NO_GO_LINE_H
#define NO_GO_LINE_H

#include "Qnode.hpp"

enum LineState 
{
    Idle = 0,
    FirstPoint = 1,
    SecondPoint = 2
};

class NoGoLine
{
private:
    int x1;
    int x2;
    int y1;
    int y2;
    LineState state;

public:
    NoGoLine();
    void SetPoint(const int x, const int y, map_marker::QNode& qnode);
    void ButtonClicked();

};

#endif // NO_GO_LINE_H
