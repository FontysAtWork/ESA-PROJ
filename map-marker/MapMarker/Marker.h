#ifndef MARKER_H
#define MARKER_H

#include <vector>

class Marker
{
private:
    double x;
    double y;
    double angle;
public:
    Marker(double x, double y, double angle);
    double GetX();
    double GetY();
    double GetAngle();
    std::vector<double> GetQuaternation();
};

#endif // MARKER_H
