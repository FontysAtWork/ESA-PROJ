#ifndef MARKER_H
#define MARKER_H

#include <vector>

enum MarkerType {
    Navigation,
    Workspace
};

class Marker
{
private:
    double x;
    double y;
    double angle;
    MarkerType type;
public:
    Marker(double x, double y, double angle);
    double GetX();
    double GetY();
    double GetAngle();
    MarkerType GetType();
    std::vector<double> GetQuaternation();
};

#endif // MARKER_H
