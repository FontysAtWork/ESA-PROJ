#include "Marker.h"

Marker::Marker(double xPos, double yPos, double ang) :
    x(xPos), y(yPos), angle(ang) {

}

double Marker::GetX() {
    return this->x;
}

double Marker::GetY() {
    return this->y;
}

double Marker::GetAngle() {
    return this->angle;
}

std::vector<double> Marker::GetQuaternation() {
    std::vector<double> v;
    //do stuff
    return v;
}
