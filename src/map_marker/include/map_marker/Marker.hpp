#ifndef MARKER_H
#define MARKER_H

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include <string>

enum MarkerType {
    Navigation = 0,
    Workspace = 1,
    Robot = 2
};

class Marker
{
private:
    MarkerType type;
    geometry_msgs::Pose pos;
    geometry_msgs::Quaternion SetQuaternation(double roll, double pitch, double yaw);
public:
    Marker(double x, double y, double angle, MarkerType t);
    Marker(geometry_msgs::Pose p, MarkerType t);
    double GetX();
    double GetY();
    double GetAngle();
    MarkerType GetType();
    std::string GetTypeStr();
    geometry_msgs::Quaternion GetQuaternation();
    geometry_msgs::Point GetPoint();
    geometry_msgs::Pose GetPose();
    double Rad2Deg(double rad);
    double Deg2Rad(double deg);
};

#endif // MARKER_H
