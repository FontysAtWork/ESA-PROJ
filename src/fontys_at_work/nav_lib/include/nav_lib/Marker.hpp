#ifndef MARKER_H
#define MARKER_H

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include <string>

enum MarkerType {
    Shelf = 1,
    Workstation = 2,
    Conveyor = 3,
    Waypoint = 4,
    Precision = 5,
    Robot = 6
};

class Marker
{
private:
    MarkerType type;
    geometry_msgs::Pose pos;    
    std::string markerName;
public:
    Marker(double x, double y, double angle, MarkerType t, std::string name);
    Marker(geometry_msgs::Pose p, MarkerType t, std::string name);
    double GetX();
    double GetY();
    double GetAngle();
    MarkerType GetType();
    std::string GetTypeStr();
    std::string GetName();
    geometry_msgs::Quaternion SetQuaternation(double roll, double pitch, double yaw);
    geometry_msgs::Quaternion GetQuaternation();
    geometry_msgs::Point GetPoint();
    geometry_msgs::Pose GetPose();
    double Rad2Deg(double rad);
    double Deg2Rad(double deg);
};

#endif // MARKER_H
