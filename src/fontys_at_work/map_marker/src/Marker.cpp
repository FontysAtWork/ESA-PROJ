#include "Marker.hpp"
#include <tf/transform_datatypes.h>
#include <math.h>

Marker::Marker(double xPos, double yPos, double angle, MarkerType t, std::string name) {
	pos.position.x = xPos;
	pos.position.y = yPos;
	pos.orientation = SetQuaternation(0, 0, Deg2Rad(angle));
	type = t;
	markerName = name;
}

Marker::Marker(geometry_msgs::Pose p, MarkerType t, std::string name) {
	pos = p;
	type = t;
	markerName = name;
}

geometry_msgs::Quaternion Marker::SetQuaternation(double roll, double pitch, double yaw) {
	return tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
}

double Marker::GetX() {
    return pos.position.x;
}

double Marker::GetY() {
    return pos.position.y;
}

double Marker::GetAngle() {
	tf::Quaternion q;
	tf::quaternionMsgToTF(pos.orientation, q);
	q.normalize();
	return Rad2Deg(tf::getYaw(q));
}

MarkerType Marker::GetType() {
    return type;
}

std::string Marker::GetTypeStr() {
	if(type == Shelf) {
		return "Shelf";
	} else if (type == Workstation) {
		return "Workstation";
	} else if (type == Conveyor) {
		return "Conveyor";
	} else if (type == Waypoint) {
		return "Waypoint";
	} else if (type == Precision) {
		return "Precision";
	} else {
		return "Other";
	}
}

std::string Marker::GetName() {
	return markerName;
}

geometry_msgs::Quaternion Marker::GetQuaternation() {
	return pos.orientation;
}

geometry_msgs::Point Marker::GetPoint() {
	return pos.position;
}

geometry_msgs::Pose Marker::GetPose() {
	return pos;
}

double Marker::Rad2Deg(double rad) {
	return (rad * 180.0 / M_PI);
}

double Marker::Deg2Rad(double deg) {
	return (deg * M_PI / 180.0);
}