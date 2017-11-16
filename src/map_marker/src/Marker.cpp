#include "Marker.hpp"
#include <tf/transform_datatypes.h>
#include <math.h>

Marker::Marker(double xPos, double yPos, double angle, MarkerType t) {
	pos.position.x = xPos;
	pos.position.y = yPos;
	pos.orientation = SetQuaternation(0, 0, Deg2Rad(angle));
	type = t;
}

Marker::Marker(geometry_msgs::Pose p, MarkerType t) {
	pos = p;
	type = t;
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
    return Rad2Deg(tf::getYaw(pos.orientation));
}

MarkerType Marker::GetType() {
    return type;
}

std::string Marker::GetTypeStr() {
	if(type == Navigation) {
		return "Navigation";
	} else if (type == Workspace) {
		return "Workspace";
	} else {
		return "Other";
	}
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