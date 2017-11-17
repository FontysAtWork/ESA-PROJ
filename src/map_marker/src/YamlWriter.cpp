#include "YamlWriter.hpp"
#include <iostream>
#include <fstream>
#include "geometry_msgs/Pose.h"

YamlWriter::YamlWriter()
{
}

void YamlWriter::writeAllMarkers(std::vector<Marker> markers, std::string fileName)
{

	std::ofstream stream;
	stream.open(fileName);
	for (int i = 0; i < markers.size(); ++i)
	{
		std::string text = markers[i].GetTypeStr() + "_" + std::to_string(i) + ":\n";
		std::string data = writeMarker(markers[i]);
		stream << text << data;
	}
	stream.close();

}

std::string YamlWriter::writeMarker(Marker marker)
{
	geometry_msgs::Pose pose = marker.GetPose();
	std::string text = positionPrefix + "x: " + std::to_string(pose.position.x) + "\n";
	text.append(positionPrefix + "y: " + std::to_string(pose.position.y) + "\n");
	text += positionPrefix + "z: " + std::to_string(pose.position.z) + "\n";
	text += orientationPrefix + "x: " + std::to_string(pose.orientation.x) + "\n";
	text += orientationPrefix + "y: " + std::to_string(pose.orientation.y) + "\n";
	text += orientationPrefix + "z: " + std::to_string(pose.orientation.z) + "\n";
	text += orientationPrefix + "w: " + std::to_string(pose.orientation.w) + "\n";
	return text;

}
