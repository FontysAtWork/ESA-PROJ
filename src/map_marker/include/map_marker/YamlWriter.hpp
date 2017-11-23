
#ifndef YAML_WRITER_H
#define YAML_WRITER_H

#include "Marker.hpp"
#include <vector>
#include <string>

class YamlWriter
{
private:
	std::string writeMarker(Marker marker);
	const std::string positionPrefix = "position_";
	const std::string orientationPrefix = "orientation_";
public:
	YamlWriter();
	void writeAllMarkers(std::vector<Marker> markers, std::string fileName);
};

#endif // YAML_WRITER_H

