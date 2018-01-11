
#ifndef YAML_WRITER_H
#define YAML_WRITER_H

#include "Marker.hpp"
#include "NoGoLine.hpp"
#include <vector>
#include <string>

class YamlWriter
{
private:
	std::string writeMarker(Marker marker);
	std::string writeLine(NoGoLine line);
	const std::string positionPrefix = "position_";
	const std::string orientationPrefix = "orientation_";
public:
	YamlWriter();
	void writeAllMarkers(std::vector<Marker> markers, std::string fileName);
	void writeAllLines(std::vector<NoGoLine> lines, std::string fileName);
};

#endif // YAML_WRITER_H

