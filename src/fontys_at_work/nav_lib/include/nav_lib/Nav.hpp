#ifndef NAV_LIB
#define NAV_LIB
	#include <string>
	#include <vector>
	#include "Marker.hpp"
	#include "NoGoLine.hpp"
	#include "YamlParser.hpp"
	#include "YamlWriter.hpp"
	#include "MapConfig.hpp"
	
namespace NAV
{
	using namespace std;
	std::vector<Marker> LoadMarkers(std::string);
	std::vector<NoGoLine> LoadNoGoLines(std::string);
	MapConfig LoadMap(std::string);
	
	void WriteMarkers(std::vector<Marker>, std::string);
	void WriteNoGoLines(std::vector<NoGoLine>, std::string);
}


#endif //NAV_LIB
