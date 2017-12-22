#ifndef MAP_CONFIG_H
#define MAP_CONFIG_H

#include <string>
#include "geometry_msgs/Pose.h"
#include "nav_lib/YamlParser.hpp"

class MapConfig
{
private:
    std::string imageName;
    double resolution;
    geometry_msgs::Pose origin;
    double negate;
    double occupiedThresh;
    double freeThresh;
public:
    MapConfig();
    void setFullConfigData(std::vector<KeyDataPair> data);
    void setConfigData(KeyDataPair data);
    geometry_msgs::Pose getOrigin();
    double getResolution();
    std::string getImageName();
    double getOccupiedThresh();
    double getFreeThresh();
    double getNegate();
    void printYaml();
};

#endif // MAP_CONFIG_H
