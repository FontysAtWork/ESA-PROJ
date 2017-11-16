#ifndef YAML_CONFIG_H
#define YAML_CONFIG_H

#include <string>
#include "geometry_msgs/Pose.h"

class YamlConfig
{
private:
    std::string imageName;
    double resolution;
    geometry_msgs::Pose origin;
    double negate;
    double occupiedThresh;
    double freeThresh;
public:
    YamlConfig();
    geometry_msgs::Pose getOrigin();
    double getResolution();
    std::string getImageName();
    double getOccupiedThresh();
    double getFreeThresh();
    double getNegate();
    void printYaml();
};

#endif // YAML_CONFIG_H
