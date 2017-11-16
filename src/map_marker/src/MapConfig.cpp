#include "../include/map_marker/MapConfig.hpp"
    YamlConfig::YamlConfig()
    {

    }
    geometry_msgs::Pose YamlConfig::getOrigin()
    {
        return origin;
    }
    double YamlConfig::getResolution()
    {
        return resolution;
    }
    std::string YamlConfig::getImageName()
    {
        return imageName;
    }
    double YamlConfig::getOccupiedThresh()
    {
        return occupiedThresh;
    }
    double YamlConfig::getFreeThresh()
    {
        return freeThresh;
    }
    double YamlConfig::getNegate()
    {
        return negate;
    }
    void YamlConfig::printYaml()
    {
        
    }


