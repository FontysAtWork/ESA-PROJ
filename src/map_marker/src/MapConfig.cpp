#include "MapConfig.hpp"
    MapConfig::MapConfig()
    {

    }

    void MapConfig::setFullConfigData(std::vector<KeyDataPair> data)
    {
        for (int i = 0; i < data.size(); ++i)
        {
            setConfigData(data[i]);
        }

    }

    void MapConfig::setConfigData(KeyDataPair data)
    {

        if(data.key == "image")
        {
            imageName = data.data[0];
        }
        else if(data.key == "resolution")
        {
            resolution = std::atof(data.data[0].c_str());
        }
        else if(data.key == "origin")
        {
            origin.position.x = std::atof(data.data[0].c_str());
            origin.position.y = std::atof(data.data[1].c_str());
            origin.orientation.z = std::atof(data.data[2].c_str());
        }
        else if(data.key == "negate")
        {
            negate = std::atof(data.data[0].c_str());
        }
        else if(data.key == "occupied_thresh")
        {
            occupiedThresh = std::atof(data.data[0].c_str());
        }
        else if(data.key == "free_thresh")
        {
            freeThresh = std::atof(data.data[0].c_str());
        }

    }

    geometry_msgs::Pose MapConfig::getOrigin()
    {
        return origin;
    }
    double MapConfig::getResolution()
    {
        return resolution;
    }
    std::string MapConfig::getImageName()
    {
        return imageName;
    }
    double MapConfig::getOccupiedThresh()
    {
        return occupiedThresh;
    }
    double MapConfig::getFreeThresh()
    {
        return freeThresh;
    }
    double MapConfig::getNegate()
    {
        return negate;
    }
    void MapConfig::printYaml()
    {
        
    }


