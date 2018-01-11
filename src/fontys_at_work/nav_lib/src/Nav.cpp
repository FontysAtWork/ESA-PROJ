#include "nav_lib/Nav.hpp"

namespace NAV
{
	std::vector<Marker> FillMarkerList(std::vector<KeyDataPair> data)
	{
		std::vector<Marker> markers;
		geometry_msgs::Pose p;
		MarkerType t;
		std::string name; 
		for (int i = 0; i < data.size(); i++)
		{
			
			if(data[i].key.compare(0,6,"Marker") == 0)
			{
				if(data[i].data[0].compare(0,3,"She") == 0)
				{
					t = Shelf;
				}
				else if(data[i].data[0].compare(0,3,"Wor") == 0)
				{
					t = Workstation;
				}
				else if(data[i].data[0].compare(0,3,"Con") == 0)
				{
					t = Conveyor;
				}
				else if(data[i].data[0].compare(0,3,"Way") == 0)
				{
					t = Waypoint;
				}
				else if(data[i].data[0].compare(0,3,"Pre") == 0)
				{
					t = Precision;
				}
				else
				{
					std::cout << data[i].data[0] << std::endl;
					t = Robot;
				}
			}
			else if (data[i].key.compare("position_x") == 0)
			{
				p.position.x = std::atof(data[i].data[0].c_str());
			}
			else if (data[i].key.compare("position_y") == 0)
			{
				p.position.y = std::atof(data[i].data[0].c_str());
			}
			else if (data[i].key.compare("position_z") == 0)
			{
				p.position.z = std::atof(data[i].data[0].c_str());
			}
			else if (data[i].key.compare("orientation_x") == 0)
			{
				p.orientation.x = std::atof(data[i].data[0].c_str());
			}
			else if (data[i].key.compare("orientation_y") == 0)
			{
				p.orientation.y = std::atof(data[i].data[0].c_str());
			}
			else if (data[i].key.compare("orientation_z") == 0)
			{
				p.orientation.z = std::atof(data[i].data[0].c_str());
			}
			else if (data[i].key.compare("orientation_w") == 0)
			{
				p.orientation.w = std::atof(data[i].data[0].c_str());
			}
			else if (data[i].key.compare("Name") == 0)
			{
				name = data[i].data[0];
			}
			else
			{
				ROS_ERROR("Unkown data!");
			}

			if(i == data.size() -1 || i < data.size() - 1 && data[i+1].key.compare(0,6,"Marker") == 0)
			{
				Marker m(p, t, name);
				markers.push_back(m);
				t = Robot;
				p.position.x = 0;
				p.position.y = 0;
				p.position.z = 0;
				p.orientation.x = 0;
				p.orientation.y = 0;
				p.orientation.z = 0;
				p.orientation.w = 0;
				name = "";
			}
		}
		return markers;
	}
	
	std::vector<NoGoLine> FillLineList(std::vector<KeyDataPair> data)//, std::vector<NoGoLine> lines)
	{
		std::vector<NoGoLine> lines;
		double x1;
		double x2;
		double y1;
		double y2;
		std::string name; 
		for (int i = 0; i < data.size(); i++)
		{
			
			if (data[i].key.compare("x1") == 0)
			{
				x1 = std::atof(data[i].data[0].c_str());
			}
			else if (data[i].key.compare("x2") == 0)
			{
				x2 = std::atof(data[i].data[0].c_str());
			}
			else if (data[i].key.compare("y1") == 0)
			{
				y1 = std::atof(data[i].data[0].c_str());
			}
			else if (data[i].key.compare("y2") == 0)
			{
				y2 = std::atof(data[i].data[0].c_str());
			}
			else if (data[i].key.compare("Name") == 0)
			{
				name = data[i].data[0];
			}
			else
			{
				ROS_ERROR("Unkown data!");
			}

			if(i == data.size() -1 || i < data.size() - 1 && data[i+1].key.compare(0,8,"NoGoLine") == 0)
			{
				NoGoLine l(x1, y1, x2, y2, name);
				lines.push_back(l);
				x1 = 0;
				x2 = 0;
				y1 = 0;
				y2 = 0;
				name = "";
			}
		}
		
		return lines;
	}
	
	
	std::vector<Marker> LoadMarkers(std::string filename){
		YamlParser yamlp;
		yamlp.loadYaml(filename);
		return FillMarkerList(yamlp.GetParsedYaml());
		
	}
	
	std::vector<NoGoLine> LoadNoGoLines(std::string filename){
		YamlParser yamlp;
		yamlp.loadYaml(filename);
		
		return FillLineList(yamlp.GetParsedYaml());
		
	}
	
	MapConfig LoadMap(std::string filename){
		YamlParser yamlp;
		yamlp.loadYaml(filename);
		MapConfig mapconfig;
		mapconfig.setFullConfigData(yamlp.GetParsedYaml());
		return mapconfig;
	}
	
	void WriteMarkers(std::vector<Marker> markers, std::string filename){
		YamlWriter yamlWriter;
		yamlWriter.writeAllMarkers(markers, filename);
		
	}
	
	void WriteNoGoLines(std::vector<NoGoLine> lines, std::string filename){
		YamlWriter yamlWriter;
		yamlWriter.writeAllLines(lines, filename);
	}
	
	
};



