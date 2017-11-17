
#ifndef YAML_PARSER_H
#define YAML_PARSER_H

#include <string>
#include <vector>
#include <ros/ros.h>

extern "C" {
#include "yaml.h"
}

typedef struct{
	std::string key;
	std::vector<std::string> data;
}KeyDataPair;

class YamlParser
{
private:
	std::vector<KeyDataPair> parsedYaml;
	std::vector<unsigned char*> keys;
	std::vector<unsigned char*> data;
	void parseToken(yaml_parser_t parser, yaml_token_t token, yaml_token_type_t previousType);
	void parseData(yaml_parser_t parser, yaml_token_t token, yaml_token_type_t previousType, KeyDataPair* k);
public:
	YamlParser();
	void init();
	void loadYaml(std::string fileName);
	void printYaml(std::string fileName);
};

#endif // YAML_HPP

