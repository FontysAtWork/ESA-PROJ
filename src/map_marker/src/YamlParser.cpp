#include "YamlParser.hpp"

YamlParser::YamlParser()
{


}

void YamlParser::init()
{


}

void YamlParser::loadYaml(std::string fileName)
{
	yaml_parser_t parser;
	yaml_token_t  token;
	FILE *fh = fopen(fileName.c_str(), "r");

	if(!yaml_parser_initialize(&parser))
	{
		ROS_ERROR("Failed to initialize parser!");
		return;
	}

	if(fh == NULL)
	{
		fputs("Failed to open file!\n", stderr);
		return;
	}

	// Set input file
	yaml_parser_set_input_file(&parser, fh);


	do {
		yaml_parser_scan(&parser, &token);
		if(token.type == YAML_KEY_TOKEN)
		{		
			KeyDataPair k;
			yaml_token_t data;
			yaml_parser_scan(&parser, &data);
			//parseToken(parser, tmp, token.type);
			parseData(parser, data, token.type, &k);
			yaml_token_t value;
			yaml_parser_scan(&parser, &value);
			if(value.type == YAML_VALUE_TOKEN)
			{
				yaml_parser_scan(&parser, &data);
				parseData(parser, data, value.type, &k);
			}
			parsedYaml.push_back(k);
		}
			

		//parseToken(parser, token, token.type);

		if(token.type != YAML_STREAM_END_TOKEN)
		yaml_token_delete(&token);
	} while(token.type != YAML_STREAM_END_TOKEN);

	fclose(fh);

	for (int i = 0; i < parsedYaml.size(); ++i)
	{
		KeyDataPair k = parsedYaml[i];
		std::cout << "Key: " << k.key << std::endl;

		for (int j = 0; j < k.data.size(); ++j)
		{
			std::cout << "Data: " << k.data[j] << std::endl;
		}
	}
}

void YamlParser::printYaml(std::string fileName)
{
	yaml_parser_t parser;
	yaml_token_t  token;
	FILE *fh = fopen(fileName.c_str(), "r");

	if(!yaml_parser_initialize(&parser))
	{
		ROS_ERROR("Failed to initialize parser!");
		return;
	}

	if(fh == NULL)
	{
		fputs("Failed to open file!\n", stderr);
		return;
	}

	// Set input file
	yaml_parser_set_input_file(&parser, fh);


	do {
		yaml_parser_scan(&parser, &token);

		switch(token.type)
		{
			// Stream start/end 
			case YAML_STREAM_START_TOKEN: 
			std::cout << "STREAM START" << std::endl; break;
			case YAML_STREAM_END_TOKEN:   
			std::cout << "STREAM END" << std::endl; break;
			// Token types (read before actual token) 
			case YAML_KEY_TOKEN:   
			std::cout << "(Key token)   "; break;
			case YAML_VALUE_TOKEN:   
			std::cout << "(Data token)  "; break;
			// Block delimeters 
			case YAML_BLOCK_SEQUENCE_START_TOKEN:   
			std::cout << "<b>Start Block (Sequence)</b>" << std::endl; break;
			case YAML_BLOCK_ENTRY_TOKEN:   
			std::cout << "<b>Start Block (Entry)</b>" << std::endl; break;
			case YAML_BLOCK_END_TOKEN:   
			std::cout << "<b>End block</b>" << std::endl; break;
			// Data 
			case YAML_BLOCK_MAPPING_START_TOKEN:   
			std::cout << "[Block mapping]" << std::endl; break;
			case YAML_SCALAR_TOKEN:   
			std::cout << "value   " << token.data.scalar.value << std::endl; break;
			// Array
			case YAML_FLOW_SEQUENCE_START_TOKEN:   
			std::cout << "Start array!" << std::endl; break;
			case YAML_FLOW_SEQUENCE_END_TOKEN:   
			std::cout << "End array!" << std::endl; break;
			case YAML_FLOW_ENTRY_TOKEN:   
			std::cout << "New array item: " << std::endl; break;
			// Others 
			default:
			std::cout << "Got token of type: " << token.type << std::endl; break;
		}

		if(token.type != YAML_STREAM_END_TOKEN)
		yaml_token_delete(&token);
	} while(token.type != YAML_STREAM_END_TOKEN);

	fclose(fh);




}

void YamlParser::parseData(yaml_parser_t parser, yaml_token_t token, yaml_token_type_t previousType, KeyDataPair *k)
{
	if(previousType == YAML_VALUE_TOKEN)
	{
		if(token.type == YAML_FLOW_SEQUENCE_START_TOKEN)
		{
			yaml_token_t dataToken;
			do{
				yaml_parser_scan(&parser, &dataToken);
				if(dataToken.type == YAML_SCALAR_TOKEN)
				{
					std::string sName(reinterpret_cast<char*>(dataToken.data.scalar.value));

					k->data.push_back(sName);
				}

			}while(dataToken.type != YAML_FLOW_SEQUENCE_END_TOKEN);
		}	
		else{
		std::string sName(reinterpret_cast<char*>(token.data.scalar.value));

					k->data.push_back(sName);
		
	}
	for (int i = 0; i < k->data.size(); ++i)
	{
		//std::cout << "Data: " << k.data[i] << std::endl;
	}
	}
	else if(previousType == YAML_KEY_TOKEN)
	{
		std::string sName(reinterpret_cast<char*>(token.data.scalar.value));

					k->key = sName;
		//std::cout << "Key: " << k->key << std::endl;
	}

}

void YamlParser::parseToken(yaml_parser_t parser, yaml_token_t token, yaml_token_type_t previousType)
{

	switch(token.type)
	{
		// Token types (read before actual token) 
		case YAML_KEY_TOKEN:   
		{

		}
		break;
		/*case YAML_VALUE_TOKEN:
		{
			yaml_token_t tmp;
			yaml_parser_scan(&parser, &tmp);
			//parseToken(parser, tmp, token.type);
			//parseData(tmp, token.type, k);

		}
		break;*/
		// Block delimeters 
		/*case YAML_BLOCK_SEQUENCE_START_TOKEN:   
		break;
		case YAML_BLOCK_ENTRY_TOKEN:   
		break;
		case YAML_BLOCK_END_TOKEN:   
		break;
		// Data 
		case YAML_BLOCK_MAPPING_START_TOKEN:   
		break;		*/
		/*case YAML_SCALAR_TOKEN:  
		{
			if(previousType ==YAML_VALUE_TOKEN)
			{
				data.push_back(token.data.scalar.value);
				std::cout << data.back() << std::endl;
			}
			else if(previousType == YAML_KEY_TOKEN)
			{
				keys.push_back(token.data.scalar.value);
				std::cout << keys.back() << std::endl;
			}

		}

		break;
		// Array
		case YAML_FLOW_SEQUENCE_START_TOKEN:   
		break;
		case YAML_FLOW_SEQUENCE_END_TOKEN:   
		break;
		case YAML_FLOW_ENTRY_TOKEN:   
		break;*/
		// Others 
		default:
		break;
	}

}


