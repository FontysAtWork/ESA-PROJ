#include <sstream>
#include <task_executor/LocationIdentifier.hpp>

LocationIdentifier::LocationIdentifier(atwork_ros_msgs::LocationIdentifier loc) {
	type = (LocationType) loc.type.data;
	instance_id = loc.instance_id.data;
	description = loc.description.data;

}

LocationIdentifier::LocationIdentifier(LocationType type, int instance_id, std::string description) :
    type(type),
    instance_id(instance_id),
    description(description) {
    

}

LocationIdentifier::LocationIdentifier(){
}

LocationIdentifier::~LocationIdentifier() {

}


void LocationIdentifier::SetLocation(atwork_ros_msgs::LocationIdentifier loc) {
		type = (LocationType) loc.type.data;
	instance_id = loc.instance_id.data;
	description = loc.description.data;
}

void LocationIdentifier::SetLocation(LocationType type, int instance_id, std::string description) {
    this->type = type;
    this->instance_id = instance_id;
    this->description = description;

}

LocationType LocationIdentifier::GetType() {
	return type;
}

int LocationIdentifier::GetInstanceId() {
	return instance_id;
}

std::string LocationIdentifier::GetDescription() {
	return description;
}

atwork_ros_msgs::LocationIdentifier LocationIdentifier::GetAtworkMsg() {
	atwork_ros_msgs::LocationIdentifier loc;
	loc.type.data = (int)type;
	loc.instance_id.data = instance_id;
	loc.description.data = description;
	return loc;
}

std::string LocationIdentifier::Print() { 
	std::ostringstream s;
	s << "LocationIdentifier: (" << std::to_string((int)GetType());
	s << ", " << std::to_string(GetInstanceId());
    s << ") " << GetDescription();   
    return s.str();   
}
