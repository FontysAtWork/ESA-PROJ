#include <sstream>
#include <task_handler/ObjectIdentifier.hpp>

ObjectIdentifier::ObjectIdentifier(atwork_ros_msgs::ObjectIdentifier obj) {
	type = (ObjectType) obj.type.data;
	type_id = obj.type_id.data;
	instance_id = obj.instance_id.data;
	description = obj.description.data;

}

ObjectIdentifier::ObjectIdentifier(ObjectType _type, int _type_id, int _instance_id, std::string _description) :
    type(_type),
    type_id(_type_id),
    instance_id(_instance_id),
    description(_description) {
    

}

ObjectIdentifier::~ObjectIdentifier() {

}

ObjectType ObjectIdentifier::GetType() {
	return type;
}

int ObjectIdentifier::GetTypeId() {
	return type_id;
}

int ObjectIdentifier::GetInstanceId() {
	return instance_id;
}

std::string ObjectIdentifier::GetDescription() {
	return description;
}

atwork_ros_msgs::ObjectIdentifier ObjectIdentifier::GetAtworkMsg() {
	atwork_ros_msgs::ObjectIdentifier obj;
	obj.type.data = (int) type;
	obj.type_id.data = type_id;
	obj.instance_id.data = instance_id;
	obj.description.data = description;
	return obj;
}

std::string ObjectIdentifier::Print() { 
	std::ostringstream s;
	s << "ObjectIdentifier: (" << std::to_string((int)GetType());
	s << ", " << std::to_string(GetTypeId());
	s << ", " << std::to_string(GetInstanceId());
	s << ") " << GetDescription();  
    return s.str();   
}
