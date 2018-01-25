#include <string>
#include <atwork_ros_msgs/ObjectIdentifier.h>

enum class ObjectType : int {
	F20_20_B = 1,		// Small Aluminium Profile (Black)
	F20_20_G = 2,		// Small Aluminium Profile (Grey)
	S40_40_B = 3,		// Big Aluminium Profile (Black)
	S40_40_G = 4,		// Big Aluminium Profile (Grey)
	M20_100 = 5,		// Screw (Bolt)
	M20 = 6,			// Small Nut
	M30 = 7,			// Large Nut
	R20 = 8,			// Plastic Tube
	BEARING_BOX = 9,	//
	BEARING = 10,		//
	AXIS = 11,			//
	DISTANCE_TUBE = 12,	//
	MOTOR = 13,			//
	CONTAINER_B = 14,	// A Container (Blue)
	CONTAINER_R = 15	// A Container (Red)
};

class ObjectIdentifier
{
	public:
		ObjectIdentifier(atwork_ros_msgs::ObjectIdentifier obj);
		ObjectIdentifier(ObjectType type, int type_id, int instance_id, std::string description);
		~ObjectIdentifier();

		ObjectType GetType();
		int GetTypeId();
		int GetInstanceId();
		std::string GetDescription();
		atwork_ros_msgs::ObjectIdentifier GetAtworkMsg();	
		std::string	Print();

	private:
		ObjectType type;
		int type_id;
		int instance_id;
		std::string description;
};
