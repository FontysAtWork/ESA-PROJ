#include <string>
#include <atwork_ros_msgs/LocationIdentifier.h>

enum class LocationType : int {
	SH = 1,		// Shelf
	WS = 2,		// Workstation
	CB = 3,		// Conveyor belt
	WP = 4,		// Way Point (additional navigation goal)
	PP = 5,		// Precision Placement platform
	ROBOT = 6	// Robot
};

class LocationIdentifier
{
	public:
		LocationIdentifier(atwork_ros_msgs::LocationIdentifier obj);
		LocationIdentifier(LocationType type, int instance_id, std::string description);
		~LocationIdentifier();

		LocationType GetType();
		int GetInstanceId();
		std::string GetDescription();
		atwork_ros_msgs::LocationIdentifier GetAtworkMsg();
		std::string	Print();

	private:
		LocationType type;
		int instance_id;
		std::string description;
};
