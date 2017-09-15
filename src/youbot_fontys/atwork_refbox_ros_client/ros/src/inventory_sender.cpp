#include <atwork_ros_msgs/TaskInfo.h>
#include <iostream>
#include <vector>
#include <atwork_ros_msgs/TaskInfo.h>
#include <atwork_ros_msgs/Inventory.h>
#include <ros/ros.h>

ros::Publisher object_pub_;
void InventoryCallback(const atwork_ros_msgs::Inventory &item){
  
  for(size_t i=0; i<item.items.size(); i++){
  std::cout << "Object data " << item.items[i].object.type.data << " Description " << item.items[i].object.description.data <<std::endl;
  //display on the referee robot example the data of the object
  std_msgs::String object;
  std::stringstream object_1;
  object_1 <<item.items[i].object.type.data;
  object.data = object_1.str();
  object_pub_.publish(object);
  /*publish string message the data of the type of object:
  uint8   F20_20_B      = 1  # Small Aluminium Profile (Black)
  uint8   F20_20_G      = 2  # Small Aluminium Profile (Grey)
  uint8   S40_40_B      = 3  # Big Aluminium Profile (Black)
  uint8   S40_40_G      = 4  # Big Aluminium Profile (Grey)
  uint8   M20_100       = 5  # Screw (Bolt)
  uint8   M20           = 6  # Small Nut
  uint8   M30           = 7  # Large Nut
  uint8   R20           = 8  # Plastic Tube
  uint8   BEARING_BOX   = 9  #
  uint8   BEARING       = 10 #
  uint8   AXIS          = 11 #
  uint8   DISTANCE_TUBE = 12 #
  uint8   MOTOR         = 13 #
  uint8   CONTAINER_B   = 14 # A Container (Blue)
  uint8   CONTAINER_R   = 15 # A Container (Red)*/
  } 
}

void taskCallback(const atwork_ros_msgs::TaskInfo &task){
  
  for(size_t i=0; i<task.tasks.size(); i++){
  std::cout << " source " << task.tasks[i].transportation_task.source.description.data <<" destination " << task.tasks[i].transportation_task.destination.description.data << " id_type " << task.tasks[i].transportation_task.destination.type.data<< " id " << task.tasks[i].transportation_task.destination.instance_id.data <<std::endl;
  //display on th referee robot example the data of the source and destination
  std_msgs::String source;
  std::stringstream source_1;
  source_1 << "Source " <<task.tasks[i].transportation_task.source.description.data << " Destination " <<task.tasks[i].transportation_task.destination.description.data;
  source.data = source_1.str();
  object_pub_.publish(source);
  //display string message the data of the source and destination of the objects:
  /*
  # Location types
  uint8    SH    = 1  # Shelf
  uint8    WS    = 2  # Workstation
  uint8    CB    = 3  # Conveyor belt
  uint8    WP    = 4  # Way Point (additional navigation goal)
  uint8    PP    = 5  # Precision Placement platform
  uint8    ROBOT = 6
  */
  
  } 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "inventory_sender");
  ros::NodeHandle nh;
  ros::Subscriber inventory_sub_ = nh.subscribe("/robot_example_ros/task_info", 1000, taskCallback);
  ros::Subscriber inventory_qub_1_ = nh.subscribe("/robot_example_ros/inventory", 500, InventoryCallback);
  object_pub_ = nh.advertise<std_msgs::String>("/inventory_data", 1000);
  ros::spin();
  return 0;
}

