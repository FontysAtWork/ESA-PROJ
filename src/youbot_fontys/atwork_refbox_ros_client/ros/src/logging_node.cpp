#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <atwork_ros_msgs/LoggingStatus.h>

int main(int argc, char **argv)
{
  atwork_ros_msgs::LoggingStatus msg;
// declaration of the type of message see atwork_refbox_comm of data type information.
  ros::init(argc, argv, "logging_node");
  ros::NodeHandle nh("~");
  ros::Publisher logging_pub_ = nh.advertise<atwork_ros_msgs::LoggingStatus>("/robot_example_ros/logging_status", 100);
// setting the node to advertise a LoggingStatus type of message to the logging_status topic
  ros::Rate loop_rate(10); // one Hz
  ROS_INFO("CFH Robot example is running");

  while (ros::ok())
 {
   msg.is_logging.data = 1;
// set boolean True that the youbot is logging
   logging_pub_.publish(msg);
// Publishing the boolean
   ros::spinOnce();
  }

  return 0;
}

