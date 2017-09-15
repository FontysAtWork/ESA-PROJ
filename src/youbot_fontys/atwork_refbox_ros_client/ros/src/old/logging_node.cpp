#include <ros/ros.h>
//#include <atwork_robot_example_ros/robot_example_ros.h>
#include <std_msgs/Bool.h>
int logg(int argc, char **argv)
{
  ros::init(argc, argv, "logging_node");
  ros::NodeHandle nh("~");
  ros::Publisher logging_pub = nh.advertise<std_msgs::Bool>("logging_status", 1000);
   std_msgs::Bool msg;
  ros::Rate loop_rate(10); // one Hz

 // RobotExampleROS robot_example_ros(n);

  ROS_INFO("Alive");

  while (ros::ok())
 {
	msg.data=1;

     logging_pub.publish(msg);

     ROS_INFO("logging");

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
