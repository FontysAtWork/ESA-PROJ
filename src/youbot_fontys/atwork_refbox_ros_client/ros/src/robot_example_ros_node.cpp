#include <ros/ros.h>
#include <atwork_robot_example_ros/robot_example_ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_example_ros_node");
  ros::NodeHandle nh("~");
  ros::Rate loop_rate(10); // one Hz

  RobotExampleROS robot_example_ros(nh);

  ROS_INFO("CFH Robot example is running");

  while (ros::ok())
 {
    robot_example_ros.sendBeacon();

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
