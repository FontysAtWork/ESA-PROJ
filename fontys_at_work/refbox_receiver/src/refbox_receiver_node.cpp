/*
https://github.com/industrial-robotics/atwork_refbox_ros_client
*/

#include <ros/ros.h>
#include <refbox_receiver/refbox_receiver.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "refbox_receiver");
  ros::NodeHandle nh("~");
  ros::Rate loop_rate(10); // one Hz

  RefboxReceiver refbox_receiver(nh);

  ROS_INFO("RefboxReceiver is running!");

  while (ros::ok()) {
    refbox_receiver.sendBeacon();

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
