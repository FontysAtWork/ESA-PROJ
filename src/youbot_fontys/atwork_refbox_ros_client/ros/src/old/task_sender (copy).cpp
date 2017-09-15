#include <ros/ros.h>
#include <atwork_ros_msgs/TaskInfo.h>
#include <iostream>
#include <vector>
//#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>


//std::vector<geometry_msgs::PoseStamped> location_goals;
std::vector<geometry_msgs::Pose> location_goals;
geometry_msgs::PoseArray pose_array;
ros::Publisher poses_queue;

void taskInfoCallback(const atwork_ros_msgs::TaskInfo &task){
  
  std::cout << "location type " << task.tasks[0].navigation_task.location.type.data << " Description " << task.tasks[0].navigation_task.location.description.data <<std::endl;
  for(size_t i=0; i<task.tasks.size(); i++){
    if(task.tasks[i].navigation_task.location.type.data == 4){
      
      //move_base_msgs::MoveBaseGoal goal;
      //geometry_msgs::PoseStamped goal_pose;
      geometry_msgs::Pose goal_pose;
      goal_pose.position.x = 2.0;
      goal_pose.position.y = 2.0;
      goal_pose.orientation.w = 1.0;
      
      location_goals.push_back(goal_pose);
      pose_array.poses = location_goals;
      poses_queue.publish(pose_array);
      //to defined a pose
    }
  }
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "task_sender");
  ros::NodeHandle nh;
  ROS_INFO("CFH Robot example is running");
  
  ros::Subscriber logging_status_sub_ = nh.subscribe("/robot_example_ros/task_info", 1000, taskInfoCallback);
  poses_queue = nh.advertise<geometry_msgs::PoseArray>("/goal_queue_goals", 1000);
  ros::spin();

  return 0;
}
