#include <ros/ros.h>
#include <atwork_ros_msgs/TaskInfo.h>
#include <iostream>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
// This code publishes the goeometry pose array of the arena points
// The arena points are from the subscribing for this node to the task_info
// The coordinates of the arena point are from the data array ros param
// This is a test code using the 2 waypoints and 2 WS of the data array ros param
/*
# Location types
uint8    SH    = 1  # Shelf
uint8    WS    = 2  # Workstation
uint8    CB    = 3  # Conveyor belt
uint8    WP    = 4  # Way Point (additional navigation goal)
uint8    PP    = 5  # Precision Placement platform
uint8    ROBOT = 6
*/

std::vector<geometry_msgs::Pose> location_goals;


geometry_msgs::PoseArray pose_array;
ros::Publisher poses_queue;
bool is_found = false;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "task_sender");
  ros::NodeHandle nh;
  ROS_INFO("CFH Robot example is running");

  void taskInfoCallback(const atwork_ros_msgs::TaskInfo &task);
  
  ros::Subscriber logging_status_sub_ = nh.subscribe("/robot_example_ros/task_info", 1000, taskInfoCallback);
  poses_queue = nh.advertise<geometry_msgs::PoseArray>("/goal_queue_goals", 1000);
  ros::spin();

  return 0;
}

void taskInfoCallback(const atwork_ros_msgs::TaskInfo &task){
  ros::NodeHandle nh;
  // working station 1:
  geometry_msgs::Point_<std::allocator<void> >::_x_type wsp_1_x;
  geometry_msgs::Point_<std::allocator<void> >::_y_type wsp_1_y;
  geometry_msgs::Quaternion_<std::allocator<void> >::_z_type wsp_1_z;
  geometry_msgs::Quaternion_<std::allocator<void> >::_w_type wsp_1_w;
  // Get parametre working station 1:
  nh.getParam("/data_array/wsp_1/x", wsp_1_x);
  nh.getParam("/data_array/wsp_1/y", wsp_1_y);
  nh.getParam("/data_array/wsp_1/rotation_z", wsp_1_z);
  nh.getParam("/data_array/wsp_1/rotation_w", wsp_1_w);
  // working station 2:
  geometry_msgs::Point_<std::allocator<void> >::_x_type wsp_2_x;
  geometry_msgs::Point_<std::allocator<void> >::_y_type wsp_2_y;
  geometry_msgs::Quaternion_<std::allocator<void> >::_z_type wsp_2_z;
  geometry_msgs::Quaternion_<std::allocator<void> >::_w_type wsp_2_w;
  // Get parametre working station 2:
  nh.getParam("/data_array/wsp_2/x", wsp_2_x);
  nh.getParam("/data_array/wsp_2/y", wsp_2_y);
  nh.getParam("/data_array/wsp_2/rotation_z", wsp_2_z);
  nh.getParam("/data_array/wsp_2/rotation_w", wsp_2_w);
  // way point 1:
  geometry_msgs::Point_<std::allocator<void> >::_x_type nav_1_x;
  geometry_msgs::Point_<std::allocator<void> >::_y_type nav_1_y;
  geometry_msgs::Quaternion_<std::allocator<void> >::_z_type nav_1_z;
  geometry_msgs::Quaternion_<std::allocator<void> >::_w_type nav_1_w;
  // Get parametre way point 1:
  nh.getParam("/data_array/nav_1/x", nav_1_x);
  nh.getParam("/data_array/nav_1/y", nav_1_y);
  nh.getParam("/data_array/nav_1/rotation_z", nav_1_z);
  nh.getParam("/data_array/nav_1/rotation_w", nav_1_w);
  // way point 2:
  geometry_msgs::Point_<std::allocator<void> >::_x_type nav_2_x;
  geometry_msgs::Point_<std::allocator<void> >::_y_type nav_2_y;
  geometry_msgs::Quaternion_<std::allocator<void> >::_z_type nav_2_z;
  geometry_msgs::Quaternion_<std::allocator<void> >::_w_type nav_2_w;
  // Get parametre way point 2:
  nh.getParam("/data_array/nav_2/x", nav_2_x);
  nh.getParam("/data_array/nav_2/y", nav_2_y);
  nh.getParam("/data_array/nav_2/rotation_z", nav_2_z);
  nh.getParam("/data_array/nav_2/rotation_w", nav_2_w);

  std::cout << "location type " << task.tasks[0].transportation_task.source.type.data << " Description " << task.tasks[0].transportation_task.source.description.data <<std::endl;
  geometry_msgs::Pose goal_pose;
  for(size_t i=0; i<task.tasks.size(); i++){
    if(task.tasks[i].navigation_task.location.type.data == 2){
      //12 working stations at the competition
      switch (task.tasks[i].navigation_task.location.instance_id.data){
        case 1:
          goal_pose.position.x = wsp_1_x;
          goal_pose.position.y = wsp_1_y;
          goal_pose.orientation.z = wsp_1_z;
          goal_pose.orientation.w = wsp_1_w;
          is_found = true;
          break;
        case 2:
          goal_pose.position.x = wsp_2_x;
          goal_pose.position.y = wsp_2_y;
          goal_pose.orientation.z = wsp_2_z;
          goal_pose.orientation.w = wsp_2_w;
          is_found = true;
          break;
        case 3:
          goal_pose.position.x = wsp_2_x;
          goal_pose.position.y = wsp_2_y;
          goal_pose.orientation.z = wsp_2_z;
          goal_pose.orientation.w = wsp_2_w;
          is_found = true;
          break;
        case 4:
         goal_pose.position.x = wsp_2_x;
          goal_pose.position.y = wsp_2_y;
          goal_pose.orientation.z = wsp_2_z;
          goal_pose.orientation.w = wsp_2_w;
          is_found = true;
          break;
        case 5:
          goal_pose.position.x = wsp_2_x;
          goal_pose.position.y = wsp_2_y;
          goal_pose.orientation.z = wsp_2_z;
          goal_pose.orientation.w = wsp_2_w;
          is_found = true;
          break;
        case 6:
         goal_pose.position.x = wsp_2_x;
          goal_pose.position.y = wsp_2_y;
          goal_pose.orientation.z = wsp_2_z;
          goal_pose.orientation.w = wsp_2_w;
          is_found = true;
          break;
        case 7:
          goal_pose.position.x = wsp_2_x;
          goal_pose.position.y = wsp_2_y;
          goal_pose.orientation.z = wsp_2_z;
          goal_pose.orientation.w = wsp_2_w;
          is_found = true;
          break;
        case 8:
          goal_pose.position.x = wsp_2_x;
          goal_pose.position.y = wsp_2_y;
          goal_pose.orientation.z = wsp_2_z;
          goal_pose.orientation.w = wsp_2_w;
          is_found = true;
          break;
        case 9:
         goal_pose.position.x = nav_1_x;
          goal_pose.position.y = nav_1_y;
          goal_pose.orientation.z = nav_1_z;
          goal_pose.orientation.w = nav_1_w;
          is_found = true;
          break;
        case 10:
          goal_pose.position.x = wsp_2_x;
          goal_pose.position.y = wsp_2_y;
          goal_pose.orientation.z = wsp_2_z;
          goal_pose.orientation.w = wsp_2_w;
          is_found = true;
          break;
        case 11:
         goal_pose.position.x = nav_1_x;
          goal_pose.position.y = nav_1_y;
          goal_pose.orientation.z = nav_1_z;
          goal_pose.orientation.w = nav_1_w;
          is_found = true;
        break;
        case 12:
           goal_pose.position.x = nav_2_x;
           goal_pose.position.y = nav_2_y;
           goal_pose.orientation.z = nav_2_z;
           goal_pose.orientation.w = nav_2_w;
          is_found = true;
          break;
        default:
          is_found = false;
          break;
          // code to be executed if n doesn't match any constant
      }
    }
     
    else if(task.tasks[i].navigation_task.location.type.data == 4){
      //12 working stations at the competition
      switch (task.tasks[i].navigation_task.location.instance_id.data){
        case 1:
        goal_pose.position.x = wsp_2_x;
          goal_pose.position.y = wsp_2_y;
          goal_pose.orientation.z = wsp_2_z;
          goal_pose.orientation.w = wsp_2_w;
          is_found = true;
          break;
        case 2:
          goal_pose.position.x = nav_2_x;
          goal_pose.position.y = nav_2_y;
          goal_pose.orientation.z = nav_2_z;
          goal_pose.orientation.w = nav_2_w;
          is_found = true;
          break;
        case 3:
         goal_pose.position.x = nav_1_x;
          goal_pose.position.y = nav_1_y;
          goal_pose.orientation.z = nav_1_z;
          goal_pose.orientation.w = nav_1_w;
          is_found = true;
          break;
        case 4:
            goal_pose.position.x = nav_2_x;
  goal_pose.position.y = nav_2_y;
  goal_pose.orientation.z = nav_2_z;
  goal_pose.orientation.w = nav_2_w;
          is_found = true;
          break;
        case 5:
          goal_pose.position.x = nav_2_x;
  goal_pose.position.y = nav_2_y;
  goal_pose.orientation.z = nav_2_z;
  goal_pose.orientation.w = nav_2_w;
          is_found = false;
          break;
        case 6:
          goal_pose.position.x = nav_1_x;
          goal_pose.position.y = nav_1_y;
          goal_pose.orientation.z = nav_1_z;
          goal_pose.orientation.w = nav_1_w;
          is_found = true;
          break;
        case 7:
           goal_pose.position.x = nav_2_x;
  goal_pose.position.y = nav_2_y;
  goal_pose.orientation.z = nav_2_z;
  goal_pose.orientation.w = nav_2_w;
          is_found = true;
          break;
        case 8:
         goal_pose.position.x = wsp_2_x;
          goal_pose.position.y = wsp_2_y;
          goal_pose.orientation.z = wsp_2_z;
          goal_pose.orientation.w = wsp_2_w;
          is_found = true;
          break;
        case 9:
          goal_pose.position.x = nav_2_x;
  goal_pose.position.y = nav_2_y;
  goal_pose.orientation.z = nav_2_z;
  goal_pose.orientation.w = nav_2_w;
          is_found = true;
          break;
        case 10:
          goal_pose.position.x = nav_1_x;
          goal_pose.position.y = nav_1_y;
          goal_pose.orientation.z = nav_1_z;
          goal_pose.orientation.w = nav_1_w;
          is_found = true;
          break;
        case 11:
         goal_pose.position.x = nav_2_x;
  goal_pose.position.y = nav_2_y;
  goal_pose.orientation.z = nav_2_z;
  goal_pose.orientation.w = nav_2_w;
          is_found = true;
        break;
        case 12:
           goal_pose.position.x = nav_2_x;
  goal_pose.position.y = nav_2_y;
  goal_pose.orientation.z = nav_2_z;
  goal_pose.orientation.w = nav_2_w;
          is_found = true;
          break;
	 case 13:
          goal_pose.position.x = nav_2_x;
  goal_pose.position.y = nav_2_y;
  goal_pose.orientation.z = nav_2_z;
  goal_pose.orientation.w = nav_2_w;
          is_found = false;
          break;
         case 14:
          goal_pose.position.x =nav_1_x;
          goal_pose.position.y=nav_1_y;
          goal_pose.orientation.z =nav_1_z;
          goal_pose.orientation.w =nav_1_w;
          is_found = true;
          break;
        default:
          is_found = false;
          break;
          // code to be executed if n doesn't match any constant
      }
    }

    else if(task.tasks[i].navigation_task.location.type.data == 1){
      // 2 shelfs at the competition
      switch (task.tasks[i].navigation_task.location.instance_id.data){
        case 1:
            goal_pose.position.x = nav_2_x;
  goal_pose.position.y = nav_2_y;
  goal_pose.orientation.z = nav_2_z;
  goal_pose.orientation.w = nav_2_w;
          is_found = true;
          break;
        case 2:
          goal_pose.position.x = wsp_2_x;
          goal_pose.position.y = wsp_2_y;
          goal_pose.orientation.z = wsp_2_z;
          goal_pose.orientation.w = wsp_2_w;
          is_found = true;
          break;
      
        default:
          is_found = false;
          break;
          // code to be executed if n doesn't match any constant
      }
    }
    else if(task.tasks[i].navigation_task.location.type.data == 3){
      // 1 conveyor belt at the competition
      switch (task.tasks[i].navigation_task.location.instance_id.data){
        case 1:
          goal_pose.position.x = nav_2_x;
  goal_pose.position.y = nav_2_y;
  goal_pose.orientation.z = nav_2_z;
  goal_pose.orientation.w = nav_2_w;
          is_found = true;
          break;
        case 2:
           goal_pose.position.x = nav_1_x;
          goal_pose.position.y = nav_1_y;
          goal_pose.orientation.z = nav_1_z;
          goal_pose.orientation.w = nav_1_w;
          is_found = true;
          break;
      
        default:
          is_found = false;
          break;
          // code to be executed if n doesn't match any constant
      }
    }
    
    else if(task.tasks[i].navigation_task.location.type.data == 5){
       // 1 precision platform in the competition
      switch (task.tasks[i].navigation_task.location.instance_id.data){
        case 1:
          goal_pose.position.x = nav_2_x;
  goal_pose.position.y = nav_2_y;
  goal_pose.orientation.z = nav_2_z;
  goal_pose.orientation.w = nav_2_w;
          is_found = true;
          break;
        default:
          is_found = false;
          break;
          // code to be executed if n doesn't match any constant
      }
    }
    
    else if(task.tasks[i].navigation_task.location.type.data == 1){
      
      switch (task.tasks[i].navigation_task.location.instance_id.data){
        case 1:
          goal_pose.position.x = nav_2_x;
  goal_pose.position.y = nav_2_y;
  goal_pose.orientation.z = nav_2_z;
  goal_pose.orientation.w = nav_2_w;
          is_found = true;
          break;
        case 2:
           goal_pose.position.x = nav_1_x;
          goal_pose.position.y = nav_1_y;
          goal_pose.orientation.z = nav_1_z;
          goal_pose.orientation.w = nav_1_w;
          is_found = true;
          break;
        default:
          is_found = false;
          break;
          // code to be executed if n doesn't match any constant
      }
    }
      
    if(is_found == true){
      location_goals.push_back(goal_pose);
      is_found = false;
    }
      //to defined a pose
  
  }
  
  
  goal_pose.position.x = nav_2_x;
  goal_pose.position.y = nav_2_y;
  goal_pose.orientation.z = nav_2_z;
  goal_pose.orientation.w = nav_2_w;
  location_goals.push_back(goal_pose);
  pose_array.poses = location_goals;
  poses_queue.publish(pose_array);
}




