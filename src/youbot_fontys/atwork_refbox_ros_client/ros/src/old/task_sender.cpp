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
bool is_found = false;

void taskInfoCallback(const atwork_ros_msgs::TaskInfo &task){
  
  std::cout << "location type " << task.tasks[0].transportation_task.source.type.data << " Description " << task.tasks[0].transportation_task.source.description.data <<std::endl;
  geometry_msgs::Pose goal_pose;
  for(size_t i=0; i<task.tasks.size(); i++){
    if(task.tasks[i].transportation_task.source.type.data == 2){
      
      switch (task.tasks[i].transportation_task.source.instance_id.data){
        case 1:
          goal_pose.position.x = 6.135;
          goal_pose.position.y = -1.230;
          goal_pose.orientation.z = 0.477;
          goal_pose.orientation.w = 0.878;
          is_found = true;
          break;
        case 2:
          goal_pose.position.x = 0.659;
          goal_pose.position.y = -3.718;
          goal_pose.orientation.z = 0.7785;
          goal_pose.orientation.w = -0.627;
          is_found = true;
          break;
        case 3:
          goal_pose.position.x = 5.558;
          goal_pose.position.y = -2.265;
          goal_pose.orientation.z = 0.949;
          goal_pose.orientation.w = 0.313;
          is_found = true;
          break;
        case 4:
          goal_pose.position.x = 6.988;
          goal_pose.position.y = -2.369;
          goal_pose.orientation.z = -0.844;
          goal_pose.orientation.w = 0.534;
          is_found = true;
          break;
        case 5:
          goal_pose.position.x = 7.964;
          goal_pose.position.y = -3.647;
          goal_pose.orientation.z = -0.255;
          goal_pose.orientation.w = 0.966;
          is_found = true;
          break;
        case 6:
          goal_pose.position.x = 7.411;
          goal_pose.position.y = -4.989;
          goal_pose.orientation.z = -0.186;
          goal_pose.orientation.w = 0.982;
          is_found = true;
          break;
        case 7:
          goal_pose.position.x = 6.389;
          goal_pose.position.y = -6.056;
          goal_pose.orientation.z = 0.848;
          goal_pose.orientation.w = 0.528;
          is_found = true;
          break;
        case 8:
          goal_pose.position.x = 2.889;
          goal_pose.position.y = -3.403;
          goal_pose.orientation.z =-0.988;
          goal_pose.orientation.w = -0.135;
          is_found = true;
          break;
        case 9:
          goal_pose.position.x = 2.889;
          goal_pose.position.y = -3.403;
          goal_pose.orientation.z =-0.988;
          goal_pose.orientation.w = -0.140;
          is_found = true;
          break;
        case 10:
          goal_pose.position.x = 2.728;
          goal_pose.position.y = -4.206;
          goal_pose.orientation.z =-0.988;
          goal_pose.orientation.w = -0.140;
          is_found = true;
          break;
        case 11:
          goal_pose.position.x =2.728; 
          goal_pose.position.y =-4.207;
          goal_pose.orientation.z =-0.988;
          goal_pose.orientation.w =-0.140;
          is_found = true;
        break;
        case 12:
          goal_pose.position.x =1.616;
          goal_pose.position.y=-3.164;
          goal_pose.orientation.z =0.148;
          goal_pose.orientation.w =-0.988;
          is_found = true;
          break;
        default:
          is_found = false;
          break;
          // code to be executed if n doesn't match any constant
      }
    }

    else if(task.tasks[i].transportation_task.source.type.data == 1){
      
      switch (task.tasks[i].transportation_task.source.instance_id.data){
        case 1:
          goal_pose.position.x = 5.086;
          goal_pose.position.y = -0.541;
          goal_pose.orientation.z = 0.417;
          goal_pose.orientation.w = 0.988;
          is_found = true;
          break;
        case 2:
          //todo
          break;
      
        default:
          is_found = false;
          break;
          // code to be executed if n doesn't match any constant
      }
    }
    else if(task.tasks[i].transportation_task.source.type.data == 3){
      
      switch (task.tasks[i].transportation_task.source.instance_id.data){
        case 1:
          goal_pose.position.x = 0.822;
          goal_pose.position.y = -1.912;
          goal_pose.orientation.z = 0.955;
          goal_pose.orientation.w = 0.295;
          is_found = false;
          break;
        case 2:
          goal_pose.position.x = 1.730;
          goal_pose.position.y = -1.121;
          goal_pose.orientation.z = 0.962;
          goal_pose.orientation.w = 0.271;
          is_found = true;
          break;
      
        default:
          is_found = false;
          break;
          // code to be executed if n doesn't match any constant
      }
    }
    
    else if(task.tasks[i].transportation_task.source.type.data == 5){
      
      switch (task.tasks[i].transportation_task.source.instance_id.data){
        case 1:
          goal_pose.position.x = 3.921;
          goal_pose.position.y = -5.047;
          goal_pose.orientation.z = -0.870;
          goal_pose.orientation.w = 0.491;
          is_found = true;
          break;
        default:
          is_found = false;
          break;
          // code to be executed if n doesn't match any constant
      }
    }
    
    else if(task.tasks[i].transportation_task.source.type.data == 1){
      
      switch (task.tasks[i].transportation_task.source.instance_id.data){
        case 1:
          //todo
          break;
        case 2:
          goal_pose.position.x = 4.799;
          goal_pose.position.y = -5.815;
          goal_pose.orientation.z = -0.286;
          goal_pose.orientation.w = 0.958;
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
  
  
  goal_pose.position.x = 8.609;
  goal_pose.position.y = -1.837;
  goal_pose.orientation.z = -0.508;
  goal_pose.orientation.w = 0.860;
  location_goals.push_back(goal_pose);
  pose_array.poses = location_goals;
  poses_queue.publish(pose_array);
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
