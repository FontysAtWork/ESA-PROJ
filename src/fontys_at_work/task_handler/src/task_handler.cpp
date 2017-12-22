#include <ros/ros.h>

#include <atwork_ros_msgs/TaskInfo.h>
#include <atwork_ros_msgs/Task.h>
#include <atwork_ros_msgs/NavigationTask.h>
#include <atwork_ros_msgs/ObjectIdentifier.h>
#include <atwork_ros_msgs/TransportationTask.h>
#include <atwork_ros_msgs/LocationIdentifier.h>

#include <task_handler/LocationIdentifier.hpp>
#include <task_handler/ObjectIdentifier.hpp>

#include <string>
#include <vector>

std::vector< atwork_ros_msgs::Task > tasks;

void taskParser(atwork_ros_msgs::Task t) {
    int task_id = t.id.data;
    int task_status = t.status.data;
    int task_type = t.type.data;

    ROS_INFO("Added task(%d)", task_id);

    if(task_type == atwork_ros_msgs::Task::TRANSPORTATION) {
        atwork_ros_msgs::TransportationTask transportation_task = t.transportation_task;

        ObjectIdentifier trans_object(transportation_task.object);
        ObjectIdentifier trans_container(transportation_task.container);

        int trans_quantity_delivered = transportation_task.quantity_delivered.data;
        int trans_quantity_requested = transportation_task.quantity_requested.data;

        LocationIdentifier trans_dst(transportation_task.destination);
        LocationIdentifier trans_src(transportation_task.source);

        std::string trans_team = transportation_task.processing_team.data;

        
        std::cout << "Obj: " << trans_object.Print() << std::endl;
        std::cout << "Ctn: " << trans_container.Print() << std::endl;
        std::cout << "Qtd: " << trans_quantity_delivered << std::endl;
        std::cout << "Qtr: " << trans_quantity_requested << std::endl;
        std::cout << "Dst: " << trans_dst.Print() << std::endl;
        std::cout << "Src: " << trans_src.Print() << std::endl;
        std::cout << "Tea: " << trans_team << std::endl;
        std::cout << "Sts: " << task_status << std::endl << std::endl;
        
        ROS_WARN("TODO: IMPLEMENT ACTIONLIB CLIENT");
        // DO STUFF
    } else if(task_type == atwork_ros_msgs::Task::NAVIGATION) {
        atwork_ros_msgs::NavigationTask navigation_task = t.navigation_task;

        LocationIdentifier nav_location(navigation_task.location);

        int nav_orientation = navigation_task.orientation.data; //NORTH=1,EAST=2,SOUTH=3,WEST=4
        ros::Time nav_wait_time = navigation_task.wait_time.data;

        
        std::cout << "Dst: " << nav_location.Print() << std::endl;
        std::cout << "Ori: " << nav_orientation << std::endl;
        std::cout << "Tim: " << nav_wait_time << std::endl;
        std::cout << "Sts: " << task_status << std::endl << std::endl;
        
        ROS_WARN("TODO: IMPLEMENT ACTIONLIB CLIENT");
        // DO STUFF
    } else {
        ROS_WARN("Unknown task :o");
    }
}

void taskCallback(const atwork_ros_msgs::TaskInfoConstPtr& msg) {
    for(auto t : msg->tasks) {
        bool exists = false;
        for(auto task : tasks) {
            if(t.id.data == task.id.data) {
                exists = true;
            }
        }
        if (!exists) {
            tasks.push_back(t);
        }
    }
}

void doStuff() {
    for(auto t : tasks) {
        taskParser(t);
    }
    std::cout << tasks.size() << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "master");
    ros::NodeHandle n;
    ros::Rate loop_rate(10); // one Hz

    ros::Subscriber sub = n.subscribe("/refbox_receiver/task_info", 1000, taskCallback);

    while (ros::ok()) {
        doStuff();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
