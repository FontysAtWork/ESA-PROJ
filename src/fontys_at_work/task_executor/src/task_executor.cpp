#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <task_executor/TaskAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <atwork_ros_msgs/Task.h>
#include <task_executor/LocationIdentifier.hpp>
#include <task_executor/ObjectIdentifier.hpp>

#include <atwork_ros_msgs/TaskInfo.h>
#include <atwork_ros_msgs/Task.h>
#include <atwork_ros_msgs/NavigationTask.h>
#include <atwork_ros_msgs/ObjectIdentifier.h>
#include <atwork_ros_msgs/TransportationTask.h>
#include <atwork_ros_msgs/LocationIdentifier.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
class TaskAction
{
	
  private:
	
	
	LocationIdentifier location;
	int orientation;
	ros::Time waitTime;
	int taskID;
	
  protected:
    ros::NodeHandle node;
    actionlib::SimpleActionServer<task_executor::TaskAction> actionServer; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string actionName;
    // create messages that are used to published feedback/result
    task_executor::TaskFeedback feedback;
    task_executor::TaskResult result;

  public:
    TaskAction(std::string name) : actionServer(node, name, boost::bind(&TaskAction::executeCB, this, _1), false),
                                        actionName(name)
    {
        actionServer.start();
        ROS_INFO("started");
    }

    ~TaskAction(void)
    {
    }

    void executeCB(const task_executor::TaskGoalConstPtr &goal)
    {
        // helper variables
        ros::Rate r(1);
        bool success = true;

        // push_back the seeds for the Task sequence
        //feedback_.sequence.clear();
        //feedback_.sequence.push_back(0);
        //feedback_.sequence.push_back(1);
		feedback.status = 0;
        // publish info to the console for the user
        //ROS_INFO("%s: Executing, creating Task sequence of order %i with seeds %i, %i", actionName.c_str(), goal->order, feedback.sequence[0], feedback.sequence[1]);

        // start executing the action
        //TaskParser(goal->task);
        
        ROS_INFO("Got a callback!!!");
        
        /*for (int i = 1; i <= goal->order; i++)
        {
            // check that preempt has not been requested by the client
            if (actionServer.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", actionName.c_str());
                // set the action state to preempted
                actionServer.setPreempted();
                success = false;
                break;
            }
            
            //feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i - 1]);
            // publish the feedback
            actionServer.publishFeedback(feedback);
            // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep();
        }*/

        if (success)
        {
            result.status = feedback.status;
            //ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            //as_.setSucceeded(result_);
        }
    }
    
    void TaskParser(atwork_ros_msgs::Task t) {
	int task_id = t.id.data;
	//int task_status = t.status.data;
	int task_type = t.type.data;

	ROS_INFO("Added task(%d)", task_id);

	if(task_type == atwork_ros_msgs::Task::TRANSPORTATION) {
		/*atwork_ros_msgs::TransportationTask transportation_task = t.transportation_task;

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
		std::cout << "Sts: " << task_status << std::endl << std::endl;*/
		
		ROS_WARN("Transportation task has not yet been implemented.");

	} else if(task_type == atwork_ros_msgs::Task::NAVIGATION) {
		location.SetLocation(t.navigation_task.location);
		orientation = t.navigation_task.orientation.data;
		waitTime = t.navigation_task.wait_time.data;
		taskID = t.id.data;
		
		std::cout << "Dst: " << location.Print() << std::endl;
		std::cout << "Ori: " << orientation << std::endl;
		std::cout << "Tim: " << waitTime << std::endl;
		std::cout << "ID: " << taskID << std::endl << std::endl;
		
	} else {
		ROS_WARN("Unknown task :o");
	}
}
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Task");

    MoveBaseClient ac("move_base", true);
    TaskAction task("Task");

    /*while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }*/
/*
    std::vector<move_base_msgs::MoveBaseGoal> goals = createNavPoints();

    ROS_INFO("Sending goals");

    for (auto g : goals)
    {
        g.target_pose.header.stamp = ros::Time::now();
        ac.sendGoal(g);
        ac.waitForResult();

        if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_WARN("The base failed to move");
        }
    }
    */
    ROS_INFO("hallo");
    ros::spin();

    return 0;
}
