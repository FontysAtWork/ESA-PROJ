#include <ros/ros.h>

#include <atwork_ros_msgs/TaskInfo.h>
#include <atwork_ros_msgs/Task.h>
#include <atwork_ros_msgs/NavigationTask.h>
#include <atwork_ros_msgs/ObjectIdentifier.h>
#include <atwork_ros_msgs/TransportationTask.h>
#include <atwork_ros_msgs/LocationIdentifier.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <task_executor/TaskAction.h>

#include <string>
#include <vector>

std::vector< atwork_ros_msgs::Task > tasks;

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const task_executor::TaskResultConstPtr& result) {
	for(unsigned int i = 0; i < tasks.size(); i++) {
		if ((int) tasks[i].id.data == result->id) {
			tasks[i].status.data = result->status;
		}
	}
}

// Called once when the goal becomes active
void activeCb() {
	ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const task_executor::TaskFeedbackConstPtr& feedback) {
	for(unsigned int i = 0; i < tasks.size(); i++) {
		if ((int) tasks[i].id.data == feedback->id) {
			tasks[i].status.data = feedback->status;
		}
	}
}

// Add tasks to vector
void TaskCallback(const atwork_ros_msgs::TaskInfoConstPtr& msg) {
	for(int i = msg->tasks.size() - 1; i >= 0; i--) {
	//for(int i = 0; i < msg->tasks.size(); i++) {
		bool exists = false;
		

		for(unsigned int j = 0; j < tasks.size(); j++) {
			if(msg->tasks[i].id.data == tasks[j].id.data) {
				exists = true;
			}
		}

		if (!exists) {
			tasks.push_back(msg->tasks[i]);
		}
	}
}

void SendGoals(actionlib::SimpleActionClient<task_executor::TaskAction> * ac) {
	for(unsigned int i = 0; i < tasks.size(); i++) {
		if(tasks[i].status.data == 1) {
			
			// send a goal to the action
			tasks[i].status.data = 7;
			task_executor::TaskGoal goal;
			goal.task = tasks[i];
			ac->sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
			ROS_INFO("Sent task %d to action server", (int) tasks[i].id.data);

			//wait for the action to return
			bool finished_before_timeout = ac->waitForResult(ros::Duration(900.0));

			if (!finished_before_timeout) {
				tasks[i].status.data = 2;
				ROS_WARN("Action did not finish before the time out.");
			}
		}

	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "master");
	ros::NodeHandle n;
	ros::Rate loop_rate(1);

	// subsribe to refbox_receiver to receive tasks
	ros::Subscriber sub = n.subscribe("/refbox_receiver/task_info", 1000, TaskCallback);

	// create the action client for tasks
	actionlib::SimpleActionClient<task_executor::TaskAction> ac("Task", true);
	
	// wait for the action server to start
	ROS_INFO("Waiting for action server to start.");
	ac.waitForServer();
	ROS_INFO("Action server started.");

	while (ros::ok()) {
		SendGoals(&ac);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
