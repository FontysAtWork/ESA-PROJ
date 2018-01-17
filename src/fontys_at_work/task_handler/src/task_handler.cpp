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
	ROS_INFO("Finished in state [%s]", state.toString().c_str());
	ROS_INFO("Answer: %d", result->status);

	for(auto t : tasks) {
		if (t.id.data == result->id) {
			t.status.data = result->status;
		}
	}

	ros::shutdown();
}

// Called once when the goal becomes active
void activeCb() {
	ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const task_executor::TaskFeedbackConstPtr& feedback) {
	ROS_INFO("Got Feedback of length %d", feedback->status);

	for(auto t : msg->tasks) {
		if (t.id.data == result->id) {
			t.status.data = result->status;
		}
	}
}

// Add tasks to vector
void TaskCallback(const atwork_ros_msgs::TaskInfoConstPtr& msg) {
	for(auto t : tasks) {
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

void SendGoals(actionlib::SimpleActionClient<task_executor::TaskAction> * ac) {
	for(auto t : tasks) {
		if(t.status.data == 0) {
			// send a goal to the action
			task_executor::TaskGoal goal;
			goal.task = t;
			ac->sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
			ROS_INFO("Sent task %d to as", (int) t.id.data);
		} else {
			ROS_INFO("Task %d already sent", (int) t.id.data);
		}
	}
	std::cout << tasks.size() << std::endl;

/*

	//wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(180.0));

	if (finished_before_timeout) {
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
	}
	else {
		ROS_INFO("Action did not finish before the time out.");
	}*/
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
