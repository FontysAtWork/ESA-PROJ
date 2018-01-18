#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <task_executor/TaskAction.h>
#include <task_executor/LocationIdentifier.hpp>
#include <task_executor/ObjectIdentifier.hpp>

#include <atwork_ros_msgs/TaskInfo.h>
#include <atwork_ros_msgs/Task.h>
#include <atwork_ros_msgs/NavigationTask.h>
#include <atwork_ros_msgs/ObjectIdentifier.h>
#include <atwork_ros_msgs/TransportationTask.h>
#include <atwork_ros_msgs/LocationIdentifier.h>

#include <nav_lib/Marker.hpp>
#include <nav_lib/Nav.hpp>

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>

#include <vector>
#include <string>
#include <stdio.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
class TaskAction
{
  private:
	LocationIdentifier location;
	int orientation;
	ros::Time waitTime;
	int taskID;
	std::vector<Marker> markers;
	MoveBaseClient move_base_ac;//("move_base", true);
	
  protected:
	ros::NodeHandle node;
	actionlib::SimpleActionServer<task_executor::TaskAction> actionServer; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
	std::string actionName;
	// create messages that are used to published feedback/result
	task_executor::TaskFeedback feedback;
	task_executor::TaskResult result;

  public:
	TaskAction(std::string name) : move_base_ac("move_base", true), actionServer(node, name, boost::bind(&TaskAction::executeCB, this, _1), false),
										actionName(name) {
		actionServer.start();
		ROS_INFO("Task actionserver started");
	}

	~TaskAction(void) {
	}

	void executeCB(const task_executor::TaskGoalConstPtr &goal) {
		ros::Rate loop_rate(1000);
		ROS_INFO("Got a callback!!!");

		feedback.status = 3; // 3 = in progress (see atwork_ros_msgs/Task.msg)
		feedback.id = taskID;
		
		// start executing the action
		TaskParser(goal->task);
		
		actionServer.publishFeedback(feedback);

		char locationname[8];

		if(location.GetType() == LocationType::SH) {
			sprintf(locationname, "SH%02d", location.GetInstanceId());
		} else if(location.GetType() == LocationType::WS) {
			sprintf(locationname, "WS%02d", location.GetInstanceId());
		} else if(location.GetType() == LocationType::CB) {
			sprintf(locationname, "CB%02d", location.GetInstanceId());
		} else if(location.GetType() == LocationType::WP) {
			sprintf(locationname, "WP%02d", location.GetInstanceId());
		} else if(location.GetType() == LocationType::PP) {
			sprintf(locationname, "PP%02d", location.GetInstanceId());
		} else {
			sprintf(locationname, "ROBOT");
		}

		std::string locname = locationname;
		ROS_INFO("Trying to find %s", locationname);

		bool found = false;
		for(auto m : markers) {
			if(locname == m.GetName()) {
				ROS_INFO("Found marker in list");
				MoveRobotToMarker(m);
				found = true;
				continue;
			}
		}

		if(!found) {
			result.status = 5; // 5 = aborted (see atwork_ros_msgs/Task.msg)
			ROS_WARN("Marker not found in list");
		} else {
			result.status = 6;  // 6 = finished (see atwork_ros_msgs/Task.msg)
			ROS_INFO("Waiting for %f secs", waitTime.toSec());
			ros::Duration(waitTime.toSec()).sleep();
		}

		result.id = taskID;
		actionServer.setSucceeded(result);
	}

	void MoveRobotToMarker(Marker m) {
		geometry_msgs::Quaternion qM;
		geometry_msgs::Quaternion qM1;
		tf::Quaternion qT;

		if(orientation == 1) {
			// NORTH
			qM = m.SetQuaternation(0, 0, 0);

		} else if (orientation == 2) {
			// EAST
			qM = m.SetQuaternation(0, 0, m.Deg2Rad(90));

		} else if (orientation == 3) {
			// SOUTH
			qM = m.SetQuaternation(0, 0, m.Deg2Rad(180));
			
		} else {
			// WEST
			qM = m.SetQuaternation(0, 0, m.Deg2Rad(270));
		}
		
		tf::quaternionMsgToTF(qM, qT);
		qT.normalize();
		tf::quaternionTFToMsg(qT, qM1);

		move_base_msgs::MoveBaseGoal g;
		g.target_pose.header.stamp = ros::Time::now();
		g.target_pose.header.frame_id = "map";
		g.target_pose.pose = m.GetPose();
		g.target_pose.pose.orientation = qM1;
		
		move_base_ac.sendGoal(g);

		ROS_INFO("Movebase goal sent");

		move_base_ac.waitForResult();

		bool finished_before_timeout = move_base_ac.waitForResult(ros::Duration(900.0));

		if (!finished_before_timeout) {
			result.status = 2; // 2 = timeout (see atwork_ros_msgs/Task.msg)
			ROS_WARN("Action did not finish before the time out.");
		}

		if (move_base_ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_ERROR("The base failed to move");
			result.status = 5; // 5 = aborted (see atwork_ros_msgs/Task.msg)
		}
	}

	void ReadMarkerFile(std::string filename) {
		markers = NAV::LoadMarkers(filename);
		if(markers.size() == 0)
		{
			ROS_WARN("No markers found!");
			ros::shutdown();
		}
	}
	
	void TaskParser(atwork_ros_msgs::Task t) {
		int task_id = t.id.data;
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
			
			/*
			std::cout << "Dst: " << location.Print() << std::endl;
			std::cout << "Ori: " << orientation << std::endl;
			std::cout << "Tim: " << waitTime << std::endl;
			std::cout << "ID: " << taskID << std::endl << std::endl;
			*/
			
		} else {
			ROS_WARN("Unknown task :o");
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Task");
	ros::NodeHandle n;
	TaskAction task("Task");
	std::string fileName;
	n.getParam("/Task/MarkerFile", fileName);
	task.ReadMarkerFile(fileName);
	
	ros::spin();

	return 0;
}








/*while (!move_base_ac.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the move_base action server to come up");
	}*/
/*
	std::vector<move_base_msgs::MoveBaseGoal> goals = createNavPoints();

	ROS_INFO("Sending goals");

	for (auto g : goals)
	{
		g.target_pose.header.stamp = ros::Time::now();
		move_base_ac.sendGoal(g);
		move_base_ac.waitForResult();

		if (move_base_ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_WARN("The base failed to move");
		}
	}
	*/
