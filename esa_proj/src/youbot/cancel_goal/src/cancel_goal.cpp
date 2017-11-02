#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <actionlib_msgs/GoalID.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

ros::Publisher pubVelCmd;
ros::Publisher pubActionLibCancel;

geometry_msgs::Pose currentPose;
geometry_msgs::Pose goalPose;


double getDistBetweenPoses2D(geometry_msgs::Pose p1, geometry_msgs::Pose p2) {
    float dx = p2.position.x - p1.position.x;
	float dy = p2.position.y - p1.position.y;
    return (sqrt(dx*dx+dy*dy));
}

double deg2rad(double deg) {
    return (deg*M_PI/180);
}

void cbGoal(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    goalPose = msg->pose;
}

void cbOdom(const nav_msgs::Odometry::ConstPtr &msg) {
    currentPose = msg->pose.pose;
    
    tf::Quaternion goal_(goalPose.orientation.x, 
						 goalPose.orientation.y, 
						 goalPose.orientation.z, 
						 goalPose.orientation.w);
	goal_ = goal_.normalize();

    double angle = fabs(tf::getYaw(goal_)-tf::getYaw(currentPose.orientation));
    //if (angle > M_PI) angle-= 2*M_PI;
    //if (angle < -M_PI) angle+= 2*M_PI;
    
    
	double distance = getDistBetweenPoses2D(currentPose, goalPose);
	
	//std::cout << "Distance: " << distance << std::endl;
	//std::cout << "angle: " << angle << std::endl;
    
    if (distance < 0.2 && angle < deg2rad(5.0)) {
        actionlib_msgs::GoalID kill_msg;
        geometry_msgs::Twist vel_msg;        

        pubActionLibCancel.publish(kill_msg);
        pubVelCmd.publish(vel_msg);
    }
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "cancel_goal");
	ros::NodeHandle n;
    ros::Subscriber subGoal = n.subscribe("move_base_simple/goal", 100, cbGoal);
    ros::Subscriber subOdom = n.subscribe("/odom", 100, cbOdom);
    pubVelCmd = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    pubActionLibCancel = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 100);
    
    goalPose.orientation.x = 0;
	goalPose.orientation.y = 0;
	goalPose.orientation.z = 0;
	goalPose.orientation.w = 1;
    
	ros::spin();

	return 0;
}
