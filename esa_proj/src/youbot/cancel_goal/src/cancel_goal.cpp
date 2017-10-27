#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <actionlib_msgs/GoalID.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

ros::Publisher pubVelCmd;
ros::Publisher pubActionLibCancel;

geometry_msgs::Pose currentPose;
geometry_msgs::Pose goalPose;


double getAngleBetweenPoses2D(geometry_msgs::Pose p1, geometry_msgs::Pose p2) {
    float dx = p2.position.x - p1.position.x;
    float dy = p2.position.y - p1.position.y;
    return (atan2(dy, dx));
}

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

    double angle = getAngleBetweenPoses2D(currentPose, goalPose);
	double distance = getDistBetweenPoses2D(currentPose, goalPose);
    
    if (distance < 0.5 && angle < 10.0) {
        actionlib_msgs::GoalID kill_msg;
        geometry_msgs::Twist vel_msg;        

        pubActionLibCancel.publish(kill_msg);
        pubVelCmd.publish(vel_msg);
    }
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "cancel_goal");
	ros::NodeHandle n;
    ros::Subscriber subGoal = n.subscribe("/goal", 100, cbGoal);
    ros::Subscriber subOdom = n.subscribe("/odom", 100, cbOdom);
    pubVelCmd = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    pubActionLibCancel = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 100);
    
	ros::spin();

	return 0;
}
