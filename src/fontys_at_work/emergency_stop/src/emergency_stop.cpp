#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"

std_msgs::Bool emergency;
ros::Publisher twistPublisher;
geometry_msgs::Twist stop;


void emergencyCallback(const std_msgs::Bool& stop_bool){
    
    if(stop_bool.data && !emergency.data){
        emergency.data = true;

        twistPublisher.publish(stop);
    }
    }

void twistCallback(const geometry_msgs::Twist& twist){

    if(!emergency.data)
	twistPublisher.publish(twist);

}

int main(int argc, char **argv)
{

    /// Receives Twist messages for the base.
	ros::Subscriber baseCommandSubscriber;
	/// Publishes Odometry messages
	ros::Publisher baseOdometryPublisher;

	ros::init(argc, argv, "emgergency-stop");
	ros::NodeHandle n;
	/* setup input/output communication */
	
	emergency.data = false;

	stop.linear.x = 0;
	stop.linear.y = 0;
	stop.linear.z = 0;
	stop.angular.z = 0;

    ros::Subscriber twistSubscriber = n.subscribe("move_base/cmd_vel", 1000, &twistCallback);
    ros::Subscriber emergencySubscriber = n.subscribe("emergency_stop", 1000, &emergencyCallback);
	twistPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel",1);

	/* coordination */
	while (n.ok()){
		ros::spinOnce();
	}

  return 0;
}

