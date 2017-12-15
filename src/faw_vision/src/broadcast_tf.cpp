#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

double deg2rad(double deg) {
    return (deg*M_PI/180);
}

tf::Transform transform;

void cbPose(const geometry_msgs::Pose::ConstPtr &msg);
void cbOdom(const nav_msgs::Odometry::ConstPtr &msg);

int main(int argc, char** argv){
  ros::init(argc, argv, "broadcast_tf");
  ros::NodeHandle n;
  ros::Subscriber subPath = n.subscribe("/tf_update", 100, cbPose);
  ros::Subscriber subOdom = n.subscribe("/odom", 100, cbOdom);

  tf::TransformBroadcaster tf_Broadcaster;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  tf::Quaternion q;
  //q.setRPY(deg2rad(-8.0), deg2rad(-30.0), 0);
  q.setRPY(0, 0, 0);
  transform.setRotation(q);

  ros::Rate rate(100);
  while(1){
    tf_Broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "camera_link"));
    rate.sleep();
    ros::spinOnce();
  }
  
  return 0;
};

void cbOdom(const nav_msgs::Odometry::ConstPtr &msg) {
  auto p = msg->pose.pose;
  transform.setOrigin(tf::Vector3(p.position.x, p.position.y, p.position.z));
  tf::Quaternion quat;
  //p.orientation.x += deg2rad(-8.0);
  //p.orientation.y += deg2rad(-30.0);
  
  tf::quaternionMsgToTF((p.orientation), quat);
  transform.setRotation(quat);
}

void cbPose(const geometry_msgs::Pose::ConstPtr &msg) {
  transform.setOrigin(tf::Vector3(msg->position.x, msg->position.y, msg->position.z));
  tf::Quaternion quat;
  tf::quaternionMsgToTF((msg->orientation), quat);
  transform.setRotation(quat);
}