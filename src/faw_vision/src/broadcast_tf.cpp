#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

tf::Transform transform;

void cbPose(const geometry_msgs::Pose::ConstPtr &msg);

int main(int argc, char** argv){
  ros::init(argc, argv, "broadcast_tf");
  ros::NodeHandle n;
  ros::Subscriber subPath = n.subscribe("/aids", 100, cbPose);

  tf::TransformBroadcaster tf_Broadcaster;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  tf::Quaternion q;
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


void cbPose(const geometry_msgs::Pose::ConstPtr &msg) {
  transform.setOrigin(tf::Vector3(msg->position.x, msg->position.y, msg->position.z));
  tf::Quaternion quat;
  tf::quaternionMsgToTF((msg->orientation), quat);
}