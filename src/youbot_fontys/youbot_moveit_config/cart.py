#!/usr/bin/env python

import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander, conversions
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import RobotTrajectory, Grasp


if __name__=='__main__':

    rospy.init_node('moveit_py_demo', anonymous=True)

    scene = PlanningSceneInterface()

    robot = MoveGroupCommander("arm_1");
    rospy.sleep(1)
   
    currentrf = robot.get_pose_reference_frame()
    robot.set_pose_reference_frame(currentrf)
    newpose = robot.get_current_pose().pose
    a = robot.get_current_pose().pose
    newposerpy = robot.get_current_rpy()

    newpose.position.x = newpose.position.x - delta_x
    newpose.position.x = newpose.position.y - delta_y
    newpose.position.x = newpose.position.z - delta_z
    
    robot.set_pose_target(newpose)
    planned = robot.plan()
    robot.execute(planned)
    rospy.sleep(2)

    newpose = robot.get_current_pose().pose

    robot.set_start_state_to_current_state()
    robot.set_pose_target(a)
    b = robot.compute_cartesian_path([newpose, a],0.01, 10000)
    pathplan = robot.plan(b[0])
    fraction = b[1]

    robot.execute(pathplan)

    finalpose = robot.get_current_pose().pose
    rospy.spin()

