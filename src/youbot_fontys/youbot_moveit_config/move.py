#!/usr/bin/env python 
import sys
import copy
import rospy
import moveit_commander 
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint
import math
from geometry_msgs.msg import Pose, Point, Quaternion 
import actionlib 
from tf import *

class ArmMoveit():

  def move_group_python_interface():

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface',
                    anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    

  def set_home():
      group = moveit_commander.MoveGroupCommander("arm_1")
      group.clear_pose_targets()
      group_variable_values = group.get_current_joint_values()
      
      group_variable_values[0] = 0.2
      group_variable_values[1] = 0.2
      group_variable_values[2] = -0.2
      group_variable_values[3] = 0.2
      group_variable_values[4] = 0.2

      group.set_joint_value_target(group_variable_values)
      plansethome = group.plan()
      group.execute(plansethome)

#--TODO--
#--Connect to vision node
#--Extract object coordinate
#--Implement the extraction to this code
#--URDF gripper
#--Make the gripper work


  def pos_the_robot(pos_x, pos_y, pos_z, rot_x, rot_y, rot_z):
    group = moveit_commander.MoveGroupCommander("arm_1")
    group.clear_pose_targets()
    quaternion = transformations.quaternion_from_euler(rot_x, rot_y, rot_z)
    

    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = pos_x
    pose_target.position.y = pos_y
    pose_target.position.z = pos_z

    pose_target.orientation.x = quaternion[0]
    pose_target.orientation.y = quaternion[1]
    pose_target.orientation.z = quaternion[2]
    pose_target.orientation.w = quaternion[3]

    group.set_pose_target(pose_target)
    plansetpos = group.plan()
    group.execute(plansetpos)
    rospy.sleep(2)

  if __name__=='__main__':
    try:
      move_group_python_interface()
      set_home()
#----------------<MOVIT_COORIDNATE>----------------#
      pos_x = 0.1
      pos_y = 0.1
      pos_z = 0.4

      rot_x = math.radians(0)
      rot_y = math.radians(0)
      rot_z = math.radians(0)

      pos_the_robot(pos_x, pos_y, pos_z, rot_x, rot_y, rot_z)
#---------------</MOVIT_COORIDNATE/>---------------#

    except rospy.ROSInterruptException:
      pass

