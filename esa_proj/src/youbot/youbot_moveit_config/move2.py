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
from moveit_msgs.msg import MoveItErrorCodes


class ArmMoveit():

  def move_group_python_interface():

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface',
                    anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

#--TODO--
#--Connect to vision node
#--Extract object coordinate
#--Implement the extraction to this code
#--URDF gripper
#--Make the gripper work


  def pos_the_robot(pos_x, pos_y, pos_z, rot_x, rot_y, rot_z):
    quaternion = transformations.quaternion_from_euler(rot_x, rot_y, rot_z)
    group = moveit_commander.MoveGroupCommander("arm_1")

    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = pos_x + 0.2
    pose_target.position.y = pos_y + 0.1
    pose_target.position.z = pos_z + 0.5

    pose_target.orientation.x = quaternion[0]
    pose_target.orientation.y = quaternion[1]
    pose_target.orientation.z = quaternion[2]
    pose_target.orientation.w = quaternion[3]

    group.set_pose_target(pose_target)
    plansetpos = group.plan()
    group.execute(plansetpos)
    rospy.sleep(1)
    place_result = place(name, locations, **kwargs)

  def cart_move(delta_x, delta_y, delta_z, d_rot_x, d_rot_y, d_rot_z):
    quaternion = transformations.quaternion_from_euler(d_rot_x, d_rot_y, d_rot_z)
    group = moveit_commander.MoveGroupCommander("arm_1")

    newpose = group.get_current_pose().pose

    newpose.position.x = newpose.position.x + delta_x
    newpose.position.y = newpose.position.y + delta_y
    newpose.position.z = newpose.position.z + delta_z

    newpose.orientation.x = quaternion[0]
    newpose.orientation.y = quaternion[1]
    newpose.orientation.z = quaternion[2]
    newpose.orientation.w = quaternion[3]

    group.set_pose_target(newpose)
    planned = group.plan()
    group.execute(planned)
    rospy.sleep(1)
    place_result = place(name, locations, **kwargs)

  if __name__=='__main__':
    try:
      move_group_python_interface()
#----------------<MOVIT_COORIDNATE>----------------#
      pos_x = 0
      pos_y = 0
      pos_z = 0

      rot_x = math.radians(0)
      rot_y = math.radians(0)
      rot_z = math.radians(0)
      print "============ executing 1"
      pos_the_robot(pos_x, pos_y, pos_z, rot_x, rot_y, rot_z)
      if place_result.error_code.val == MoveItErrorCodes.PLANNING_FAILED:
        print "============ executing 1 again"
        pos_the_robot(pos_x, pos_y, pos_z, rot_x, rot_y, rot_z)
#---------------</MOVIT_COORIDNATE/>---------------#

#----------------<MOVIT_CARTESIAN>-----------------#
      delta_x = 0
      delta_y = 0.2
      delta_z = -0.15

      d_rot_x = math.radians(0)
      d_rot_y = math.radians(120)
      d_rot_z = math.radians(90)
      print "============ executing 2"
      cart_move(delta_x, delta_y, delta_z, d_rot_x, d_rot_y, d_rot_z)
      if place_result.error_code.val == MoveItErrorCodes.PLANNING_FAILED:
        print "============ executing 2 again"
        cart_move(delta_x, delta_y, delta_z, d_rot_x, d_rot_y, d_rot_z)
#----------------</MOVIT_CARTESIAN/>-----------------#

#----------------<MOVIT_CARTESIAN>-----------------#
      delta_x = 0
      delta_y = 0.2
      delta_z = -0.1

      d_rot_x = math.radians(0)
      d_rot_y = math.radians(140)
      d_rot_z = math.radians(90)
      print "============ executing 3"
      cart_move(delta_x, delta_y, delta_z, d_rot_x, d_rot_y, d_rot_z)
      if place_result.error_code.val == MoveItErrorCodes.PLANNING_FAILED:
        print "============ executing 3 again"
        cart_move(delta_x, delta_y, delta_z, d_rot_x, d_rot_y, d_rot_z)
#----------------</MOVIT_CARTESIAN/>-----------------#

#----------------<MOVIT_COORIDNATE>----------------#
      pos_x = -0.2
      pos_y = 0
      pos_z = -0.1

      rot_x = math.radians(0)
      rot_y = math.radians(0)
      rot_z = math.radians(0)

      print "============ executing 4"
      pos_the_robot(pos_x, pos_y, pos_z, rot_x, rot_y, rot_z)
      if place_result.error_code.val == MoveItErrorCodes.PLANNING_FAILED:
        print "============ executing 4 again"
        pos_the_robot(pos_x, pos_y, pos_z, rot_x, rot_y, rot_z)
#---------------</MOVIT_COORIDNATE/>---------------#

#----------------<MOVIT_CARTESIAN>-----------------#
      delta_x = 0
      delta_y = 0
      delta_z = 0

      d_rot_x = math.radians(0)
      d_rot_y = math.radians(-90)
      d_rot_z = math.radians(0)
      print "============ executing 5"
      cart_move(delta_x, delta_y, delta_z, d_rot_x, d_rot_y, d_rot_z)
      if place_result.error_code.val == MoveItErrorCodes.PLANNING_FAILED:
        print "============ executing 5 again"
        cart_move(delta_x, delta_y, delta_z, d_rot_x, d_rot_y, d_rot_z)
#----------------</MOVIT_CARTESIAN/>-----------------#

    except rospy.ROSInterruptException:
      pass
#http://docs.ros.org/jade/api/moveit_python/html/pick__place__interface_8py_source.html

