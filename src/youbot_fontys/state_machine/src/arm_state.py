#!/usr/bin/env python 
import rospy as rp
import sys
import tf
import moveit_commander 
import geometry_msgs.msg

def init():
    rp.init_node('arm_state', anonymous=False) 
    rp.set_param('arm_state', "arm_arm_stby")
    openGripper()
    rp.set_param('object_count', 0)
    armState()

def armState():
    while True:
        arm_state = rp.get_param('arm_state')
#        gripper_state = rp.get_param('gripper_state')
        if arm_state == "arm_arm_home":
            rp.loginfo("ARM_STATE  -->  home")
            setHome()
        elif arm_state == "arm_arm_prepare":
            rp.loginfo("ARM_STATE  -->  prepare")
            setPrepare()
            openGripper()
        elif arm_state == "arm_arm_scan":
            rp.loginfo("ARM_STATE  -->  scan")
            setScan()
        elif arm_state == "arm_arm_grasp_external":
            rp.loginfo("ARM_STATE  -->  grasp external")
            setGraspExternal()
        elif arm_state == "arm_arm_grasp_internal":
            rp.loginfo("ARM_STATE  -->  grasp internal")
            setGraspInternal()
        elif arm_state == "arm_arm_place_external":
            rp.loginfo("ARM_STATE  -->  place external")
            setPlacepExternal()
        elif arm_state == "arm_arm_place_internal":
            rp.loginfo("ARM_STATE  -->  place internal")
            setPlacepInternal()

def setHome():
    rp.set_param('arm_state', "arm_arm_moving")
    
    group = moveit_commander.MoveGroupCommander("arm_1")
    group.clear_pose_targets()
    group_variable_values = group.get_current_joint_values()
      
    group_variable_values[0] = 0.02
    group_variable_values[1] = 0.02
    group_variable_values[2] = -0.02
    group_variable_values[3] = 0.02
    group_variable_values[4] = 0.02
    
    group.set_joint_value_target(group_variable_values)
    plansethome = group.plan()
    group.execute(plansethome)
    rp.set_param('arm_state', "arm_arm_stby_home")

def setPrepare():
    rp.set_param('object_picked', False)
    rp.set_param('arm_state', "arm_arm_moving")
    
    group = moveit_commander.MoveGroupCommander("arm_1")
    group.clear_pose_targets()
    group_variable_values = group.get_current_joint_values()
      
    group_variable_values[0] = 2.947411422159712
    group_variable_values[1] = 2.2877879267043655
    group_variable_values[2] = -4.042161394184938
    group_variable_values[3] = 0.585455847150137
    group_variable_values[4] = 2.9563972410052988
    
    group.set_joint_value_target(group_variable_values)
    plansethome = group.plan()
    group.execute(plansethome)
    
    group = moveit_commander.MoveGroupCommander("arm_1")
    group.clear_pose_targets()
    group_variable_values = group.get_current_joint_values()
      
    group_variable_values[0] = 2.947411422159712
    group_variable_values[1] = 2.2877879267043655
    group_variable_values[2] = -4.042161394184938
    group_variable_values[3] = 0.585455847150137
    group_variable_values[4] = 2.9563972410052988
    
    group.set_joint_value_target(group_variable_values)
    plansethome = group.plan()
    group.execute(plansethome)
    
    rp.set_param('arm_state', "arm_arm_stby_prepare")

def setScan():
    rp.set_param('arm_state', "arm_arm_moving")
    
    group = moveit_commander.MoveGroupCommander("arm_1")
    group.clear_pose_targets()
    group_variable_values = group.get_current_joint_values()
      
    group_variable_values[0] = 1.4478057987802968
    group_variable_values[1] = 1.2673617240586994
    group_variable_values[2] = -1.8420025719124922
    group_variable_values[3] = 3.536168925123754
    group_variable_values[4] = 2.954386290852248
    
    group.set_joint_value_target(group_variable_values)
    plansethome = group.plan()
    group.execute(plansethome)
    
    group = moveit_commander.MoveGroupCommander("arm_1")
    group.clear_pose_targets()
    group_variable_values = group.get_current_joint_values()
      
    group_variable_values[0] = 1.4478057987802968
    group_variable_values[1] = 1.2673617240586994
    group_variable_values[2] = -1.8420025719124922
    group_variable_values[3] = 3.536168925123754
    group_variable_values[4] = 2.954386290852248
    
    group.set_joint_value_target(group_variable_values)
    plansethome = group.plan()
    group.execute(plansethome)
    
    rp.set_param('arm_state', "arm_arm_stby_scan")

def setGraspExternal():
    rp.set_param('arm_state', "arm_arm_moving")
    rp.get_param('')
    ls2 = tf.TransformListener()
    object_to_arm = ls2.lookupTransform('/object', '/arm_link_5', rp.Time(0))
    object_to_arm_pos = object_to_arm[0]
    object_to_arm_rot = object_to_arm[1]    
    
    group = moveit_commander.MoveGroupCommander("arm_1")
    pose = group.get_current_pose().pose
    pos_x = pose.position.x + object_to_arm_pos[0]
    pos_y = pose.position.y + object_to_arm_pos[1]
    pos_z = pose.position.z - object_to_arm_pos[2] + 0.07
    
    rot_x = object_to_arm_rot[0]
    rot_y = object_to_arm_rot[1]
    rot_z = object_to_arm_rot[2]
    rot_w = object_to_arm_rot[3]
    
    executeTrajectory(pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w)
    closeGripper()
    rp.set_param('object_picked', True)
    rp.set_param('arm_state', "arm_arm_stby_grasp_external")

def setGraspInternal():
    object_count = rp.get_param('object_count')
    rp.set_param('arm_state', "arm_arm_moving")
    
    if object_count == 0:
        group = moveit_commander.MoveGroupCommander("arm_1")
        group.clear_pose_targets()
        group_variable_values = group.get_current_joint_values()
          
        group_variable_values[0] = 3.59015543623787
        group_variable_values[1] = 0.9315389239350645
        group_variable_values[2] = -3.529899819624825
        group_variable_values[3] = 0.03523058948877694
        group_variable_values[4] = 2.954386290852248
        
        group.set_joint_value_target(group_variable_values)
        plansethome = group.plan()
        group.execute(plansethome)
        
        group = moveit_commander.MoveGroupCommander("arm_1")
        group.clear_pose_targets()
        group_variable_values = group.get_current_joint_values()
          
        group_variable_values[0] = 3.59015543623787
        group_variable_values[1] = 0.9315389239350645
        group_variable_values[2] = -3.529899819624825
        group_variable_values[3] = 0.03523058948877694
        group_variable_values[4] = 2.954386290852248
        
        group.set_joint_value_target(group_variable_values)
        plansethome = group.plan()
        group.execute(plansethome)
        
        group = moveit_commander.MoveGroupCommander("arm_1")
        group.clear_pose_targets()
        group_variable_values = group.get_current_joint_values()
          
        group_variable_values[0] = 3.5901538077515154
        group_variable_values[1] = 0.9378673553605125
        group_variable_values[2] = -3.8614650340162404
        group_variable_values[3] = 0.36060318985185114
        group_variable_values[4] = 2.954386290852248
        
        group.set_joint_value_target(group_variable_values)
        plansethome = group.plan()
        group.execute(plansethome)
        closeGripper()
        
        group = moveit_commander.MoveGroupCommander("arm_1")
        group.clear_pose_targets()
        group_variable_values = group.get_current_joint_values()
          
        group_variable_values[0] = 3.59015543623787
        group_variable_values[1] = 0.9315389239350645
        group_variable_values[2] = -3.529899819624825
        group_variable_values[3] = 0.03523058948877694
        group_variable_values[4] = 2.954386290852248
        
    elif object_count == 1:
        group = moveit_commander.MoveGroupCommander("arm_1")
        group.clear_pose_targets()
        group_variable_values = group.get_current_joint_values()
          
        group_variable_values[0] = 3.59015543623787 - 0.3
        group_variable_values[1] = 0.9315389239350645
        group_variable_values[2] = -3.529899819624825
        group_variable_values[3] = 0.03523058948877694
        group_variable_values[4] = 2.954386290852248
        
        group.set_joint_value_target(group_variable_values)
        plansethome = group.plan()
        group.execute(plansethome)
        
        group = moveit_commander.MoveGroupCommander("arm_1")
        group.clear_pose_targets()
        group_variable_values = group.get_current_joint_values()
          
        group_variable_values[0] = 3.59015543623787 - 0.3
        group_variable_values[1] = 0.9315389239350645
        group_variable_values[2] = -3.529899819624825
        group_variable_values[3] = 0.03523058948877694
        group_variable_values[4] = 2.954386290852248
        
        group.set_joint_value_target(group_variable_values)
        plansethome = group.plan()
        group.execute(plansethome)
        
        group = moveit_commander.MoveGroupCommander("arm_1")
        group.clear_pose_targets()
        group_variable_values = group.get_current_joint_values()
          
        group_variable_values[0] = 3.5901538077515154 - 0.3
        group_variable_values[1] = 0.9378673553605125
        group_variable_values[2] = -3.8614650340162404
        group_variable_values[3] = 0.36060318985185114
        group_variable_values[4] = 2.954386290852248
        
        group.set_joint_value_target(group_variable_values)
        plansethome = group.plan()
        group.execute(plansethome)
        closeGripper()
        
        group = moveit_commander.MoveGroupCommander("arm_1")
        group.clear_pose_targets()
        group_variable_values = group.get_current_joint_values()
          
        group_variable_values[0] = 3.59015543623787 - 0.3
        group_variable_values[1] = 0.9315389239350645
        group_variable_values[2] = -3.529899819624825
        group_variable_values[3] = 0.03523058948877694
        group_variable_values[4] = 2.954386290852248
        
        group.set_joint_value_target(group_variable_values)
        plansethome = group.plan()
        group.execute(plansethome)
        
    elif object_count == 2:   
        group = moveit_commander.MoveGroupCommander("arm_1")
        group.clear_pose_targets()
        group_variable_values = group.get_current_joint_values()
          
        group_variable_values[0] = 3.59015543623787 - 0.6
        group_variable_values[1] = 0.9315389239350645
        group_variable_values[2] = -3.529899819624825
        group_variable_values[3] = 0.03523058948877694
        group_variable_values[4] = 2.954386290852248
        
        group.set_joint_value_target(group_variable_values)
        plansethome = group.plan()
        group.execute(plansethome)
        
        group = moveit_commander.MoveGroupCommander("arm_1")
        group.clear_pose_targets()
        group_variable_values = group.get_current_joint_values()
          
        group_variable_values[0] = 3.59015543623787 - 0.6
        group_variable_values[1] = 0.9315389239350645
        group_variable_values[2] = -3.529899819624825
        group_variable_values[3] = 0.03523058948877694
        group_variable_values[4] = 2.954386290852248
        
        group.set_joint_value_target(group_variable_values)
        plansethome = group.plan()
        group.execute(plansethome)
        
        group = moveit_commander.MoveGroupCommander("arm_1")
        group.clear_pose_targets()
        group_variable_values = group.get_current_joint_values()
          
        group_variable_values[0] = 3.5901538077515154 - 0.6
        group_variable_values[1] = 0.9378673553605125
        group_variable_values[2] = -3.8614650340162404
        group_variable_values[3] = 0.36060318985185114
        group_variable_values[4] = 2.954386290852248
        
        group.set_joint_value_target(group_variable_values)
        plansethome = group.plan()
        group.execute(plansethome)
        closeGripper()
        
        group = moveit_commander.MoveGroupCommander("arm_1")
        group.clear_pose_targets()
        group_variable_values = group.get_current_joint_values()
          
        group_variable_values[0] = 3.59015543623787 - 0.6
        group_variable_values[1] = 0.9315389239350645
        group_variable_values[2] = -3.529899819624825
        group_variable_values[3] = 0.03523058948877694
        group_variable_values[4] = 2.954386290852248
        
        group.set_joint_value_target(group_variable_values)
        plansethome = group.plan()
        group.execute(plansethome)
        
    rp.set_param('object_count', object_count - 1)
    rp.set_param('object_picked', True)
    setPrepare()
    
def setPlacepExternal():
    object_count = rp.get_param('object_count')
    rp.set_param('arm_state', "arm_arm_moving")
    
    pos_x = 0.168428428263
    pos_y = 0.281016118584
    pos_z = 0.209628977196
    
    rot_x = -0.694006828516
    rot_y = 0.703508407321
    rot_z = 0.108422425973
    rot_w = 0.108051008086  
    executeTrajectory(pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w)
    
    group = moveit_commander.MoveGroupCommander("arm_1")
    pose = group.get_current_pose().pose
    pos_x = pose.position.x
    pos_y = pose.position.y
    pos_z = pose.position.z - 0.05
    
    rot_x = pose.orientation.x
    rot_y = pose.orientation.y
    rot_z = pose.orientation.z
    rot_w = pose.orientation.w
    executeTrajectory(pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w)
   
    rp.set_param('object_count', object_count - 1)
    rp.set_param('arm_state', "arm_arm_stby_place_external")
    
def setPlacepInternal():
    object_count = rp.get_param('object_count')
    rp.set_param('arm_state', "arm_arm_moving")
    
    if object_count == 0:
        group = moveit_commander.MoveGroupCommander("arm_1")
        group.clear_pose_targets()
        group_variable_values = group.get_current_joint_values()
          
        group_variable_values[0] = 3.59015543623787
        group_variable_values[1] = 0.9315389239350645
        group_variable_values[2] = -3.529899819624825
        group_variable_values[3] = 0.03523058948877694
        group_variable_values[4] = 2.954386290852248
        
        group.set_joint_value_target(group_variable_values)
        plansethome = group.plan()
        group.execute(plansethome)
        
        group = moveit_commander.MoveGroupCommander("arm_1")
        group.clear_pose_targets()
        group_variable_values = group.get_current_joint_values()
          
        group_variable_values[0] = 3.59015543623787
        group_variable_values[1] = 0.9315389239350645
        group_variable_values[2] = -3.529899819624825
        group_variable_values[3] = 0.03523058948877694
        group_variable_values[4] = 2.954386290852248
        
        group.set_joint_value_target(group_variable_values)
        plansethome = group.plan()
        group.execute(plansethome)
        
        group = moveit_commander.MoveGroupCommander("arm_1")
        group.clear_pose_targets()
        group_variable_values = group.get_current_joint_values()
          
        group_variable_values[0] = 3.5901538077515154
        group_variable_values[1] = 0.9378673553605125
        group_variable_values[2] = -3.8614650340162404
        group_variable_values[3] = 0.36060318985185114
        group_variable_values[4] = 2.954386290852248
        
        group.set_joint_value_target(group_variable_values)
        plansethome = group.plan()
        group.execute(plansethome)
        openGripper()
        
        group = moveit_commander.MoveGroupCommander("arm_1")
        group.clear_pose_targets()
        group_variable_values = group.get_current_joint_values()
          
        group_variable_values[0] = 3.59015543623787
        group_variable_values[1] = 0.9315389239350645
        group_variable_values[2] = -3.529899819624825
        group_variable_values[3] = 0.03523058948877694
        group_variable_values[4] = 2.954386290852248
        
    elif object_count == 1:
        group = moveit_commander.MoveGroupCommander("arm_1")
        group.clear_pose_targets()
        group_variable_values = group.get_current_joint_values()
          
        group_variable_values[0] = 3.59015543623787 - 0.3
        group_variable_values[1] = 0.9315389239350645
        group_variable_values[2] = -3.529899819624825
        group_variable_values[3] = 0.03523058948877694
        group_variable_values[4] = 2.954386290852248
        
        group.set_joint_value_target(group_variable_values)
        plansethome = group.plan()
        group.execute(plansethome)
        
        group = moveit_commander.MoveGroupCommander("arm_1")
        group.clear_pose_targets()
        group_variable_values = group.get_current_joint_values()
          
        group_variable_values[0] = 3.59015543623787 - 0.3
        group_variable_values[1] = 0.9315389239350645
        group_variable_values[2] = -3.529899819624825
        group_variable_values[3] = 0.03523058948877694
        group_variable_values[4] = 2.954386290852248
        
        group.set_joint_value_target(group_variable_values)
        plansethome = group.plan()
        group.execute(plansethome)
        
        group = moveit_commander.MoveGroupCommander("arm_1")
        group.clear_pose_targets()
        group_variable_values = group.get_current_joint_values()
          
        group_variable_values[0] = 3.5901538077515154 - 0.3
        group_variable_values[1] = 0.9378673553605125
        group_variable_values[2] = -3.8614650340162404
        group_variable_values[3] = 0.36060318985185114
        group_variable_values[4] = 2.954386290852248
        
        group.set_joint_value_target(group_variable_values)
        plansethome = group.plan()
        group.execute(plansethome)
        openGripper()
        
        group = moveit_commander.MoveGroupCommander("arm_1")
        group.clear_pose_targets()
        group_variable_values = group.get_current_joint_values()
          
        group_variable_values[0] = 3.59015543623787 - 0.3
        group_variable_values[1] = 0.9315389239350645
        group_variable_values[2] = -3.529899819624825
        group_variable_values[3] = 0.03523058948877694
        group_variable_values[4] = 2.954386290852248
        
        group.set_joint_value_target(group_variable_values)
        plansethome = group.plan()
        group.execute(plansethome)
        
    elif object_count == 2:   
        group = moveit_commander.MoveGroupCommander("arm_1")
        group.clear_pose_targets()
        group_variable_values = group.get_current_joint_values()
          
        group_variable_values[0] = 3.59015543623787 - 0.6
        group_variable_values[1] = 0.9315389239350645
        group_variable_values[2] = -3.529899819624825
        group_variable_values[3] = 0.03523058948877694
        group_variable_values[4] = 2.954386290852248
        
        group.set_joint_value_target(group_variable_values)
        plansethome = group.plan()
        group.execute(plansethome)
        
        group = moveit_commander.MoveGroupCommander("arm_1")
        group.clear_pose_targets()
        group_variable_values = group.get_current_joint_values()
          
        group_variable_values[0] = 3.59015543623787 - 0.6
        group_variable_values[1] = 0.9315389239350645
        group_variable_values[2] = -3.529899819624825
        group_variable_values[3] = 0.03523058948877694
        group_variable_values[4] = 2.954386290852248
        
        group.set_joint_value_target(group_variable_values)
        plansethome = group.plan()
        group.execute(plansethome)
        
        group = moveit_commander.MoveGroupCommander("arm_1")
        group.clear_pose_targets()
        group_variable_values = group.get_current_joint_values()
          
        group_variable_values[0] = 3.5901538077515154 - 0.6
        group_variable_values[1] = 0.9378673553605125
        group_variable_values[2] = -3.8614650340162404
        group_variable_values[3] = 0.36060318985185114
        group_variable_values[4] = 2.954386290852248
        
        group.set_joint_value_target(group_variable_values)
        plansethome = group.plan()
        group.execute(plansethome)
        openGripper()
        
        group = moveit_commander.MoveGroupCommander("arm_1")
        group.clear_pose_targets()
        group_variable_values = group.get_current_joint_values()
          
        group_variable_values[0] = 3.59015543623787 - 0.6
        group_variable_values[1] = 0.9315389239350645
        group_variable_values[2] = -3.529899819624825
        group_variable_values[3] = 0.03523058948877694
        group_variable_values[4] = 2.954386290852248
        
        group.set_joint_value_target(group_variable_values)
        plansethome = group.plan()
        group.execute(plansethome)
        
    rp.set_param('object_count', object_count + 1)
    rp.set_param('object_picked', False)
    setPrepare()
    
def openGripper():
    rp.set_param('gripper_state', "gripper_opening")
    
    rp.set_param('gripper_state', "gripper_open")
    
def closeGripper():
    rp.set_param('gripper_state', "gripper_closing")
    
    rp.set_param('gripper_state', "gripper_closed")
    
def executeTrajectory(pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w):
    rp.loginfo("ARM: Executing trajectory")
    tolerance = 0
    tries = 0
    group = moveit_commander.MoveGroupCommander("arm_1")
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = pos_x
    pose_target.position.y = pos_y
    pose_target.position.z = pos_z

    pose_target.orientation.x = rot_x
    pose_target.orientation.y = rot_y
    pose_target.orientation.z = rot_z
    pose_target.orientation.w = rot_w
    
    try: 
        group.set_joint_value_target(pose_target)
        group.allow_replanning(True)
        group.set_goal_position_tolerance(tolerance) 
        group.set_planning_time(10) 
        group.allow_looking(True)  
        plansetpos = group.plan()
        group.execute(plansetpos) 
        
    except:
        if tries < 10:
            tolerance += 0.001
            tries += 1
            executeTrajectory(pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w)
        else:
            rp.set_param('arm_state', "arm_arm_failed")
            tolerance = 0
            tries = 0          
    rp.sleep(2) 


    

if __name__ == '__main__': 
    try: 
        init()
        rp.spin()
    except rp.ROSInterruptException: 
        rp.loginfo("Ctrl-C caught. Quitting")
        sys.exit()