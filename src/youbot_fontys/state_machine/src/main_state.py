#!/usr/bin/env python 
import rospy as rp
import sys
import time

from geometry_msgs.msg import PoseArray

def init():
    rp.init_node('main_state', anonymous=False) 
    rp.set_param('pose_n', 0)
    rp.set_param('object_n', 0)
    subToRef()
    
def subToRef():
    rp.loginfo("MAIN: Ready to receive data")
    pose_msg = rp.wait_for_message("/goal_queue_goals", PoseArray)
    print pose_msg.poses[0]
    
    try:
        object_msg = rp.wait_for_message("/object_array", PoseArray, timeout = 2)
    except:
        object_msg = 0

    rp.loginfo("MAIN: Done receiving")     
#    pose_msg = [0, 1] #test value
#    object_msg = [0, 1] #test value
    mainState(pose_msg, object_msg)

def sendGoal(pose_msg):
    if rp.get_param('base_state') == "base_stby" or rp.get_param('base_state') == "base_goal_stby_reached":
        pose_n = rp.get_param('pose_n')
#        object_n = rp.get_param('object_n')
        try:
            rp.set_param('goal_x', pose_msg.poses[pose_n].position.x)
            rp.set_param('goal_y', pose_msg.poses[pose_n].position.y)
            rp.set_param('goal_z', pose_msg.poses[pose_n].orientation.z)
            rp.set_param('goal_w', pose_msg.poses[pose_n].orientation.w)
            
            rp.loginfo("MAIN: Goal send to base")
            rp.set_param('pose_n', pose_n + 1)
            rp.set_param('base_state', "base_goal_send")
        except:
            rp.loginfo("MAIN: No more poses in PoseArray, heading to exit")
    #        rp.set_param('goal_x', 0)
    #        rp.set_param('goal_y', 0)
    #        rp.set_param('goal_z', 0)
    #        rp.set_param('goal_w', 1)
    
        

def mainState(pose_msg, object_msg):    
    if pose_msg != 0:
        nav_received = True
    elif pose_msg == 0:
        nav_received = False
        
    if object_msg != 0:
        object_received = True
    elif object_msg == 0:
        object_received = False

    if object_received == False and nav_received == True:
        test_state = "BNT"
        rp.loginfo("MAIN: Test received is Basic Navigation Test")          
    elif object_received == True and nav_received == False:
        test_state = "BMT"
        rp.loginfo("MAIN: Test received is Basic Manipulation Test")
    elif object_received == True and nav_received == True:
        test_state = "BTT"
        rp.loginfo("MAIN: Test received is Basic Transportation Test")
    
    while True:            
        if test_state == "BTT":
            rp.loginfo("MAIN: Starting the BTT test")
            sendGoal(pose_msg)
            rp.set_param('arm_state', "arm_arm_prepare")
            object_n = rp.get_param('object_n')
            tasked_object = object_msg[object_n]
            rp.set_param('vision_state', "vision_object_send")
            rp.set_param('tasked_object', tasked_object)
            pick_or_place = "pick" #test value
            if rp.get_param("base_state") == "base_stby_reached":
                if pick_or_place == "pick":
                    if rp.get_param('arm_state') == "arm_arm_stby_prepare":
                        rp.set_param('arm_state', "arm_arm_scan")
                    elif rp.get_param('vision_state') == "vision_tf_ready" and rp.get_param('arm_state') == "arm_arm_stby_scan":
                        rp.set_param('arm_state', "arm_arm_grasp_external")
                    elif rp.get_param('arm_state') == "arm_arm_stby_external":
                        rp.set_param('arm_state', "arm_arm_prepare")
                    elif rp.get_param('arm_state') == "arm_arm_stby_prepare":
                        rp.set_param('arm_state', "arm_arm_place_internal")
                        sendGoal(pose_msg)
                        
                elif pick_or_place == "place":
                    if rp.get_param('arm_state') == "arm_arm_stby_prepare":
                        rp.set_param('arm_state', "arm_arm_grasp_internal")
                    elif rp.get_param('arm_state') == "arm_arm_stby_internal":
                        rp.set_param('arm_state', "arm_arm_prepare")
                    elif rp.get_param('arm_state') == "arm_arm_stby_prepare":
                        rp.set_param('arm_state', "arm_arm_grasp_external")
                        rp.set_param('object_n' + 1)
                        sendGoal(pose_msg)
                
        elif test_state == "BMT":
            rp.loginfo("MAIN: Starting the BMT test")
            rp.set_param('arm_state', "arm_arm_scan")
            if rp.get_param('vision_state') == "vision_tf_ready" and rp.get_param('arm_state') == "arm_arm_stby_scan":
                rp.set_param('arm_state', "arm_arm_grasp_external")
                if rp.get_param('arm_state') == "arm_arm_stby_external":
                    rp.set_param('arm_state', "arm_arm_prepare")
                if rp.get_param('arm_state') == "arm_arm_stby_prepare":
                    rp.set_param('arm_state', "arm_arm_place_internal")
                    
        elif test_state == "BNT":
#            rp.loginfo("MAIN: Starting the BNT test")
#            rp.set_param('arm_state', "arm_arm_home")
            sendGoal(pose_msg)

if __name__ == '__main__': 
    try: 
        init()
        rp.spin()
    except rp.ROSInterruptException: 
        rp.loginfo("Ctrl-C caught. Quitting")
        sys.exit()