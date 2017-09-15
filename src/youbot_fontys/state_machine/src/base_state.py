#!/usr/bin/env python 
import rospy as rp
import sys
import actionlib 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal 
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from actionlib_msgs.msg import * 

def init():
    rp.init_node('base_state', anonymous=False) 
    rp.set_param('base_state', "base_stby")

class GoToPose():
    def __init__(self): 
        self.goal_sent = False 
        rp.on_shutdown(self.shutdown) 
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction) 
        rp.loginfo("Wait for the action server to come up") 
        self.move_base.wait_for_server(rp.Duration(5)) 
        rp.loginfo("ready")
    
    def goto(self, x, y, rotz, rotw): 
        self.goal_sent = True 
        goal = MoveBaseGoal() 
        goal.target_pose.header.frame_id = 'map' 
        goal.target_pose.header.stamp = rp.Time.now() 
        goal.target_pose.pose = Pose(Point(x, y, 0), (Quaternion(0, 0, rotz, rotw)))
        rp.loginfo("%s", goal)
        # Start moving 
        rp.set_param('base_state', "base_goal_navigating")
        try:
            self.move_base.send_goal(goal) 
        except:
                rp.logerr("not a valid pose")
        rp.set_param('base_state', "navigating")
        # Gives Youbot 60 seconds to complete task 
        success = self.move_base.wait_for_result(rp.Duration(60)) 
    
        state = self.move_base.get_state() 
        result = False 
        
        if success and state == GoalStatus.SUCCEEDED: 
            result = True 
        else: 
            self.move_base.cancel_goal() 
    
        self.goal_sent = False 
        rp.set_param('base_state', "base_stby")
        return result 

    def shutdown(self): 
        self.move_base.cancel_goal() 
        rp.loginfo("Stop") 
        rp.sleep(1) 
    
def baseState():
    while True:
        base_state = rp.get_param('base_state')
        if base_state == "base_stby" or base_state == "base_goal_stby_reached":
#            rp.loginfo("BASE_STATE  -->  stanby")
            twistZero()
        elif base_state == "base_goal_send":
            rp.loginfo("BASE_STATE  -->  goal is send")
            goTo()

def twistZero():
    pub = rp.Publisher('cmd_vel', Twist, queue_size = 1)
    twist = Twist()
    twist.linear.x = 0; 
    twist.linear.y = 0; 
    twist.linear.z = 0;
    twist.angular.x = 0; 
    twist.angular.y = 0; 
    twist.angular.z = 0; 
    pub.publish(twist)

def goTo():
    x = rp.get_param('goal_x')
    y = rp.get_param('goal_y')
    rotz = rp.get_param('goal_z')
    rotw = rp.get_param('goal_w')
    success = navigator.goto(x, y, rotz, rotw)
    
    if success: 
        rp.set_param('base_state', "base_goal_stby_reached")  
        rp.loginfo("BASE: Goal reached")             
    else: 
        rp.set_param('base_state', "base_stby")
        rp.logerr("BASE: Base failed to reach goal")
        
    

if __name__ == '__main__': 
    try: 
        init()
        navigator = GoToPose()
        baseState()
        rp.spin()
    except rp.ROSInterruptException: 
        rp.loginfo("Ctrl-C caught. Quitting")
        sys.exit()