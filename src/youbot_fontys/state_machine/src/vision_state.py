#!/usr/bin/env python 
import rospy as rp
import sys
import tf
from object_recognition_msgs.msg import RecognizedObjectArray

def init():
    rp.init_node('vision_state', anonymous=False) 
    rp.set_param('vision_state', "vision_stby")
    visionState()

def visionState():
    while True:
        if rp.get_param('vision_state') == "vision_object_send":
            tasked_object = rp.get_param('tasked_object')
            try:
                tasked_object = tasked_object.header[0]
                rp.set_param('vision_state', "vision_object_received")
                break
            except:
                tasked_object_id = "empty"
        elif rp.get_param('vision_state') != "vision_object_send":
            tasked_object_id = "empty"
    
    n = 0
    object_x = 0
    object_y = 0
    object_z = 0
    object_xr = 0
    object_yr = 0
    object_zr = 0
    object_wr = 0
    
    while True:
        object_array = rp.wait_for_message("/recognized_object_array", RecognizedObjectArray)
        try:
            if object_array.objects[n].id == tasked_object_id:
                object_x += object_array.objects[n].pose.pose.pose.position.x
                object_y += object_array.objects[n].pose.pose.pose.position.y
                object_z += object_array.objects[n].pose.pose.pose.position.z
                object_xr += object_array.objects[n].pose.pose.pose.orientation.x
                object_yr += object_array.objects[n].pose.pose.pose.orientation.y
                object_zr += object_array.objects[n].pose.pose.pose.orientation.z
                object_wr += object_array.objects[n].pose.pose.pose.orientation.w
            if n > 18:
                n += 1
                object_x = object_x / n
                object_y = object_y / n
                object_z = object_z / n
                object_xr = object_xr / n
                object_yr = object_yr / n
                object_zr = object_zr / n
                object_wr = object_wr / n
                break
        except:
            n += 1
        
        tfHandler(object_x, object_y, object_z, object_xr, object_yr, object_zr, object_wr)

def tfHandler(object_x, object_y, object_z, object_xr, object_yr, object_zr, object_wr):
    rp.set_param('vision_state', "vision_object_detected")
    br1 = tf.TransformBroadcaster()
    ls1 = tf.TransformListener()
    while True:
        br1.sendTransform((object_x, object_y, object_z), 
                           (object_xr, object_yr, object_zr, object_wr), 
                            rp.Time.now(), 
                            "object", 
                            "cam_link")
                            
        object_to_world = ls1.lookupTransform('/object', '/map', rp.Time(0))
        tfToWorld(object_to_world)

def tfToWorld(object_to_world):
    br2 = tf.TransformBroadcaster()
    object_to_world_pos = object_to_world[0]
    object_to_world_rot = object_to_world[1]
    while True:
        try:            
            br2.sendTransform((object_to_world_pos[0], object_to_world_pos[1], object_to_world_pos[3]), 
                               (object_to_world_rot[0], object_to_world_rot[1], object_to_world_rot[2], object_to_world_rot[3]), 
                                rp.Time.now(), 
                                "object", 
                                "map")
            rp.set_param('vision_state', "vision_tf_ready")
            if rp.get_param('object_picked') == True:
                break
        except:
            rp.set_param('vision_state', "vision_tf_notready")
    

if __name__ == '__main__': 
    try: 
        init()
        rp.spin()
    except rp.ROSInterruptException: 
        rp.loginfo("Ctrl-C caught. Quitting")
        sys.exit()