#!/usr/bin/env python

import rospy
import mavros
from mavros.utils import *
from mavros import command, mission, setpoint
from geometry_msgs.msg import PoseStamped
from mavros.msg import State, Waypoint 
from mavros.srv import CommandBool, SetMode, WaypointPush

local_pos_pub = setpoint.get_pub_position_local(queue_size=10, latch=True)

target = PoseStamped() # setpoint target to meet
DESCEND_SPEED = 0.0 # descend speed [m/s]
ACC_RAD = 20.0 # acceptence radius for setpoint [cm]
landState = 0 # counter for landing progress

def arm(state):
    try:
        ret_arm = command.arming(state)
    except rospy.ServiceException as ex:
        fault(ex)
        
    if ret_arm.success:
        rospy.loginfo("Copter armed state: %r" % state)
    else:
        rospy.loginfo("Unable to arm/disarm copter")

def set_offboard(state):
    try:
        ret_mode = command.guided_enable(value=state)
    except rospy.ServiceException as ex:
        fault(ex)

    if ret_mode.success:
        rospy.loginfo("Copter OFFBOARD: %r" % state)
    else: 
        rospy.loginfo("Unable to enable/disable OFFBOARD")

def get_distance(pose1, pose2):
    # calculate distance between two points
    return dist

def setup_copter():
    # set mode and set offboard

def move_away():
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2
   # move away    

def set_land_alt():
    # set correct altitude for landing

def move_to_land():
    # position above landing position

def autoland():
    # engage in controlled descent

def finish_land():
    # disarm and shutdown copter
    # show results

def advance_state():
    global landState
    if landState is 'SETUP':
        # done with setup    
    if landState is 'MOVE_AWAY':
        if dist(curPose, target) <= ACC_RAD:
            
    if landState is 'SET_ALT':
        if requirement:
            
    if landState is 'MOVE_LAND':
        if requirement:
            
    if landState is 'AUTOLAND':
        if requirement:
            
    if landState is 'FINISH_LAND':
        if requirement:
            



def handle_state():
    switcher = {
            'SETUP': setup_copter,
            'MOVE_AWAY': move_away,
            'SET_ALT': set_land_alt,
            'MOVE_LAND': move_to_land,
            'AUTOLAND': autoland,
            'FINISH_LAND': finish_land,
            }
    switcher[landState]() # execute correct state function
            
def position_control():
    rospy.init_node('autoland_node', anonymous=True)
    rate = rospy.Rate(20.0) # MUST be more then 2Hz
    # wait for FCU connection
    wait_fcu_connection()
    command.setup_services()

    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()

    # set mode to OFFBOARD 
    set_offboard(True)    
    # Arm the copter
    arm(True)
    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        
        # Update timestamp and publish pose 
        pose.header.stamp = now
        local_pos_pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        position_control()
    except rospy.ROSInterruptException:
        pass
