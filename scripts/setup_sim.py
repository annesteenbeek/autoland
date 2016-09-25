#!/usr/bin/env python

import numpy
import rospy
import mavros
import math
from mavros.utils import *
from mavros import command, mission, setpoint
from geometry_msgs.msg import PoseStamped, Point, Pose
# requires https://github.com/AndreaIncertiDelmonte/mavros repo with compile fixes branch for RPi
from mavros.msg import State, Waypoint 
from mavros.srv import CommandBool, SetMode, WaypointPush
from std_msgs.msg import Float64
from autoland.srv import *

# set variables
target = PoseStamped() # setpoint target to meet
COMM_RATE = 100.0 # ros rate [hz]

cur_local_pose = PoseStamped()
current_mode = ""
prev_mode = ""
external_postion = Point()

TARGET_REACHED = False

MOVE_X = 5
MOVE_Y = 4
MOVE_Z = 10

ACC_RAD = 0.5

def pose_cb(pose):
    global cur_local_pose
    cur_local_pose = pose

def state_cb(state):
    global current_mode, prev_mode
    current_mode = state.mode
    if current_mode != prev_mode:
        rospy.loginfo("changed mode to: %s" % current_mode)
        prev_mode = current_mode

def arm(state):
    try:
        ret_arm = command.arming(state)
    except rospy.ServiceException as ex:
        fault(ex)

def set_offboard(state):
    try:
        ret_mode = command.guided_enable(value=state)
    except rospy.ServiceException as ex:
        fault(ex)

def set_topics():
    global local_pos_pub, velocity_cmd_pub, local_pos_sub, state_sub
    local_pos_pub = setpoint.get_pub_position_local(queue_size=10)
    local_pos_sub = rospy.Subscriber(mavros.get_topic('local_position', 'local'), PoseStamped, pose_cb)
    state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)


def get_distance(target):
    dx = target.pose.position.x - cur_local_pose.pose.position.x
    dy = target.pose.position.y - cur_local_pose.pose.position.y
    dz = target.pose.position.z - cur_local_pose.pose.position.z

    dv = numpy.array((dx, dy, dz))
    dist = numpy.linalg.norm(dv) 
    return dist 

def shutdown_hook():
    sys.exit("received rospy shutdown, exiting...")

# setmode and set offboard
def setup_copter():
    rospy.init_node('setup_sim')
    rospy.on_shutdown(shutdown_hook)
    rospy.loginfo("waiting for fcu connection")
    wait_fcu_connection()
    rospy.loginfo("connected to fcu, setitng up topics")
    pose = PoseStamped()
    set_topics()
    pose.pose.position.x = MOVE_X
    pose.pose.position.y = MOVE_Y
    pose.pose.position.z = MOVE_Z

    rate = rospy.Rate(COMM_RATE) # MUST be more then 2Hz
    rospy.loginfo("waiting for autoland service")
    rospy.wait_for_service('start_autoland')
    rospy.loginfo("service started")
    start_auto_land = rospy.ServiceProxy('start_autoland', StartAutoland)
    rospy.loginfo("start sending setpoint")

    rospy.loginfo("setting mode to offboard") 
    while not current_mode == "OFFBOARD":
        for i in range(100):
            pose.header.stamp = rospy.get_rostime()
            local_pos_pub.publish(pose)
            rate.sleep()
        set_offboard(True)    

    # Arm the copter
    rospy.loginfo("arming")
    arm(True)
    TARGET_REACHED = False
    while not TARGET_REACHED:
        pose.header.stamp = rospy.get_rostime()
        local_pos_pub.publish(pose)
        
        if get_distance(pose) < ACC_RAD:
            TARGET_REACHED = True
            rospy.loginfo("target reached")
            print(cur_local_pose.pose.position)
            start_auto_land()
        rate.sleep()

if __name__ == '__main__':
    try:
        setup_copter()
    except rospy.ROSInterruptException:
        pass
