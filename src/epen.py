#!/usr/bin/env python

import numpy
import rospy
import mavros
import math
import sys
from mavros.utils import *
from mavros import command, mission, setpoint
from geometry_msgs.msg import PoseStamped, Point, Pose
# requires https://github.com/AndreaIncertiDelmonte/mavros repo with compile fixes branch for RPi
from mavros.msg import State, Waypoint 
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry 

COMM_RATE = 100 # ros rate [hz]
rate = None

def pose_cb(pose):
    global cur_local_pose
    cur_local_pose = pose

local_pos_sub = rospy.Subscriber(mavros.get_topic('local_position', 'local'), PoseStamped, pose_cb)
rospy.Subscriber('gps/rtkfix', Odometry, piksi_cb, queue_size=3)

def shutdown_hook():
    sys.exit("received rospy shutdown, exiting...")

def external_pose_estimator():
    global rate
    rospy.init_node('external_pose_estimator')
    rospy.on_shutdown(shutdown_hook)
    rate = rospy.Rate(COMM_RATE)


if __name__ == '__main__':
    try:
        external_pose_estimator()
    except rospy.ROSInterruptException:
        pass
