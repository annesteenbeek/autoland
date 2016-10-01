#!/usr/bin/env python

import numpy
import rospy
import mavros
import math
import sys
from mavros.utils import *
from mavros import command, mission, setpoint
from geometry_msgs.msg import PoseStamped, Point, Pose
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry 

COMM_RATE = 100 # ros rate [hz]
rate = None
piksi_local_x = None
piksi_local_y = None
piksi_cov = None
px4_local_pose = PoseStamped()
px4_active = False
piksi_active = False

def pose_cb(pose):
    global px4_local_pose, px4_active
    px4_local_pose = pose
    px4_active = True

def piksi_cb(data):
    global piksi_local_x, piksi_local_y, piksi_cov, piksi_active
    piksi_local_x = data.pose.pose.position.x
    piksi_local_y = data.pose.pose.position.y
    piksi_cov = data.pose.covariance[0]
    piksi_active = True

def setup_topics():
    global pub
    mavros.set_namespace()
    rospy.Subscriber(mavros.get_topic('local_position', 'local'), PoseStamped, pose_cb)
    rospy.Subscriber('gps/rtkfix', Odometry, piksi_cb, queue_size=3)
    pub = rospy.Publisher('external_pose_estimation', PoseStamped, queue_size=10)

def external_pose_estimator():
    global rate
    rospy.init_node('external_pose_estimator')
    rate = rospy.Rate(COMM_RATE)
    setup_topics()
    external_pose = PoseStamped()
    external_pose.header.frame_id = 'fcu'
    while not px4_active and not piksi_active: 
        rate.sleep()

    rospy.loginfo("start sending external position information")
    while not rospy.is_shutdown():
        external_pose.header.stamp = rospy.get_rostime()
        external_pose.pose.orientation = px4_local_pose.pose.orientation
        external_pose.pose.position.x = piksi_local_x
        external_pose.pose.position.y = piksi_local_y
        external_pose.pose.position.z = px4_local_pose.pose.position.z
        pub.publish(external_pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        external_pose_estimator()
    except rospy.ROSInterruptException:
        pass
