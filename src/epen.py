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
from autoland.srv import *

COMM_RATE = 100 # ros rate [hz]
rate = None
piksi_local_x = None
piksi_local_y = None
piksi_cov = None
px4_local_pose = None
px4_active = False
piksi_active = False
start_heading = None # heading in [rad] at arming
heading_ang = None

def pose_cb(pose):
    global px4_local_pose
    px4_local_pose = pose

def piksi_cb(data):
    global piksi_local_x, piksi_local_y, piksi_cov
    piksi_local_x = data.pose.pose.position.x
    piksi_local_y = data.pose.pose.position.y
    piksi_cov = data.pose.covariance[0]

def comp_cb(data):
    global heading_ang
    heading_ang = data

def set_launch_heading(input):
    global start_heading
    start_heading = numpy.deg2rad(heading_ang)
    rospy.loginfo("Start heading is set to %.2f deg", heading_ang)
    return start_heading

def setup_topics():
    global pub, pos_sub
    mavros.set_namespace()
    pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_cb)
    rospy.Subscriber('gps/rtkfix', Odometry, piksi_cb, queue_size=3)
    pub = rospy.Publisher('external_pose_estimation', PoseStamped, queue_size=10)
    comp_sub = rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, comp_cb)

def external_pose_estimator():
    global rate
    rospy.init_node('external_pose_estimator')
    rate = rospy.Rate(COMM_RATE)
    setup_topics()
    external_pose = PoseStamped()
    external_pose.header.frame_id = 'launch_start'
    
    while piksi_local_x is None and px4_local_pose is None and heading_ang is None: 
        rate.sleep()

    rospy.loginfo("start sending external position information")

    rospy.Service('set_launch_heading', SetLaunchHeading, set_launch_heading) 

    rospy.loginfo("waiting for start heading")

    while start_heading is None and not rospy.is_shutdown():
        rospy.sleep()

    rot_matrix = numpy.array([[numpy.cos(start_heading), numpy.sin(start_heading)], 
                              [-numpy.sin(start_heading),  numpy.cos(start_heading)]])

    while not rospy.is_shutdown():
        external_pose.header.stamp = rospy.get_rostime()
        external_pose.pose.orientation = px4_local_pose.pose.orientation
        piksi_enu = numpy.array([piksi_local_x, piksi_local_y]) 
        piksi_fcu_rot = numpy.dot(piksi_enu, rot_matrix)
        
        external_pose.pose.position.x = piksi_fcu_rot[0]
        external_pose.pose.position.y = piksi_fcu_rot[1]
        external_pose.pose.position.z = px4_local_pose.pose.position.z
        pub.publish(external_pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        external_pose_estimator()
    except rospy.ROSInterruptException:
        pass
