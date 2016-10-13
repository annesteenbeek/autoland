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
px4_local_pose = None
mov_avg_que = 10
piksi_x_mean_list = []
piksi_y_mean_list = []


def pose_cb(pose):
    global px4_local_pose
    px4_local_pose = pose

def piksi_cb(data):
    global piksi_local_x, piksi_local_y, piksi_cov
    piksi_local_x = data.pose.pose.position.x
    piksi_local_y = data.pose.pose.position.y
    piksi_cov = data.pose.covariance[0]
     

def setup_topics():
    global pub, pos_sub
    mavros.set_namespace()
    pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_cb)
    rospy.Subscriber('gps/rtkfix', Odometry, piksi_cb, queue_size=3)
    pub = rospy.Publisher('external_pose_estimation', PoseStamped, queue_size=10)

def external_pose_estimator():
    global rate
    rospy.init_node('external_pose_estimator')
    rate = rospy.Rate(COMM_RATE)
    setup_topics()
    external_pose = PoseStamped()
    external_pose.header.frame_id = 'launch_start'
    
    while piksi_local_x is None or px4_local_pose is None:
        rate.sleep()

    while not rospy.is_shutdown():
        piksi_x_mean_list = [piksi_local_x] + piksi_x_mean_list
        piksi_y_mean_list = [piksi_local_y] + piksi_y_mean_list
        
        if len(piksi_x_mean_list) > mov_avg_que:
            piksi_x_mean_list.pop()
            piksi_y_mean_list.pop()

        piksi_avg_x = sum(piksi_x_mean_list)/len(piksi_x_mean_list)
        piksi_avg_y = sum(piksi_y_mean_list)/len(piksi_y_mean_list)

        external_pose.header.stamp = rospy.get_rostime()
        external_pose.pose.orientation = px4_local_pose.pose.orientation
        piksi_enu = numpy.array([piksi_local_x, piksi_local_y]) 
        
        external_pose.pose.position.x = piksi_avg_x
        external_pose.pose.position.y = piksi_avg_y 
        external_pose.pose.position.z = px4_local_pose.pose.position.z
        pub.publish(external_pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        external_pose_estimator()
    except rospy.ROSInterruptException:
        pass
