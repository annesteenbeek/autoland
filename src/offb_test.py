#!/usr/bin/env python

import rospy
import mavros
from mavros.utils import *
from mavros import command, mission, setpoint
from geometry_msgs.msg import PoseStamped, Point, Pose
from mavros.msg import State
from mavros.srv import CommandBool, SetMode

prev_mode = ""

def pose_cb(pose):
    global cur_local_pose
    cur_local_pose = pose

def state_cb(state):
    global current_mode, prev_mode
    current_mode = state.mode
    if current_mode == prev_mode:
        rospy.loginfo("changed mode to: %s" % current_mode)
        prev_mode = current_mode

def set_topics():
    global local_pos_pub, velocity_cmd_pub, local_pos_sub, state_sub
    local_pos_pub = setpoint.get_pub_position_local(queue_size=10, latch=True)
    local_pos_sub = rospy.Subscriber(mavros.get_topic('local_position', 'local'), PoseStamped, pose_cb)
    state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)


def run_test():
    rospy.init_node('flight_test')
    rate = rospy.Rate(100) # rate in HZ
    rospy.loginfo('setting up topics')
    set_topics()
    rospy.loginfo('topics set up')
    rospy.loginfo('waiting for fcu connection')
    wait_fcu_connection()
    rospy.loginfo('connected to fcu')
    command.setup_services()

    # command.guided_enable(True)
    target  = PoseStamped()

    target.pose.position.x = 0
    target.pose.position.y = 0
    target.pose.position.z = 3  

    # send a few setpoints before starting
    for i in range(100):
        local_pos_pub.publish(target)
        rate.sleep()

    # print current position
    rospy.loginfo("cur x position: %.2f", cur_local_pose.pose.position.x)  
    rospy.loginfo("cur y position: %.2f", cur_local_pose.pose.position.y)
    rospy.loginfo("cur z position: %.2f", cur_local_pose.pose.position.z)
    
    rospy.loginfo('waiting for offboard mode')

    while current_mode != "OFFBOARD" and not rospy.is_shutdown():
        local_pos_pub.publish(target)
        rate.sleep()

    rospy.loginfo('in offboard mode')

    rospy.loginfo('sending pose')
    while not rospy.is_shutdown():
        target.header.stamp = rospy.get_rostime()
        local_pos_pub.publish(target)
        rate.sleep()
    
if __name__ == '__main__':
        try:
            run_test()
        except rospy.ROSInterruptException:
            pass
