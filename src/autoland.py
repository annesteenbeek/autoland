#!/usr/bin/env python

import rospy
import mavros
from mavros.utils import *
from mavros import command, mission, setpoint
from geometry_msgs.msg import PoseStamped
from mavros.msg import State, Waypoint 
from mavros.srv import CommandBool, SetMode, WaypointPush

local_pos_pub = setpoint.get_pub_position_local(queue_size=10, latch=True)

pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 2

waypoint = mission.Waypoint(
    frame = 3,
    command = 22,
    param1 = 0.0,
    param2 = 0.0,
    param3 = 0.0,
    param4 = 0.0,
    x_lat = 47.397804260253906,
    y_long = 8.54560661315918,
    z_alt = 4 
)

def arm(state):
    try:
        ret_arm = command.arming(state)
    except rospy.ServiceException as ex:
        fault(ex)
        
    if ret_arm.success:
        rospy.loginfo("Copter armed state: %r" % state)
    else:
        rospy.loginfo("Unable to arm/disarm copter")


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
    ret_mode = command.guided_enable(value=True)
    if ret_mode.success:
        rospy.loginfo("copter mode is OFFBOARD")
    else: 
        rospy.loginfo("unable to set mode to OFFBOARD")
        
    # Arm the copter
    arm(True)
    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        
        # Update timestamp and publish pose 
        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        position_control()
    except rospy.ROSInterruptException:
        pass
