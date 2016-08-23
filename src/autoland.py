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
from mavros.srv import CommandBool, SetMode, WaypointPush
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry 

# set variables
target = PoseStamped() # setpoint target to meet
DESCEND_SPEED = 0.4 # descend speed [m/s]
COMM_RATE = 100.0 # ros rate [hz]
ACC_RAD = 20.0 # acceptence radius for setpoint [cm]
allowed_deviation = 0.1 # radius to exceed to stop descend [m]
LAND_ALT = 10 # altitude for landing [m]
LAND_X = 0 # relative to piksi 0
LAND_Y = 0
LOITER_TIME = 5. # time to loiter bevore descending [sec]
DISARM_ALT = 0.1 # altitude to turn off engines [m]
landState = 'SET_ALT' # counter for landing progress
prevState = landState
landed = False
effort_x = 0
effort_y = 0
rate = 0
cur_local_pose = PoseStamped()
current_mode = ""
prev_mode = ""
external_postion = Point()

def pose_cb(pose):
    global cur_local_pose
    cur_local_pose = pose

def state_cb(state):
    global current_mode, prev_mode
    current_mode = state.mode
    if current_mode == prev_mode:
        rospy.loginfo("changed mode to: %s" % current_mode)
        prev_mode = current_mode

def external_cb(point):
    global external_position
    print(point)
    external_position = point

def pid_cb(effort, direction):
    global effort_x, effort_y
    if direction is "x":
        effort_x = effort.data
    elif direction is "y":
        effort_y = effort.data
    else:
        rospy.logerr("Incorrect pid cb direction")

def piksi_cb(data):
    global piksi_local_x, piksi_local_y, piksi_cov
    piksi_local_x = data.pose.pose.position.x
    piksi_local_y = data.pose.pose.position.y
    piksi_cov = data.pose.covariance[0]

def set_topics():
    global local_pos_pub, velocity_cmd_pub, local_pos_sub, state_sub
    # mavros.set_namespace()
    local_pos_pub = setpoint.get_pub_position_local(queue_size=10, latch=True)
    velocity_cmd_pub = setpoint.get_pub_velocity_cmd_vel(queue_size=10, latch=True)
    local_pos_sub = rospy.Subscriber(mavros.get_topic('local_position', 'local'), PoseStamped, pose_cb)
    state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)

def arm(state):
    try:
        ret_arm = command.arming(state)
    except rospy.ServiceException as ex:
        fault(ex)

def set_offboard(state):
    try:
        command.guided_enable(value=state)
    except rospy.ServiceException as ex:
        fault(ex)

# calculate distance between two points in 3d, returns [cm]
def get_distance(target):
    # NOTE this could probably be done more elegent with TF
    dx = target.pose.position.x - cur_local_pose.pose.position.x
    dy = target.pose.position.y - cur_local_pose.pose.position.y
    dz = target.pose.position.z - cur_local_pose.pose.position.z

    dv = numpy.array((dx, dy, dz))
    dist = numpy.linalg.norm(dv) * 100 # return in cm
    return dist 

# set correct altitude for landing
def set_land_alt():
    global target
    pose = PoseStamped()
    pose.pose.position.x = cur_local_pose.pose.position.x
    pose.pose.position.y = cur_local_pose.pose.position.y
    pose.pose.position.z = LAND_ALT # land altitude relative to launch altitude
    target = pose
    pose.header.stamp = rospy.get_rostime()
    local_pos_pub.publish(pose)

# position above landing position
def move_to_land():
    global target
    pose = PoseStamped()
    pose.pose.position.x = LAND_X
    pose.pose.position.y = LAND_Y
    pose.pose.position.z = LAND_ALT

    target = pose
    pose.header.stamp = rospy.get_rostime()
    local_pos_pub.publish(pose)

def stabalize_above_land():
    doneLoitering = False
    start = rospy.get_rostime()
    # loiter for x sec to reach stable position
    rospy.loginfo("AUTOLAND loitering for %.1fsec" %LOITER_TIME)
    while not doneLoitering:
        now = rospy.get_rostime()
        target.header.stamp = now
        local_pos_pub.publish(target)
        if now - start >= rospy.Duration(LOITER_TIME):
            doneLoitering = True

# engage in controlled descent
def autoland():
    global landed
    rospy.loginfo("REACHED AUTOLAND")
    stabalize_above_land()
    rospy.loginfo("AUTOLAND loitering done, starting descent")
    velocity_cmd_pub = setpoint.get_pub_velocity_cmd_vel(queue_size=10)
    rospy.Subscriber('effort_x', Float64, pid_cb, "x", 1)
    rospy.Subscriber('effort_y', Float64, pid_cb, "y", 1) 
    
    # create setpoint publishers
    set_x_pub = rospy.Publisher("autoland_setpoint_x", Float64, latch=True, queue_size=1)
    set_y_pub = rospy.Publisher("autoland_setpoint_y", Float64, latch=True, queue_size=1)
    
    # create piksi subscribers
    rospy.Subscriber('gps/rtkfix', Odometry, piksi_cb, queue_size=3)
    
    # create plant state publishers
    pose_x_pub = rospy.Publisher("autoland_pose_x", Float64, queue_size=3)
    pose_y_pub = rospy.Publisher("autoland_pose_y", Float64, queue_size=3)

    while not landed:
        # TODO apply moving avg filter
        # TODO launch file
        # TODO simple tests
        pos_x = piksi_local_x
        pos_y = piksi_local_y
        pos_z = cur_local_pose.pose.position.z # TODO get from barometer/laser

        # publish x and y position
        pose_x_pub.publish(pos_x)
        pose_y_pub.publish(pos_y)

        # send landing setpoints
        set_x_pub.publish(LAND_X)
        set_y_pub.publish(LAND_Y)

        speed_x = effort_x
        speed_y = effort_y

        dist_to_land = math.sqrt(
                 math.pow(LAND_X - pos_x, 2) + math.pow(LAND_Y - pos_y,2)
                )
        if dist_to_land > allowed_deviation or piksi_cov == 1000:
            speed_z = 0
        else:
            speed_z = - DESCEND_SPEED
        
        vel = setpoint.TwistStamped(header=setpoint.Header(frame_id='mavsetp', stamp=rospy.get_rostime()))
        vel.twist.linear = setpoint.Vector3(x=speed_x, y=speed_y, z=speed_z)
        vel.twist.angular = setpoint.Vector3(z=0)
        velocity_cmd_pub.publish(vel)
        rate.sleep()

        # TODO make sure roll, pitch are within acceptable bounds
        if pos_z <= DISARM_ALT:
            rospy.loginfo("Quadcopter landed!")
            rospy.loginfo("x pos: %.3fm" % pos_x)
            rospy.loginfo("y pos: %.3fm" % pos_y)
            rospy.loginfo("Total distance: %.3fm" % dist_to_land)
            landed = True
            arm(False)
            rospy.loginfo("FINISHED LAND")
            landed = True
    
def advance_state():
    global landState, prevState
    if landState is 'SET_ALT':
        landState = 'MOVE_LAND' 
        
    elif landState is 'MOVE_LAND':
        landState = 'AUTOLAND' 
            
    elif landState is 'AUTOLAND':
        landState = 'FINISH_LAND' 
            
    elif landState is 'FINISH_LAND':
        rospy.loginfo("done with landing") 

    if prevState is not landState:
        rospy.loginfo('landing state changed to: %s' % landState)
    prevState = landState            

def handle_state():
    switcher = {
            'SET_ALT': set_land_alt,
            'MOVE_LAND': move_to_land,
            'AUTOLAND': autoland,
            'FINISH_LAND': finish_land,
            }
    switcher[landState]() # execute correct state function
            
def landing_automator():
    rospy.init_node('autoland_node', anonymous=True)
    rate = rospy.Rate(COMM_RATE) # MUST be more then 2Hz
    rospy.loginfo("setting up topics")
    set_topics()
    rospy.loginfo("topics set up")
    rospy.loginfo("waiting for fcu connection")
    wait_fcu_connection()
    rospy.loginfo("connected to fcu")
    command.setup_services()

    set_offboard(True)    
    # make sure in offboard mode, could retry but better to exit and retry manually
    rospy.sleep(1)
    if current_mode != "OFFBOARD":
        rospy.logerr("Not in offboard mode, aborting.")
        sys.exit("unable to enter offboard mode, exiting")

    while not landed and not rospy.is_shutdown():
        if get_distance(target) <= ACC_RAD:
            advance_state()
            handle_state()
        target.header.stamp = rospy.get_rostime()    
        local_pos_pub.publish(target)
        rate.sleep()

if __name__ == '__main__':
    try:
        landing_automator()
    except rospy.ROSInterruptException:
        pass
