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
from autoland.srv import *

# set variables
target = PoseStamped() # setpoint target to meet
DESCEND_SPEED = 2 # descend speed [m/s]
COMM_RATE = 100.0 # ros rate [hz]
rate = None
ACC_RAD = 0.2 # acceptence radius for setpoint [m]
allowed_deviation = 0.1 # radius to exceed to stop descend [m]
LAND_ALT = 20 # altitude for landing [m]
LAND_X = None # relative to piksi 0
LAND_Y = None 
LAND_Z = None
LOITER_TIME = 10. # time to loiter bevore descending [sec]
DISARM_ALT = 0.1 # altitude to turn off engines [m]
landed = False
effort_x = 0
effort_y = 0
rate = 0
cur_local_pose = PoseStamped()
external_pose = None
current_mode = ""
prev_mode = ""
armed = None
start_land = False

def pose_cb(pose):
    global cur_local_pose
    cur_local_pose = pose

def state_cb(state):
    global current_mode, prev_mode, armed
    current_mode = state.mode
    armed = state.armed
    if current_mode == prev_mode:
        rospy.loginfo("changed mode to: %s" % current_mode)
        prev_mode = current_mode

def external_cb(pose):
    global external_pose
    external_pose = pose
    
def arm(state):
    try:
        ret_arm = command.arming(state)
    except rospy.ServiceException as ex:
        fault(ex)

def set_start_land(input):
    global start_land
    rospy.loginfo("Auto land service has been called")
    if LAND_X is None:
        rospy.logerr("Unable to land, land pos not set")
    else:
        if current_mode != "OFFBOARD":
            rospy.loginfo("Vehicle not in OFFBOARD mode yet,  waiting...")
            target = PoseStamped()
            target.header.frame_id = 'fcu'
            target.pose.position = cur_local_pose.position
            while not current_mode == "OFFBOARD":
                target.header.stamp = rospy.get_rostime()
                local_pos_pub.publish(target)
                rate.sleep()
        rospy.loginfo("Vehicle in OFFBOARD mode, continueing...")
        start_land = True
        rospy.loginfo("landing at x %.2f y %.2f" % (LAND_X, LAND_Y))
    return "success"

def pid_cb(effort, direction):
    global effort_x, effort_y
    if direction is "x":
        effort_x = effort.data
    elif direction is "y":
        effort_y = effort.data
    else:
        rospy.logerr("Incorrect pid cb direction")

def set_topics():
    global local_pos_pub, velocity_cmd_pub, state_sub
    # mavros.set_namespace()
    local_pos_pub = setpoint.get_pub_position_local(queue_size=10, latch=True)
    velocity_cmd_pub = setpoint.get_pub_velocity_cmd_vel(queue_size=10, latch=True)
    rospy.Subscriber(mavros.get_topic('local_position', 'local'), PoseStamped, pose_cb)
    rospy.Subscriber("external_pose_estimation", PoseStamped, external_cb)
    state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)
    rospy.Service('start_autoland', StartAutoland, set_start_land)

# calculate distance between two points in 3d
def get_distance(target):
    # NOTE this could probably be done more elegent with TF
    dx = target.pose.position.x - cur_local_pose.pose.position.x
    dy = target.pose.position.y - cur_local_pose.pose.position.y
    dz = target.pose.position.z - cur_local_pose.pose.position.z

    dv = numpy.array((dx, dy, dz))
    dist = numpy.linalg.norm(dv)
    return dist 

def set_land_pos():
    # TODO set land yaw
    global LAND_X, LAND_Y, LAND_Z#, LAND_YAW
    LAND_X = external_pose.pose.position.x
    LAND_Y = external_pose.pose.position.y
    LAND_Z = external_pose.pose.position.z
    rospy.loginfo("landing pos has been set")

# set correct altitude for landing
def set_land_alt():
    rospy.loginfo("Moving to landing altitude")
    target = PoseStamped()
    target.pose.position.x = cur_local_pose.pose.position.x
    target.pose.position.y = cur_local_pose.pose.position.y
    # don't change height when drone is currently higher then land alt
    if cur_local_pose.pose.position.z >= LAND_ALT:
        target.pose.position.z = cur_local_pose.pose.position.z
    else:
        target.pose.position.z = LAND_ALT # land altitude relative to launch altitude
    while not get_distance(target) <= ACC_RAD: 
        target.header.stamp = rospy.get_rostime()
        local_pos_pub.publish(target)
        rate.sleep()
    rospy.loginfo("Reached altitude with distance %.3f" % get_distance(target))

# position above landing position
def move_to_land():
    rospy.loginfo("Moving above land target")
    target = PoseStamped()
    target.pose.position.x = LAND_X
    target.pose.position.y = LAND_Y
    target.pose.position.z = cur_local_pose.pose.position.z # dont change alt

    while not get_distance(target) <= ACC_RAD: 
        target.header.stamp = rospy.get_rostime()
        local_pos_pub.publish(target)
        rate.sleep()
    rospy.loginfo("Reached above landing with distance %.3f" % get_distance(target))

def stabalize_above_land():
    doneLoitering = False
    start = rospy.get_rostime()
    # loiter for x sec to reach stable position
    rospy.loginfo("AUTOLAND loitering for %.1fsec" %LOITER_TIME)
    target = cur_local_pose
    while not doneLoitering:
        now = rospy.get_rostime()
        target.header.stamp = now
        local_pos_pub.publish(target)
        if now - start >= rospy.Duration(LOITER_TIME):
            doneLoitering = True
        rate.sleep()

# engage in controlled descent
def do_land():
    global landed
    # stabalize_above_land()
    # rospy.loginfo("AUTOLAND loitering done, starting descent")
    velocity_cmd_pub = setpoint.get_pub_velocity_cmd_vel(queue_size=10)
    rospy.Subscriber('effort_x', Float64, pid_cb, "x", 1)
    rospy.Subscriber('effort_y', Float64, pid_cb, "y", 1) 
    
    # create setpoint publishers
    set_x_pub = rospy.Publisher("autoland_setpoint_x", Float64, latch=True, queue_size=1)
    set_y_pub = rospy.Publisher("autoland_setpoint_y", Float64, latch=True, queue_size=1)
   
    # create plant state publishers
    pose_x_pub = rospy.Publisher("autoland_pose_x", Float64, queue_size=3)
    pose_y_pub = rospy.Publisher("autoland_pose_y", Float64, queue_size=3)

    while not landed:
        pos_x = external_pose.pose.position.x
        pos_y = external_pose.pose.position.y
        pos_z = external_pose.pose.position.z 

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
        if dist_to_land > allowed_deviation:# or piksi_cov == 1000:
            speed_z = 0
        else:
            speed_z = - DESCEND_SPEED
        
        vel = setpoint.TwistStamped(header=setpoint.Header(frame_id='mavsetp', stamp=rospy.get_rostime()))
        vel.twist.linear = setpoint.Vector3(x=speed_x, y=speed_y, z=speed_z)
        vel.twist.angular = setpoint.Vector3(z=0)
        velocity_cmd_pub.publish(vel)
        rate.sleep()

        # TODO make sure roll, pitch are within acceptable bounds
        if (pos_z - LAND_Z) <= DISARM_ALT:
            rospy.loginfo("Quadcopter landed!")
            rospy.loginfo("x pos: %.3fm" % pos_x)
            rospy.loginfo("y pos: %.3fm" % pos_y)
            rospy.loginfo("Total distance: %.3fm" % dist_to_land)
            # TODO fix end of landing
            arm(False)
            rospy.loginfo("FINISHED LAND")
            landed = True
    
def shutdown_hook():
        sys.exit("Received rospy shutdown, exiting...")
        
def landing_automator():
    global rate
    rospy.init_node('autoland')
    rospy.on_shutdown(shutdown_hook)
    rate = rospy.Rate(COMM_RATE) # MUST be more then 2Hz
    rospy.loginfo("setting up topics")
    set_topics()
    rospy.loginfo("topics set up")
    rospy.loginfo("waiting for fcu connection")
    wait_fcu_connection()
    rospy.loginfo("connected to fcu")
    command.setup_services()
    
    if not armed:
       # wait unit vehicle is armed and store that location
       got_land = False
       rospy.loginfo("not armed, waiting")
       while not got_land: 
            if armed:
                # wait until external pose has been sent for accurate starting pos
                if external_pose is not None: 
                    rospy.loginfo("Armed and receiving, setting landing pos")
                    set_land_pos()
                    got_land = True
            rate.sleep()
    elif armed: 
        rospy.logerr("Script needs to be ran before launch")
        sys.exit("Exiting...")
    else:
        rospy.logerr("unknown armed state")
        sys.exit("Exiting...")
        
    # auto land is started using a service
    while not start_land:
       rate.sleep()

    rospy.loginfo("starting automated landing")
    set_land_alt()
    move_to_land()
    do_land()
    rospy.loginfo("finished automated landing")

if __name__ == '__main__':
    try:
        landing_automator()
    except rospy.ROSInterruptException:
        pass
