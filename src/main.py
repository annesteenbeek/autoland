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
from mavros_msgs.msg import State, Waypoint 
from mavros_msgs.srv import CommandBool, SetMode, WaypointPush
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry 
from autoland.srv import *

# set variables
target = PoseStamped() # setpoint target to meet
DESCEND_SPEED = 0.4 # descend speed [m/s]
COMM_RATE = 100.0 # ros rate [hz]
rate = None
ACC_RAD = 0.2 # acceptence radius for setpoint [m]
MAX_APPROACH_SPEED = 0.1
allowed_deviation = 0.1 # radius to exceed to stop descend [m]
LAND_ALT = 3 # altitude for landing [m]
rtk_start_x = None # relative to piksi 0
rtk_start_y = None 
local_start_z = None
local_start_x = None
local_start_y = None
LOITER_TIME = 3. # time to loiter bevore descending [sec]
DISARM_ALT = 0.4 # altitude to turn off engines [m]
landed = False
effort_x = 0
effort_y = 0
rate = 0
cur_local_pose = None
external_pose = None
current_mode = ""
prev_mode = ""
armed = None
start_land = False

# for z p filter
min_speed_z = -1
max_speed_z = 1
min_speed_xy = -1
max_speed_xy = 1
z_p = 0.2
xy_p = 0.2

def pose_cb(pose):
    global cur_local_pose
    cur_local_pose = pose

def state_cb(state):
    global current_mode, prev_mode, armed
    current_mode = state.mode
    armed = state.armed
    if current_mode != prev_mode:
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

def is_guided():
    result = False
    if current_mode is "":
        rospy.logerr("Unable to retrieve mode, not yet available")
    elif current_mode is "OFFBOARD" or current_mode is "GUIDED":
        result = True
    return result

def is_ready():
    return False if \
            cur_local_pose is None or \
            external_pose is None or \
            armed is None \
            else True
            
# if land pos is set, keep sending current position
def is_start_autoland():
    if not is_guided() and rtk_start_x is not None:
        target = PoseStamped()
        target.header.frame_id = 'fcu'
        target.pose= cur_local_pose.pose
        target.header.stamp = rospy.get_rostime()
        local_pos_pub.publish(target)
    elif is_guided() and rtk_start_x is not None: 
        rospy.loginfo("landing at x %.2f y %.2f" % (rtk_start_x, rtk_start_y))
        return True
    return False

def pid_cb(effort, direction):
    global effort_x, effort_y
    if direction is "x":
        effort_x = effort.data
    elif direction is "y":
        effort_y = effort.data
    else:
        rospy.logerr("Incorrect pid cb direction")

def set_topics():
    global local_pos_pub, velocity_cmd_pub, state_sub, pos_sub, external_pos_sub, set_mode_pub
    mavros.set_namespace()
    local_pos_pub = setpoint.get_pub_position_local(queue_size=10, latch=True)
    velocity_cmd_pub = setpoint.get_pub_velocity_cmd_vel(queue_size=10, latch=True)
    pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_cb)
    external_pos_sub = rospy.Subscriber("external_pose_estimation", PoseStamped, external_cb)
    state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)
    rospy.Service('start_autoland', StartAutoland, set_start_land)
    set_mode_pub = rospy.ServiceProxy("mavros/set_mode", SetMode)

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
    global rtk_start_x, rtk_start_y, local_start_x, local_start_y, local_start_z, local_start_orientation
    if external_pose is None:
        rospy.logerr("No external positions available")
        sys.exit("unable to set landing pos, exiting...")
    else:
        rtk_start_x = external_pose.pose.position.x
        rtk_start_y = external_pose.pose.position.y
        local_start_z = cur_local_pose.pose.position.z
        local_start_x = cur_local_pose.pose.position.x
        local_start_y = cur_local_pose.pose.position.y
        local_start_orientation = cur_local_pose.pose.orientation
        rospy.loginfo("landing pos has been set")


# set correct altitude for landing
def set_land_alt():
    rospy.loginfo("Moving to landing altitude")
    target = PoseStamped()
    target.pose.position.x = cur_local_pose.pose.position.x
    target.pose.position.y = cur_local_pose.pose.position.y
    target.pose.position.z = LAND_ALT + local_start_z

    while not get_distance(target) <= ACC_RAD: 
        target.header.stamp = rospy.get_rostime()
        local_pos_pub.publish(target)
        rate.sleep()
    rospy.loginfo("Reached altitude with distance %.3f" % get_distance(target))

# position above landing position
def move_to_land():
    rospy.loginfo("Moving above land target")
    target = PoseStamped()
    target.pose.position.x = local_start_x
    target.pose.position.y = local_start_y
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
    target.pose.position.x = local_start_x
    target.pose.position.y = local_start_y
    target.pose.position.z = local_start_z + LAND_ALT
    target.pose.orientation = local_start_orientation # set start yaw by copying start orientation (only yaw is used)
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
    rospy.loginfo("AUTOLAND loitering done, starting descent")
    velocity_cmd_pub = setpoint.get_pub_velocity_cmd_vel(queue_size=10)
    rospy.Subscriber('effort_x', Float64, pid_cb, "x", 1)
    rospy.Subscriber('effort_y', Float64, pid_cb, "y", 1) 

    # create setpoint publishers
    set_x_pub = rospy.Publisher("autoland_setpoint_x", Float64, latch=True, queue_size=1)
    set_y_pub = rospy.Publisher("autoland_setpoint_y", Float64, latch=True, queue_size=1)
   
    # create plant state publishers
    pose_x_pub = rospy.Publisher("autoland_pose_x", Float64, queue_size=3)
    pose_y_pub = rospy.Publisher("autoland_pose_y", Float64, queue_size=3)

    hold_height = None # altitude to hold when not above target

    while not landed:

        pos_x = external_pose.pose.position.x
        pos_y = external_pose.pose.position.y
        pos_z = external_pose.pose.position.z

        dist_to_land = math.sqrt(
                 math.pow(rtk_start_x - pos_x, 2) + math.pow(rtk_start_y - pos_y,2)
                )

        if dist_to_land > allowed_deviation:
            above_target = False
            if hold_height is None:
                hold_height = pos_z
                rospy.loginfo("not above target, holding alt")
        else:
            above_target = True
            if hold_height is not None:
                hold_height = None # reset hold height
                rospy.loginfo("above target, descending")

        # publish x and y position
        pose_x_pub.publish(pos_x)
        pose_y_pub.publish(pos_y)

        # send landing setpoints
        set_x_pub.publish(rtk_start_x)
        set_y_pub.publish(rtk_start_y)

        speed_x = effort_x
        speed_y = effort_y
        
        if above_target:
            speed_z = - DESCEND_SPEED
        else:
            speed_z = z_p * (hold_height - pos_z)
            # enforce min/max
            if speed_z > max_speed_z:
                speed_z = max_speed_z
            elif speed_z < min_speed_z:
                speed_z = min_speed_z

        # speed_x = xy_p * (rtk_start_x - pos_x)
        # # enforce min/max
        # if speed_x > max_speed_xy:
        #     speed_x = max_speed_xy
        # elif speed_x < min_speed_xy:
        #     speed_x = min_speed_xy

        # speed_y = xy_p * (rtk_start_y - pos_y)
        # # enforce min/max
        # if speed_y > max_speed_xy:
        #     speed_y = max_speed_xy
        # elif speed_y < min_speed_xy:
        #     speed_y = min_speed_xy
      
        vel = setpoint.TwistStamped(header=setpoint.Header(frame_id='mavsetp', stamp=rospy.get_rostime()))
        vel.twist.linear = setpoint.Vector3(x=speed_x, y=speed_y, z=speed_z)
        vel.twist.angular = setpoint.Vector3(z=0)
        velocity_cmd_pub.publish(vel)
        rate.sleep()

        set_speed_total = math.sqrt(speed_x**2 + speed_y**2)

        if (pos_z - local_start_z) <= DISARM_ALT: #and set_speed_total <= MAX_APPROACH_SPEED:
            # TODO go straight down until landing is detected
            rospy.loginfo("Quadcopter landed!")
            rospy.loginfo("x pos: %.3fm" % (pos_x - rtk_start_x))
            rospy.loginfo("y pos: %.3fm" % (pos_y - rtk_start_y))
            rospy.loginfo("Total distance: %.3fm" % dist_to_land)
            rospy.loginfo("At altitude %.3fm" % (pos_z - local_start_z))
            set_mode_pub(custom_mode="LAND")
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
    # command.setup_services()
    
    if not armed:
       # wait unit vehicle is armed and store that location
       got_land = False
       rospy.loginfo("not armed, waiting")
       while not got_land: 
            if armed:
                # make sure external pose is available
                while external_pose is None:
                    rate.sleep()
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
        
    # wait until offboard button is switched
    while not is_start_autoland():
       rate.sleep()

    rospy.loginfo("starting automated landing")
    rospy.loginfo("setting landing altitude")
    set_land_alt()
    rospy.loginfo("moving to landing position")
    move_to_land()
    stabalize_above_land()
    do_land()
    rospy.loginfo("finished automated landing")

if __name__ == '__main__':
    try:
        landing_automator()
    except rospy.ROSInterruptException:
        pass
