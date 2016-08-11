#!/usr/bin/env python

import numpy
import rospy
import mavros
from mavros.utils import *
from mavros import command, mission, setpoint
from geometry_msgs.msg import PoseStamped, Point, Pose
from mavros.msg import State, Waypoint 
from mavros.srv import CommandBool, SetMode, WaypointPush

cur_local_pose = PoseStamped()
cur_state = State()
external_postion = Point()

def pose_cb(pose):
    global cur_local_pose
    cur_local_pose = pose

def state_cb(state):
    global current_state
    current_state = state

def external_cb(point):
    global external_position
    print(point)
    external_position = point

local_pos_pub = setpoint.get_pub_position_local(queue_size=10, latch=True)
velocity_cmd_pub = setpoint.get_pub_velocity_cmd_vel(que_size = 1, latch=True)
local_pos_sub = rospy.Subscriber(mavros.get_topic('local_position', 'local'), PoseStamped, pose_cb)
state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)

# get external position information from gazebo/sensors
# external_pos_sub = rospy.Subscriber('/gazebo/model_states/pose', Pose, external_cb)

target = PoseStamped() # setpoint target to meet
DESCEND_SPEED = 0.0 # descend speed [m/s]
ACC_RAD = 20.0 # acceptence radius for setpoint [cm]
LAND_ALT = 12 # altitude for landing [m]
LAND_X = 0
LAND_Y = 0
LOITER_TIME = 5. # time to loiter bevore descending [sec]
landState = 'MOVE_AWAY' # counter for landing progress
prevState = landState
landed = False

def arm(state):
    try:
        ret_arm = command.arming(state)
    except rospy.ServiceException as ex:
        fault(ex)
        
    if ret_arm.success:
        rospy.loginfo("Copter armed state: %r" % state)
    else:
        rospy.loginfo("Unable to arm/disarm copter")
    return ret_arm.success

def set_offboard(state):
    try:
        ret_mode = command.guided_enable(value=state)
    except rospy.ServiceException as ex:
        fault(ex)

    if ret_mode.success:
        rospy.loginfo("Copter OFFBOARD: %r" % state)
    else: 
        rospy.loginfo("Unable to enable/disable OFFBOARD")
    return ret_mode.success

# calculate distance between two points returns [cm]
def get_distance(target):
    # NOTE this could probably be done with TF
    dx = target.pose.position.x - cur_local_pose.pose.position.x
    dy = target.pose.position.y - cur_local_pose.pose.position.y
    dz = target.pose.position.z - cur_local_pose.pose.position.z

    dv = numpy.array((dx, dy, dz))
    dist = numpy.linalg.norm(dv) * 100 # return in cm
    # print("distance to target: %.1fcm" % dist)
    return dist 

# set mode and set offboard
def setup_copter():
    pose = PoseStamped()
    pose.pose.position.x = 5
    pose.pose.position.y = 5
    pose.pose.position.z = 8

    rate = rospy.Rate(20.0) # MUST be more then 2Hz
    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()
    # set mode to OFFBOARD 
    offb_res = set_offboard(True)    
    # Arm the copter
    arm_res = arm(True)
    
    # TODO replace with actual state tests, res always returns success=True
    return (offb_res and arm_res)

# move away (Should only be used for tests to move copter into random position)    
def move_away():
    global target
    pose = PoseStamped()
    pose.pose.position.x = 5
    pose.pose.position.y = 5
    pose.pose.position.z = 8
    
    target = pose
    pose.header.stamp = rospy.get_rostime()
    local_pos_pub.publish(pose)

# set correct altitude for landing
def set_land_alt():
    global target
    pose = PoseStamped()
    # get prev target xy (should grab current pose x/y)
    pose.pose.position.x = target.pose.position.x 
    pose.pose.position.y = target.pose.position.y 
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
    rospy.loginfo("REACHED AUTOLAND")
    global target
    stabalize_above_land()
    rospy.loginfo("AUTOLAND loitering done, starting descent")
    # TODO currently using local pos, should use external estimator
    landed = False
    while not landed:
        error_x = cur_local_pose.pose.position.x - LAND_X
        error_y = cur_local_pose.pose.position.Y - LAND_Y
        speed_x = error_x * Pgain
        speed_y = error_y * Pgain
        speed_z = - DESCEND_SPEED
        
        vel = setpoint.TwistStamped(header=setpoint.Header(frame_id='mavsetp', stamp=rospy.get_rostime()))
        vel.twist.linear = setpoint.Vector3(x=speed_x, y=speed_y, z=speed_z)
        vel.twist.angular = setpoint.Vector3(z=0)
        velocity_cmd_pub(vel)
        rate.sleep()
        if cur_local_pose.pose.position.z <= 0.4:
            landed = True

# disarm and shutdown copter
def finish_land():
    global landed
    rospy.loginfo("FINISHED LAND")
    landed = True
    # show results

def advance_state():
    global landState, prevState
    if landState is 'MOVE_AWAY':
        landState = 'SET_ALT'
            
    elif landState is 'SET_ALT':
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
            'MOVE_AWAY': move_away,
            'SET_ALT': set_land_alt,
            'MOVE_LAND': move_to_land,
            'AUTOLAND': autoland,
            'FINISH_LAND': finish_land,
            }
    switcher[landState]() # execute correct state function
            
def landing_automator():
    rospy.init_node('autoland_node', anonymous=True)
    rate = rospy.Rate(20.0) # MUST be more then 2Hz
    # wait for FCU connection
    wait_fcu_connection()
    command.setup_services()

    if setup_copter():
        move_away() # move copter to random position
        while not rospy.is_shutdown() and not landed:
            if get_distance(target) <= ACC_RAD:
                advance_state()
                handle_state()
            target.header.stamp = rospy.get_rostime()    
            local_pos_pub.publish(target)
            rate.sleep()
    else:
        rospy.loginfo('unable to start copter')

if __name__ == '__main__':
    try:
        landing_automator()
    except rospy.ROSInterruptException:
        pass
