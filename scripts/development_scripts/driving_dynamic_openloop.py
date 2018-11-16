#!/usr/bin/env python

# Script developed by Vincenz Frenzel
# This file is for testing purposes only to fly open loop with full speed and log distances covered
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import signal
import sys
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
from bebop_auto.msg import Auto_Driving_Msg
from nav_msgs.msg import Odometry
import common_resources as cr
import time
import numpy as np


def signal_handler(signal, frame):
    sys.exit(0)


def publish_command(x,y,z,r):
    if not rospy.is_shutdown():
        msg = Twist()  # [0,0,0],[0,pitch,yaw])
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        msg.angular.z = r
        cmd_vel_pub.publish(msg)
    else:
        rospy.loginfo("flight command not sent")


def publish_status(st):
    pub = rospy.Publisher('/bebop/' + st, Empty, queue_size=1, latch=True)
    if not rospy.is_shutdown():
        msg = Empty()
        pub.publish(msg)
    else:
        rospy.loginfo("status command not sent")


def callback_states_changed(data):
    # update variables
    global state_bebop
    state_bebop = data.state


def callback_bebop_odometry_changed(data):
    global timer
    global state_auto
    global bebop_p
    global counter

    # BEBOP STATE overview
    #   0   landed
    #   1   takeoff
    #   2   hovering
    #   3   flying
    #   4   landing
    #   5   emergency
    #   6   not observed (usertakeoff, User take off state. Waiting for user action to take off)
    #   7   not observed (for fixed wing, motor ramping state)
    #   8   not observed (emergency landing after sensor defect. Only YAW is taken into account)

    # rospy.loginfo("state machine sees: auto " + str(state_auto) + " and bebop " + str(state_bebop))
    if state_auto == 1:
        rospy.loginfo("takeoff")
        publish_status("takeoff")
        if state_bebop == 1:                           # air received response from ground
            rospy.loginfo("drone takes off")
            state_auto = state_auto + 1                                    # 3 - drone takes off

    elif state_auto == 2 and state_bebop == 2:                         # drone is hovering/flying
        timer = time.time()
        rospy.loginfo("drone hovers 2 sec")
        state_auto = state_auto + 1                                    # 4 - drone hovers 2 sec

    elif state_auto == 3 and timer + 0.15 < time.time():  # drone has hovered for 2 sec
        state_auto = state_auto + 1
        timer = time.time()
        counter = 0
        bebop_position = data.pose.pose.position
        bebop_p = np.array([bebop_position.x, bebop_position.y, 0])

    elif state_auto == 4:  # drone has hovered for 2 sec
        rospy.loginfo("accelerate")
        drive_msg = Auto_Driving_Msg()
        drive_msg.x = 1.0
        publish_command(drive_msg.x, drive_msg.y, drive_msg.z, drive_msg.r)
        counter = counter + 1
        rospy.loginfo("fwd: " + "{:.2f}".format(drive_msg.x) + " | left: " + "{:.2f}".format(
            drive_msg.y) + " | up: " + "{:.2f}".format(drive_msg.z) + " | ccw: " + "{:.2f}".format(drive_msg.r))
        # if counter == 5:
        #     rospy.loginfo("accelerate 0.5")
        #     drive_msg = Auto_Driving_Msg()
        #     drive_msg.x = 0.5
        #     publish_command(drive_msg.x, drive_msg.y, drive_msg.z, drive_msg.r)
        #     counter = counter + 1
        #     rospy.loginfo("fwd: " + "{:.2f}".format(drive_msg.x) + " | left: " + "{:.2f}".format(
        #         drive_msg.y) + " | up: " + "{:.2f}".format(drive_msg.z) + " | ccw: " + "{:.2f}".format(drive_msg.r))
        if counter >= 6:
            state_auto = state_auto + 1
            timer = time.time()

    elif state_auto == 5:  # drone has accel for x sec
        rospy.loginfo("brake")
        drive_msg = Auto_Driving_Msg()
        publish_command(drive_msg.x, drive_msg.y, drive_msg.z, drive_msg.r)
        rospy.loginfo("fwd: " + "{:.2f}".format(drive_msg.x) + " | left: " + "{:.2f}".format(
            drive_msg.y) + " | up: " + "{:.2f}".format(drive_msg.z) + " | ccw: " + "{:.2f}".format(drive_msg.r))
        if state_bebop == 2:
            rospy.loginfo("braking took " + str(time.time()-timer))
            state_auto = state_auto + 1

    elif state_auto == 6:
        bebop_position = data.pose.pose.position
        bebop_p2 = np.array([bebop_position.x, bebop_position.y, 0])
        print np.linalg.norm(bebop_p2 - bebop_p)
        rospy.loginfo("land")
        publish_status("land")
        if state_bebop == 4:
            rospy.loginfo("landing")
            state_auto = state_auto + 1

    elif state_auto == 7 and state_bebop == 0:
        rospy.loginfo("landed")
        bebop_position = data.pose.pose.position
        bebop_p2 = np.array([bebop_position.x, bebop_position.y, 0])
        print np.linalg.norm(bebop_p2 - bebop_p)
        state_auto = state_auto + 1
        quit()


if __name__ == '__main__':
    # Enable killing the script with Ctrl+C.
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('driving_dynamic_openloop', anonymous=True)

    # Global variables for autonomy mode, and the status of the drone and the state machine
    state_auto = -1
    state_bebop = -1
    bebop_p = None
    counter = None

    # create a state machine publisher and a global command publisher
    cmd_vel_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1, latch=True)
    rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged,
                     callback_states_changed)
    rospy.Subscriber("/bebop/odom", Odometry, callback_bebop_odometry_changed)

    rospy.loginfo("ready")

    state_auto = 1

    rospy.spin()