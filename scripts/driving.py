#!/usr/bin/env python

# Script developed by Vincenz Frenzel
#  --- Changelog ---
# Goal:     Input from ground or from navigation. Ground overrides navigation. Publish to drone.
# Status:   06/19: Not existing
#           06/25: Drone takes off autonomously and lands autonomously on the same location
#           06/27: Drone takes off autonomously, flies to a virtual target and lands after reaching the target
#           11/03: Commented code

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import signal
import sys
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
from std_msgs.msg import Bool, Int32
from bebop_auto.msg import Auto_Driving_Msg
import common_resources as cr


def signal_handler(_, __):
    sys.exit(0)


# publish flying commands to drone
def publish_command(x, y, z, r):
    if not rospy.is_shutdown():
        msg = Twist()  # [0,0,0],[0,pitch,yaw])
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        msg.angular.z = r
        cmd_vel_pub.publish(msg)
    else:
        rospy.loginfo("flight command not sent")


# publish status to drone (takeoff, land, ...)
def publish_status(st):
    pub = rospy.Publisher('/bebop/' + st, Empty, queue_size=1, latch=True)
    if not rospy.is_shutdown():
        msg = Empty()
        pub.publish(msg)
    else:
        rospy.loginfo("status command not sent")


# update autonomous state
def callback_states_changed(data):
    global state_auto
    state_auto = data.data


# update autonomous active bool
def callback_autonomous_driving(data):
    print("autonomy " + str(data))
    global autonomy_active
    autonomy_active = data.data


# update current driving message
def callback_autonomous_drive_msg_changed(data):
    global drive_msg
    drive_msg = data


if __name__ == '__main__':
    # Enable killing the script with Ctrl+C.
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('driving', anonymous=True)

    # Global variables for autonomy mode, and the status of the drone and the state machine
    autonomy_active = False
    state_auto = -1

    # create a state machine publisher and a global command publisher
    cmd_vel_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1, latch=True)
    rospy.Subscriber("/auto/state_auto", Int32, callback_states_changed)
    rospy.Subscriber("/auto/autonomous_driving", Bool, callback_autonomous_driving)
    rospy.Subscriber("/auto/auto_drive", Auto_Driving_Msg, callback_autonomous_drive_msg_changed)

    # run with 20Hz
    rate = rospy.Rate(20)

    rospy.loginfo("ready")

    while True:
        rate.sleep()

        # only do something if autonomy is active
        if autonomy_active:
            # state 2: takeoff
            if state_auto == 2:
                rospy.loginfo("takeoff")
                publish_status("takeoff")
            # state 90: land
            elif state_auto == 90:
                rospy.loginfo("land")
                publish_status("land")

            # always send driving commands
            publish_command(drive_msg.x, drive_msg.y, drive_msg.z, drive_msg.r)
            rospy.loginfo("fwd: " + "{:.2f}".format(drive_msg.x) + " | left: " + "{:.2f}".format(
                drive_msg.y) + " | up: " + "{:.2f}".format(drive_msg.z) + " | ccw: " + "{:.2f}".format(drive_msg.r))

        # otherwise don't do anything
        else:
            pass
            # publish_command(0,0,0,0)
