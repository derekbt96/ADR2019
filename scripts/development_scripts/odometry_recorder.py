#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Combine many other packages
# Status:   07/11: Started with script

import rospy
import signal
import sys
import time
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
import common_resources as cr


def signal_handler(signal, frame):
    sys.exit(0)


def callback_zed_odometry_changed(data):
    rospy.loginfo(["zed", data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z, data.pose.pose.orientation.w,
                  data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z])


def callback_bebop_odometry_changed(data):
    rospy.loginfo(["bebop", data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z, data.pose.pose.orientation.w,
                  data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z])


def callback_states_changed(data):
    global state_auto
    state_auto = data.data
    rospy.loginfo("state auto changed to " + str(state_auto))


if __name__ == '__main__':
    # Enable killing the script with Ctrl+C.
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('main_navigation', anonymous=True)
    rate = rospy.Rate(cr.frequency)

    state_auto_publisher = rospy.Publisher("/auto/state_auto", Int32, queue_size=1, latch=True)
    rospy.Subscriber("/bebop/odom", Odometry, callback_bebop_odometry_changed)
    rospy.Subscriber("/zed/odom", Odometry, callback_zed_odometry_changed)
    rospy.Subscriber("/auto/state_auto", Int32, callback_states_changed)

    state_auto = None

    # initializes startup by publishing state 0
    state_auto_publisher.publish(0)

    # Wait until connection between ground and air is established
    while state_auto is None:
        state_auto_publisher.publish(0)
        rospy.loginfo("waiting None")
        time.sleep(0.5)
    while state_auto == 0:
        rospy.loginfo("waiting 0")
        time.sleep(0.5)
    while state_auto == 1:
        state_auto_publisher.publish(state_auto + 1)
        rospy.loginfo("waiting 1")
        time.sleep(0.5)
    state_auto_publisher.publish(2)

    rospy.loginfo("Jetson communicating")

    rospy.spin()
