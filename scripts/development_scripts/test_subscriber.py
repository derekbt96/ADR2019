#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Combine many other packages
# Status:   07/11: Started with script

import rospy
import signal
import sys
import math
import numpy as np
import time
from std_msgs.msg import Int32, String, Float64MultiArray, Bool, Float32
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
from bebop_auto.msg import Auto_Driving_Msg, Gate_Detection_Msg, WP_Msg
from nav_msgs.msg import Odometry
from tf import transformations as tfs
import common_resources as cr


def signal_handler():
    sys.exit(0)


def calculate_omegas(input_data):
    angles = np.unwrap(input_data[1, :])
    times = input_data[0, :]
    d_a = np.diff(angles)
    d_t = np.diff(times)
    return d_a / d_t


def callback_test_changed(data):
    global dynamic_detection_input_history
    global dynamic_omega

    if dynamic_omega is None:
        dynamic_detection_input_history = np.append(dynamic_detection_input_history,
                                                    [[data.data[0]], [data.data[1]]], axis=1)
        if dynamic_detection_input_history.shape[0] > 20:
            rospy.loginfo("enough measurements")
            dynamic_detection_input_history = np.delete(dynamic_detection_input_history, 0, axis=1)
            # calculate std deviation of list
            omegas = calculate_omegas(dynamic_detection_input_history)
            std_deviation = np.std(omegas)
            # when std dev is low enough, provide waypoint
            if std_deviation < 0.3:
                rospy.loginfo("measurements accepted")
                dynamic_omega = np.mean(omegas)
                rospy.loginfo(std_deviation)
            else:
                rospy.loginfo("standard deviation too high:")
                rospy.loginfo(std_deviation)
        else:
            rospy.loginfo("collecting measurements")
    else:
        # add to list only if gate position is close to where it's supposed to be
        t_diff = np.array(data.data[1]) - dynamic_detection_input_history[0][-1]
        angle_theory = dynamic_omega * t_diff + dynamic_detection_input_history[1][-1]
        angle_theory = angle_theory % (2*math.pi)
        diff = math.fabs(angle_theory - data.data[0])
        diff = min(diff, 2*math.pi-diff)
        if diff < 20*math.pi/180:
            rospy.loginfo("use detected pointer")
            dynamic_detection_input_history = np.append(dynamic_detection_input_history,
                                                        [[data.data[0]], [data.data[1]]], axis=1)
            dynamic_detection_input_history = np.delete(dynamic_detection_input_history, 0, axis=1)
            omegas = calculate_omegas(dynamic_detection_input_history)
            dynamic_omega = np.mean(omegas)

            # calculate current pointer position based on 5 last measurements
            t_delta = time.time() - dynamic_detection_input_history[0][-5:]
            a_delta = t_delta * dynamic_omega
            angles = a_delta + dynamic_detection_input_history[1][-5:]
            angles = angles % (2*math.pi)

            current_angle = math.atan2(np.sum(np.sin(angles)), np.sum(np.cos(angles)))
            if current_angle < 0:
                current_angle = current_angle + math.pi
            print current_angle

        else:
            rospy.loginfo("discard detected pointer")

    rospy.loginfo("dynamic_omega")
    rospy.loginfo(dynamic_omega)


if __name__ == '__main__':
    # Enable killing the script with Ctrl+C.
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('dyn_gate_test', anonymous=True)

    # Variables
    dynamic_detection_input_history = np.array([[], []])

    # Publishers

    # Subscribers
    rospy.Subscriber("/test", Float64MultiArray, callback_test_changed)

    # initializes startup by publishing state 0

    rospy.spin()
