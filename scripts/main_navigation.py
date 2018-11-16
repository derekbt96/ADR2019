#!/usr/bin/env python

# Script developed by Vincenz Frenzel (navigation by Derek Thompson)
#  --- Changelog ---
# Goal:     Main logic of autonomy
# Status:   07/11: Started with script
#           11/02: Added last comments to script

import rospy
import signal
import sys
import math
import numpy as np
import time
from std_msgs.msg import Int32, String, Float64MultiArray, Float32MultiArray, Bool, Float32, Empty
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
from bebop_auto.msg import Auto_Driving_Msg, Gate_Detection_Msg, WP_Msg
from nav_msgs.msg import Odometry
from tf import transformations as tfs
import common_resources as cr


def signal_handler(_, __):
    # enable Ctrl+C of the script
    sys.exit(0)


def callback_states_changed(data, args):
    # update states, either autonomy state or bebop state
    if args == "state_auto":
        global state_auto
        state_auto = data.data
        rospy.loginfo("state auto changed to " + str(state_auto))
    elif args == "state_bebop":
        global state_bebop
        state_bebop = data.state
        rospy.loginfo("state bebop changed to " + str(state_bebop))


def callback_visual_gate_dynamic_changed(input_data):
    # handling measurements of pointer angle of dynamic gate
    global detection_dynamic_input_history
    global detection_dynamic_data

    measurement = input_data.data  # vector=[time, angle]

    # if fast navigation is active
    if nav_active == "fast":
        # append measurement to history
        detection_dynamic_input_history = np.append(detection_dynamic_input_history, [[measurement[0], measurement[1]]],
                                                    axis=0)
        # if there are 5 measurements and it is not triggered yet, see if it should be triggered
        if detection_dynamic_input_history.shape[0] > 5 and not detection_dynamic_data.triggered:
            data = detection_dynamic_input_history
            # Resolution determines how accurate the fourier transform is
            resolution = 0.001

            plots = [False, False, False, False] # turn off all plots

            # Frequency returned in [Hz]
            [freq, offset] = cr.extract_freq(data, resolution, plots[0], plots[1])
            # detect rotation direction by unwrapping all angles, diff and evaluating which sign (+/-) dominates
            # if minus dominates, invert frequency
            if np.sum(np.sign(np.diff(np.unwrap(np.transpose(detection_dynamic_input_history)[1][:])))) < 0:
                freq = -freq
            global detection_dynamic_freqs
            freq = -0.5  # hard coded frequency should be removed for this to work properly
            # add frequencies to a list
            detection_dynamic_freqs = np.append(detection_dynamic_freqs,freq)

            # if there are 5 frequencies collected, determine their standard deviation
            if detection_dynamic_freqs.shape[0] > 5:
                dev = np.std(detection_dynamic_freqs)
            else:
                dev = 2
            dev = 0.00000001  # hard coded deviation because of hard coded frequency need to be removed

            # delete oldest measurement if there are more than 20 because cr.extract_freq would take too long eventually
            if detection_dynamic_freqs.shape[0] > 20:
                detection_dynamic_freqs = np.delete(detection_dynamic_freqs, 0)

            # prepare all variables so trigger is as fast as possible
            theta_current = None    # current angle
            cur_t = time.time()     # current time
            theta_trigger = None    # trigger angle
            angle_diff = None       # angle difference between current and trigger
            exec_time = None        # trigger time
            a_dev = None            # angle std deviation

            if dev < 0.01 and abs(freq)>0.01:

                # following line is not quite functional
                # current_angle = cr.angle_in_t_seconds(data, freq, offset, t_future, plots[2], plots[3])

                # trigger script works with periods not frequencies
                detection_dynamic_data.period = 1.0/freq
                # elapsed time since last two measurements
                t_delta = cur_t - np.transpose(detection_dynamic_input_history)[0][-2:]
                # angles that were passed with this period since last two measurements
                a_delta = 2 * math.pi * t_delta / detection_dynamic_data.period
                # current angle based on last two measurements
                angles = a_delta + np.transpose(detection_dynamic_input_history)[1][-2:]
                angles = np.unwrap(angles)
                # calculate their std. deviation after unwrapping them
                a_dev = np.std(angles)
                # if std deviation is low enough, accept current angle
                if a_dev < 30*math.pi/180:
                    theta_current = math.atan2(np.sum(np.sin(angles)), np.sum(np.cos(angles)))
                    # make sure it's positive
                    if theta_current < 0:
                        theta_current = theta_current + 2 * math.pi
                    detection_dynamic_data.theta = theta_current

                    # calculate trigger angle
                    theta_trigger = -2 * math.pi * detection_dynamic_data.time_taken_to_gate / detection_dynamic_data.period
                    # make sure trigger angle is in [0,2pi]
                    while not (0 <= theta_trigger <= 2 * math.pi):
                        if theta_trigger <= 0:
                            theta_trigger = theta_trigger + 2 * math.pi
                        else:
                            theta_trigger = theta_trigger - 2 * math.pi

                    # calculate angle difference (sign of period required to account for rotation direction)
                    angle_diff = (theta_trigger - theta_current)*np.sign(detection_dynamic_data.period)
                    # make sure angle difference is in [0,2pi]
                    while not (0 <= angle_diff <= 2 * math.pi):
                        if angle_diff <= 0:
                            angle_diff = angle_diff + 2 * math.pi
                        else:
                            angle_diff = angle_diff - 2 * math.pi

                    # execution time is based on angle_difference, period and current time (at start of this script)
                    exec_time = cur_t + angle_diff / (2*math.pi / (abs(detection_dynamic_data.period)))

                    # do logging before the trigger because trigger hold entire script for a while
                    log_string = str(
                        0) + ", " + str(
                        freq) + ", " + str(
                        offset) + ', ' + str(
                        measurement[0]) + ", " + str(
                        measurement[1]) + ", " + str(
                        dev) + ', ' + str(
                        theta_current or 0) + ', ' + str(
                        cur_t) + ", " + str(
                        theta_trigger or 0) + ', ' + str(
                        angle_diff or 0) + ", " + str(
                        exec_time or 0) + ", " + str(
                        1) + ", " + str(
                        a_dev or 0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        time.time()) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0)
                    publisher_nav_log.publish(log_string)

                    # if trigger is already triggered (by another instance that was running in parallel), don't trigger
                    if detection_dynamic_data.triggered:
                        return
                    else:
                        # set triggered flag right away (so only one instance can trigger)
                        detection_dynamic_data.triggered = True
                        # wait for the right time
                        while exec_time > time.time():
                            time.sleep(0.01)
                        # execute throttle for 1.5 seconds and send brake command until full stop
                        full_throttle_executer(1.5)
                        # turn off navigation (advance in state)
                        global nav_active
                        nav_active = "off"
                        return

            # send logs also if trigger was never reached
            log_string = str(
                0) + ", " + str(
                freq) + ", " + str(
                offset) + ', ' + str(
                measurement[0]) + ", " + str(
                measurement[1]) + ", " + str(
                dev) + ', ' + str(
                theta_current or 0) + ', ' + str(
                cur_t) + ", " + str(
                theta_trigger or 0) + ', ' + str(
                angle_diff or 0) + ", " + str(
                exec_time or 0) + ", " + str(
                0) + ", " + str(
                a_dev or 0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                time.time()) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0)

            publisher_nav_log.publish(log_string)


def callback_visual_gate_detection_changed(data):
    global wp_average
    global wp_input_history

    # detection must be active
    if not detection_active:
        rospy.loginfo("detection not active")
        log_string = "0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0"
        publisher_visual_log.publish(log_string)
        return

    # content must not be empty
    if data.tvec == ():
        rospy.loginfo("empty visual input")
        log_string = "0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0"
        publisher_visual_log.publish(log_string)
        return

    rospy.loginfo("visual gate detected")

    # read data
    bebop_position = data.bebop_pose.position
    bebop_orientation = data.bebop_pose.orientation

    # bebop position and orientation
    bebop_p = np.array([bebop_position.x, bebop_position.y, bebop_position.z])
    bebop_q = np.array([bebop_orientation.x, bebop_orientation.y, bebop_orientation.z, bebop_orientation.w])

    # gate position and orientation
    # rospy.loginfo("tvec")
    # rospy.loginfo(data.tvec)
    # rospy.loginfo("rvec")
    # rospy.loginfo(data.rvec)

    gate_pos = np.array(data.tvec)
    gate_q = cr.axang2quat(np.array(data.rvec))
    #
    # rospy.loginfo("bebop_p")
    # rospy.loginfo(bebop_p)
    # rospy.loginfo("bebop_q")
    # rospy.loginfo(bebop_q)
    # rospy.loginfo("gate_pos")
    # rospy.loginfo(gate_pos)
    # rospy.loginfo("gate_q")
    # rospy.loginfo(gate_q)

    # transform measurement into global frame, calculate headings
    # headings are required to make sure gate is not backwards (it is 1*pi periodic so could be wrong way round)
    gate_global_p = cr.qv_mult(bebop_q, cr.qv_mult(cr.cam_q, gate_pos) + cr.BZ) + bebop_p
    gate_global_q = tfs.quaternion_multiply(bebop_q, tfs.quaternion_multiply(cr.cam_q, gate_q))
    gate_normal_vec = cr.qv_mult(gate_global_q, [0, 0, 1])
    heading_to_gate = math.atan2((gate_global_p[1] - bebop_p[1]), gate_global_p[0] - bebop_p[0])
    heading_of_gate = math.atan2(gate_normal_vec[1], gate_normal_vec[0])
    heading_difference = math.fabs(heading_to_gate - heading_of_gate) * 180 / math.pi
    
    # rospy.loginfo("gate_global_p")
    # rospy.loginfo(gate_global_p)
    # rospy.loginfo("gate_global_q")
    # rospy.loginfo(gate_global_q)
    # rospy.loginfo("gate_normal_vec")
    # rospy.loginfo(gate_normal_vec)

    if 90 < heading_difference < 270:
        if heading_of_gate < 0:
            heading_of_gate = heading_of_gate + math.pi
        else:
            heading_of_gate = heading_of_gate - math.pi

    # accept info as current WP. publish for ground control station
    wp_current = cr.WP(gate_global_p, heading_of_gate)
    msg = WP_Msg()
    msg.pos.x = wp_current.pos[0]
    msg.pos.y = wp_current.pos[1]
    msg.pos.z = wp_current.pos[2]
    msg.hdg = wp_current.hdg
    publisher_wp_current.publish(msg)

    rospy.loginfo("new measured wp_current")
    rospy.loginfo(wp_current)

    # while there is no average yet: collect gate positions
    if wp_average is None:
        wp_input_history.append(wp_current)
        # once enough measurements are there, delete oldest ones
        if len(wp_input_history) > 15:
            rospy.loginfo("enough measurements")
            del wp_input_history[0]
            # calculate average of measurements (average position and angle)
            average = cr.find_average(wp_input_history)
            # calculate std deviation of positions
            std_deviation = cr.find_std_dev_waypoints(average, wp_input_history)
            # when std dev is low enough, provide waypoint ("lock on" to average gate)
            if std_deviation < 0.25:
                rospy.loginfo("measurements accepted")
                wp_average = average
            else:
                rospy.loginfo("standard deviation too high:")
                rospy.loginfo(std_deviation)
        else:
            # not enough measurements yet
            std_deviation = 5
            average = cr.WP([0, 0, 0], 0)
            rospy.loginfo("collecting measurements")

        # logging for matlab
        log_string = str(wp_current.pos[0]) + ", " + \
            str(wp_current.pos[1]) + ", " + \
            str(wp_current.pos[2]) + ", " + \
            str(wp_current.hdg) + ", " + \
            str(average.pos[0]) + ", " + \
            str(average.pos[1]) + ", " + \
            str(average.pos[2]) + ", " + \
            str(average.hdg) + ", " + \
            str(bebop_p[0]) + ", " + \
            str(bebop_p[1]) + ", " + \
            str(bebop_p[2]) + ", " + \
            str(bebop_q[3]) + ", " + \
            str(bebop_q[0]) + ", " + \
            str(bebop_q[1]) + ", " + \
            str(bebop_q[2]) + ", " + \
            str(heading_to_gate) + ", " + \
            str(std_deviation) + ", " + \
            str(1.0) + ", " + \
            str(time.time()-t_log)
    else:
        # now, average is established
        # add measurement to list only if gate position and hdg is close to average
        distance = np.linalg.norm(wp_current.pos - wp_average.pos)
        hdg_diff = abs(math.atan2(math.sin(wp_current.hdg - wp_average.hdg), math.cos(wp_current.hdg - wp_average.hdg)))
        if distance < 0.4 and hdg_diff < 15*math.pi/180:
            # measurements are accepted. update history and recalculate average
            rospy.loginfo("use detected gate")
            wp_input_history.append(wp_current)
            del wp_input_history[0]
            wp_average = cr.find_average(wp_input_history)
            rospy.loginfo("wp_average")
            rospy.loginfo(wp_average)
        else:
            # measurement rejected
            rospy.loginfo("discard detected gate")

        # logging for matlab
        log_string = str(wp_current.pos[0]) + ", " + \
            str(wp_current.pos[1]) + ", " + \
            str(wp_current.pos[2]) + ", " + \
            str(wp_current.hdg) + ", " + \
            str(wp_average.pos[0]) + ", " + \
            str(wp_average.pos[1]) + ", " + \
            str(wp_average.pos[2]) + ", " + \
            str(wp_average.hdg) + ", " + \
            str(bebop_p[0]) + ", " + \
            str(bebop_p[1]) + ", " + \
            str(bebop_p[2]) + ", " + \
            str(bebop_q[3]) + ", " + \
            str(bebop_q[0]) + ", " + \
            str(bebop_q[1]) + ", " + \
            str(bebop_q[2]) + ", " + \
            str(heading_to_gate) + ", " + \
            str(distance) + ", " + \
            str(2.0) + ", " + \
            str(time.time()-t_log)

    publisher_visual_log.publish(log_string)

    # WP for ground control station
    msg = WP_Msg()
    if wp_average is not None:
        msg.pos.x = wp_average.pos[0]
        msg.pos.y = wp_average.pos[1]
        msg.pos.z = wp_average.pos[2]
        msg.hdg = wp_average.hdg
    publisher_wp_average.publish(msg)

    rospy.loginfo("visual gate done")

    return


def calculate_visual_wp():
    global wp_visual
    global wp_average
    global wp_visual_old
    global wp_look
    global wp_blind

    # backup old WP if it not None
    wp_visual_old = wp_visual or wp_visual_old

    # normally just forward average WP to visual WP
    # but state 72: hover is required in front of gate based on measurement
    if state_auto == 72:
        rospy.loginfo("state 72, calculate hover position")

        # hover 2.8m behind dynamic gate at -0.5 height
        gate_pos = wp_average.pos
        gate_heading = wp_average.hdg
        hover_distance = -3.2
        hover_alt = -0.5
        extra_dist = np.array(
            [hover_distance * math.cos(gate_heading), hover_distance * math.sin(gate_heading), hover_alt])
        # fly to this mock location, look at the gate
        wp_visual = cr.WP(gate_pos + extra_dist, gate_heading)
        wp_look = wp_average

    # state 63: exiting jungle. 2nd jungle gate is shifted by an additional meter backwards in gate direction
    # so that drone fries faster to it without ever reaching it
    elif state_auto == 63:
        rospy.loginfo("state 63, calculate 2nd jungle position")

        # set WP 1m ahead in gate direction
        gate_pos = wp_average.pos
        gate_heading = wp_average.hdg
        hover_distance = 1.0
        extra_dist = np.array(
            [hover_distance * math.cos(gate_heading), hover_distance * math.sin(gate_heading), 0])
        # fly to this mock location and also look at this mock location
        wp_visual = cr.WP(gate_pos + extra_dist, gate_heading)
        wp_look = wp_visual

    else:
        # just forward ths WP
        wp_visual = wp_average

    rospy.loginfo("wp_visual")
    rospy.loginfo(wp_visual)

    # for ground control station
    msg = WP_Msg()
    if wp_visual is not None:
        msg.pos.x = wp_visual.pos[0]
        msg.pos.y = wp_visual.pos[1]
        msg.pos.z = wp_visual.pos[2]
        msg.hdg = wp_average.hdg
    publisher_wp_visual.publish(msg)
    return


def calculate_relative_wp(start, vector, heading):
    # creates a WP based on a start point, a translation vector and heading info of x axis
    # rotation matrix based of x axis heading
    rot_mat = np.array(
        [[math.cos(heading), -math.sin(heading), 0], [math.sin(heading), math.cos(heading), 0], [0, 0, 1]])
    # vector is rotated
    vector_global = np.matmul(rot_mat, vector)
    # add start point to vector
    return cr.WP(start + vector_global, 0)


def calculate_blind_waypoint(fly, look):
    global wp_blind
    global wp_look
    global wp_blind_old
    # make backup of old WP if it is not None
    wp_blind_old = wp_blind or wp_blind_old

    # scale all distances if a scale is established
    if wp_scale is not None:
        fly = wp_scale * fly
        look = wp_scale * look

    # normally base blind WP calculation on last gate. For first gate that was not available though
    if wp_visual_old is None:
        # waypoint calculation based on own position and pose
        rospy.loginfo("wp_visual_old not avail")

        # this could probably be done somehow with the function "calculate_relative_wp"
        # basic idea: use bebop positions to calculate WP (required for the first two states, so they are both based on
        # current position)
        bebop_position = bebop_odometry.pose.pose.position
        bebop_orientation = bebop_odometry.pose.pose.orientation
        bebop_p = [bebop_position.x, bebop_position.y, bebop_position.z]
        bebop_q = [bebop_orientation.x, bebop_orientation.y, bebop_orientation.z, bebop_orientation.w]

        # rotate translation vector from bebop frame into global frame and add own position
        blind_position = fly  # front, left , up
        blind_position_global = cr.qv_mult(bebop_q, blind_position) + bebop_p
        blind_position_global = blind_position_global.tolist()

        # same with look
        look_position = look  # front, left, up
        look_position_global = cr.qv_mult(bebop_q, look_position) + bebop_p
        look_position_global = look_position_global.tolist()

        wp_blind = cr.WP(blind_position_global, 0)
        wp_look = cr.WP(look_position_global, 0)

    else:
        # waypoint calculation based on last gate
        rospy.loginfo("wp_visual_old avail")

        # see above
        fly_start = wp_visual_old.pos
        fly_vector = fly
        fly_x_heading = wp_visual_old.hdg  # heading of "forward" to calculate blind WP (old visual's hdg)

        look_start = wp_visual_old.pos
        look_vector = look
        look_x_heading = wp_visual_old.hdg

        wp_blind = calculate_relative_wp(fly_start, fly_vector, fly_x_heading)
        wp_look = calculate_relative_wp(look_start, look_vector, look_x_heading)

    rospy.loginfo("state " + str(state_auto) + ": blind wp set")
    rospy.loginfo("wp_blind")
    rospy.loginfo(wp_blind)


def select_waypoint():

    # publish blind WP for ground control station
    msg = WP_Msg()
    if wp_blind is not None:
        msg.pos.x = wp_blind.pos[0]
        msg.pos.y = wp_blind.pos[1]
        msg.pos.z = wp_blind.pos[2]
        msg.hdg = wp_blind.hdg
    publisher_wp_blind.publish(msg)

    # publish look WP for ground control station
    msg = WP_Msg()
    if wp_look is not None:
        msg.pos.x = wp_look.pos[0]
        msg.pos.y = wp_look.pos[1]
    publisher_wp_look.publish(msg)

    # select visual if it is available, otherwise select blind
    global wp_select
    if wp_visual is not None:
        rospy.loginfo("fly visual")
        wp_select = wp_visual
    elif wp_blind is not None:
        rospy.loginfo("fly blind")
        wp_select = wp_blind
    else:
        rospy.loginfo("no wp")
        wp_select = None


def navigate_through():
    # navigation algorithm to carry us onto the centerline and up to the gate, all navigation written by Derek
    bebop_position = bebop_odometry.pose.pose.position
    bebop_orientation = bebop_odometry.pose.pose.orientation

    bebop_p = [bebop_position.x, bebop_position.y, bebop_position.z]
    bebop_q = [bebop_orientation.x, bebop_orientation.y, bebop_orientation.z, bebop_orientation.w]
    bebop_x_vec = cr.qv_mult(bebop_q, [1, 0, 0])
    hdg = math.atan2(bebop_x_vec[1], bebop_x_vec[0])

    rospy.loginfo("fly from")
    rospy.loginfo([bebop_p[0], bebop_p[1], bebop_p[2], hdg])
    rospy.loginfo("fly to")
    rospy.loginfo(wp_select)

    # heading angle
    angle = tfs.euler_from_quaternion(bebop_q)[2]

    # bebop velocity
    velocity = bebop_odometry.twist.twist.linear

    # global difference between WP ad position
    diff_global = wp_select.pos - bebop_p
    # X and Y components hypothenuse
    dist = math.hypot(diff_global[0], diff_global[1])

    # heading of the gate itself
    gate_theta = wp_select.hdg
    # heading from drone to gate position
    pos_theta = math.atan2(diff_global[1], diff_global[0])

    # difference between the two headings
    d_theta = gate_theta - pos_theta
    if d_theta > math.pi:
        d_theta = -2 * math.pi + d_theta
    elif d_theta < -math.pi:
        d_theta = 2 * math.pi + d_theta
    else:
        pass

    # lateral deviation from centerline
    y_pos_error = -dist * math.sin(d_theta)
    y_vel_des = nav_through_PID_y_pos.update(y_pos_error)

    # axial deviation to gate
    x_pos_error = cr.min_value(dist * math.cos(d_theta), 0.1)
    if dist > 2:
        # if further than 2m: be within 30 degrees to get x velocity desired
        x_vel_des = x_pos_error*max(cr.limit_value(1-6*abs(d_theta)/math.pi, 1.0), 0)
    else:
        # if closer than 2m: be within 13 degrees to get x velocity desired
        x_vel_des = x_pos_error*max(cr.limit_value(1-14*abs(d_theta)/math.pi, 1.0), -.25)

    # height difference
    z_error = diff_global[2]

    # yaw difference
    r_error = -(angle - pos_theta)
    if r_error > math.pi:
        r_error = -2 * math.pi + r_error
    elif r_error < -math.pi:
        r_error = 2 * math.pi + r_error

    # special algorithm that limits lateral velocity. up to 0.1 use proportional. everything above: squeeze to a third
    y_vel_des_sum = sum(y_vel_des)
    if y_vel_des_sum > 0.1:
        y_vel_des_sum = (y_vel_des_sum - 0.1)/3 + 0.1
    elif y_vel_des_sum < -0.1:
        y_vel_des_sum = (y_vel_des_sum + 0.1)/3 - 0.1

    # velocity error
    y_vel_error = y_vel_des_sum - velocity.y
    # don't allow too high x velocity
    x_vel_error = cr.limit_value(x_vel_des, 0.6) - velocity.x

    # update PID loops
    nav_cmd_x = nav_through_PID_x_vel.update(x_vel_error)
    nav_cmd_y = nav_through_PID_y_vel.update(y_vel_error)
    nav_cmd_z = nav_through_PID_z_vel.update(z_error)
    nav_cmd_r = nav_through_PID_r_vel.update(r_error)

    # create message. limit with global limits, add a part to x to always move forward a bit
    msg = Auto_Driving_Msg()
    msg.x = cr.limit_value(sum(nav_cmd_x) + 0.04, nav_limit_x_thr)
    msg.y = cr.limit_value(sum(nav_cmd_y), nav_limit_y_thr)
    msg.z = cr.limit_value(sum(nav_cmd_z), nav_limit_z)
    msg.r = cr.limit_value(sum(nav_cmd_r), nav_limit_r)

    log_string = str(
         dist) + ", " + str(
        0)+', ' + str(
        0)+', ' + str(
        0)+', ' + str(
        x_vel_des) + ", " + str(
        velocity.x) + ", " + str(
        x_vel_error) + ", " + str(
        nav_cmd_x[0]) + ", " + str(
        nav_cmd_x[1]) + ", " + str(
        nav_cmd_x[2]) + ", " + str(
        sum(nav_cmd_x)) + ", " + str(
        msg.x) + ", " + str(
        y_pos_error) + ", " + str(
        y_vel_des[0]) + ", " + str(
        y_vel_des[1]) + ", " + str(
        y_vel_des[2]) + ", " + str(
        sum(y_vel_des)) + ", " + str(
        velocity.y) + ", " + str(
        y_vel_error) + ", " + str(
        nav_cmd_y[0]) + ", " + str(
        nav_cmd_y[1]) + ", " + str(
        nav_cmd_y[2]) + ", " + str(
        sum(nav_cmd_y)) + ", " + str(
        msg.y) + ", " + str(
        diff_global[2]) + ", " + str(
        z_error) + ", " + str(
        nav_cmd_z[0]) + ", " + str(
        nav_cmd_z[1]) + ", " + str(
        nav_cmd_z[2]) + ", " + str(
        sum(nav_cmd_z)) + ", " + str(
        msg.z) + ", " + str(
        pos_theta) + ", " + str(
        angle) + ", " + str(
        r_error) + ", " + str(
        nav_cmd_r[0]) + ", " + str(
        nav_cmd_r[1]) + ", " + str(
        nav_cmd_r[2]) + ", " + str(
        sum(nav_cmd_r)) + ", " + str(
        msg.r) + ", " + str(
        time.time()-t_log) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0)

    publisher_nav_log.publish(log_string)

    # command itself is returned
    return msg


def navigate_point():
    # navigation algorithm to fly us to WPs, all navigation written by Derek
    bebop_position = bebop_odometry.pose.pose.position
    bebop_orientation = bebop_odometry.pose.pose.orientation

    bebop_p = [bebop_position.x, bebop_position.y, bebop_position.z]
    bebop_q = [bebop_orientation.x, bebop_orientation.y, bebop_orientation.z, bebop_orientation.w]
    bebop_x_vec = cr.qv_mult(bebop_q, [1, 0, 0])
    hdg = math.atan2(bebop_x_vec[1], bebop_x_vec[0])

    rospy.loginfo("fly from")
    rospy.loginfo(
        [bebop_odometry.pose.pose.position.x, bebop_odometry.pose.pose.position.y, bebop_odometry.pose.pose.position.z,
         hdg])
    rospy.loginfo("fly to")
    rospy.loginfo(wp_select)

    angle = tfs.euler_from_quaternion(bebop_q)[2]
    velocity = bebop_odometry.twist.twist.linear

    # transform velocities into global frame
    global_vel = [velocity.x * math.cos(angle) - velocity.y * math.sin(angle),
                  velocity.y * math.cos(angle) + velocity.x * math.sin(angle),
                  velocity.z]

    diff_global = wp_select.pos - bebop_p
    diff_global_look = wp_look.pos - bebop_p

    pos_theta = math.atan2(diff_global_look[1], diff_global_look[0])

    r_error = -(angle - pos_theta)
    if r_error > math.pi:
        r_error = -2 * math.pi + r_error
    elif r_error < -math.pi:
        r_error = 2 * math.pi + r_error

    x_pos_error = diff_global[0]
    y_pos_error = diff_global[1]

    x_vel_des = nav_point_PID_x_pos.update(x_pos_error)
    y_vel_des = nav_point_PID_y_pos.update(y_pos_error)

    # here is a bug that was done through copy paste. Compare to velocity limiter in navigate_through. 0.1 should be 0.5
    # but this needs to be tuned again and tested. Discovered in Madrid
    x_vel_des_sum = sum(x_vel_des)
    if x_vel_des_sum > 0.1:
        x_vel_des_sum = (x_vel_des_sum - 0.5)/3 + 0.5
    elif x_vel_des_sum < -0.1:
        x_vel_des_sum = (x_vel_des_sum + 0.5)/3 - 0.5

    # same bug as above
    y_vel_des_sum = sum(y_vel_des)
    if y_vel_des_sum > 0.1:
        y_vel_des_sum = (y_vel_des_sum - 0.5)/3 + 0.5
    elif y_vel_des_sum < -0.1:
        y_vel_des_sum = (y_vel_des_sum + 0.5)/3 - 0.5

    x_vel_error = x_vel_des_sum - global_vel[0]
    y_vel_error = y_vel_des_sum - global_vel[1]

    z_error = diff_global[2]

    nav_cmd_x = nav_point_PID_x_vel.update(x_vel_error)
    nav_cmd_y = nav_point_PID_y_vel.update(y_vel_error)
    nav_cmd_z = nav_point_PID_z_vel.update(z_error)
    nav_cmd_r = nav_point_PID_r_vel.update(r_error)

    # return navigation commands back into vehicle frame
    nav_cmd_x_veh = sum(nav_cmd_x) * math.cos(-angle) - sum(nav_cmd_y) * math.sin(-angle)
    nav_cmd_y_veh = sum(nav_cmd_y) * math.cos(-angle) + sum(nav_cmd_x) * math.sin(-angle)

    # limit commands
    msg = Auto_Driving_Msg()
    msg.x = cr.limit_value(nav_cmd_x_veh, nav_limit_x_pt)
    msg.y = cr.limit_value(nav_cmd_y_veh, nav_limit_y_pt)
    msg.z = cr.limit_value(sum(nav_cmd_z), nav_limit_z)
    msg.r = cr.limit_value(sum(nav_cmd_r), nav_limit_r)

    log_string = str(
        x_pos_error) + ", " + str(
        x_vel_des[0]) + ", " + str(
        x_vel_des[1]) + ", " + str(
        x_vel_des[2]) + ", " + str(
        sum(x_vel_des)) + ", " + str(
        global_vel[0]) + ", " + str(
        x_vel_error) + ", " + str(
        nav_cmd_x[0]) + ", " + str(
        nav_cmd_x[1]) + ", " + str(
        nav_cmd_x[2]) + ", " + str(
        sum(nav_cmd_x)) + ", " + str(
        msg.x) + ", " + str(
        y_pos_error) + ", " + str(
        y_vel_des[0]) + ", " + str(
        y_vel_des[1]) + ", " + str(
        y_vel_des[2]) + ", " + str(
        sum(y_vel_des)) + ", " + str(
        global_vel[1]) + ", " + str(
        y_vel_error) + ", " + str(
        nav_cmd_y[0]) + ", " + str(
        nav_cmd_y[1]) + ", " + str(
        nav_cmd_y[2]) + ", " + str(
        sum(nav_cmd_y)) + ", " + str(
        msg.y) + ", " + str(
        diff_global[2]) + ", " + str(
        z_error) + ", " + str(
        nav_cmd_z[0]) + ", " + str(
        nav_cmd_z[1]) + ", " + str(
        nav_cmd_z[2]) + ", " + str(
        sum(nav_cmd_z)) + ", " + str(
        msg.z) + ", " + str(
        pos_theta) + ", " + str(
        angle) + ", " + str(
        r_error) + ", " + str(
        nav_cmd_r[0]) + ", " + str(
        nav_cmd_r[1]) + ", " + str(
        nav_cmd_r[2]) + ", " + str(
        sum(nav_cmd_r)) + ", " + str(
        msg.r) + ", " + str(
        time.time()-t_log) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0)

    publisher_nav_log.publish(log_string)
    return msg


def navigate_jungle():
    # navigation algorithm to lines us up with jungle much more precisely, all navigation written by Derek
    bebop_position = bebop_odometry.pose.pose.position
    bebop_orientation = bebop_odometry.pose.pose.orientation

    bebop_p = [bebop_position.x, bebop_position.y, bebop_position.z]
    bebop_q = [bebop_orientation.x, bebop_orientation.y, bebop_orientation.z, bebop_orientation.w]
    bebop_x_vec = cr.qv_mult(bebop_q, [1, 0, 0])
    hdg = math.atan2(bebop_x_vec[1], bebop_x_vec[0])

    rospy.loginfo("fly from")
    rospy.loginfo([bebop_p[0], bebop_p[1], bebop_p[2], hdg])
    rospy.loginfo("fly to")
    rospy.loginfo(wp_select)

    # quat = bebop_odometry.pose.pose.orientation
    angle = tfs.euler_from_quaternion(bebop_q)[2]

    velocity = bebop_odometry.twist.twist.linear

    diff_global = wp_select.pos - bebop_p

    dist = math.hypot(diff_global[0], diff_global[1])

    gate_theta = wp_select.hdg
    pos_theta = math.atan2(diff_global[1], diff_global[0])

    d_theta = gate_theta - pos_theta
    if d_theta > math.pi:
        d_theta = -2 * math.pi + d_theta
    elif d_theta < -math.pi:
        d_theta = 2 * math.pi + d_theta
    else:
        pass

    y_pos_error = -dist * math.sin(d_theta)
    y_vel_des = nav_through_PID_y_pos.update(y_pos_error)

    x_pos_error = cr.min_value(dist * math.cos(d_theta), 0.1)
    # if further than 2 meters: be within 10 deg, otherwise no forward
        # if within 10 deg: be slower than .07 m/s laterally
    # if between 1.2 and 2m: be within 5 deg, otherwise backwards
        # if within 5 deg: be slower than 0.07 laterally
    # if closer that 1.2: be less than 10cm off laterally (lateral velocity negligible now)
    if dist > 2:
        x_vel_des = x_pos_error * max(cr.limit_value(1 - 18 * abs(d_theta) / math.pi, 1.0), 0)
        if abs(d_theta) < math.pi / 18:
            x_vel_des = x_vel_des * max(1 - abs(velocity.y / .07), 0)
    elif dist > 1.2:
        x_vel_des = x_pos_error * max(cr.limit_value(1 - 36 * abs(d_theta) / math.pi, 1.0), -.02)
        if abs(d_theta) < math.pi / 36 and dist > 1.2:
            x_vel_des = x_vel_des * max(1 - abs(velocity.y / .07), 0)
    else:
        x_vel_des = x_pos_error * max((.1 - abs(y_pos_error)) / .1, -.02)

    z_error = diff_global[2]

    r_error = -(angle - pos_theta)
    if r_error > math.pi:
        r_error = -2 * math.pi + r_error
    elif r_error < -math.pi:
        r_error = 2 * math.pi + r_error

    y_vel_des_sum = sum(y_vel_des)
    if y_vel_des_sum > 0.1:
        y_vel_des_sum = (y_vel_des_sum - 0.1)/3 + 0.1
    elif y_vel_des_sum < -0.1:
        y_vel_des_sum = (y_vel_des_sum + 0.1)/3 - 0.1

    y_vel_error = y_vel_des_sum - velocity.y
    x_vel_error = cr.limit_value(x_vel_des, 0.5) - velocity.x  # prevent buildup of too much axial speed

    nav_cmd_x = nav_through_PID_x_vel.update(x_vel_error)
    nav_cmd_y = nav_through_PID_y_vel.update(y_vel_error)
    nav_cmd_z = nav_through_PID_z_vel.update(z_error)
    nav_cmd_r = nav_through_PID_r_vel.update(r_error)

    msg = Auto_Driving_Msg()
    msg.x = cr.limit_value(sum(nav_cmd_x) + 0.04, nav_limit_x_thr)
    msg.y = cr.limit_value(sum(nav_cmd_y), nav_limit_y_thr)
    msg.z = cr.limit_value(sum(nav_cmd_z), nav_limit_z)
    msg.r = cr.limit_value(sum(nav_cmd_r), nav_limit_r)

    log_string = str(
         dist) + ", " + str(
        0)+', ' + str(
        0)+', ' + str(
        0)+', ' + str(
        x_vel_des) + ", " + str(
        velocity.x) + ", " + str(
        x_vel_error) + ", " + str(
        nav_cmd_x[0]) + ", " + str(
        nav_cmd_x[1]) + ", " + str(
        nav_cmd_x[2]) + ", " + str(
        sum(nav_cmd_x)) + ", " + str(
        msg.x) + ", " + str(
        y_pos_error) + ", " + str(
        y_vel_des[0]) + ", " + str(
        y_vel_des[1]) + ", " + str(
        y_vel_des[2]) + ", " + str(
        sum(y_vel_des)) + ", " + str(
        velocity.y) + ", " + str(
        y_vel_error) + ", " + str(
        nav_cmd_y[0]) + ", " + str(
        nav_cmd_y[1]) + ", " + str(
        nav_cmd_y[2]) + ", " + str(
        sum(nav_cmd_y)) + ", " + str(
        msg.y) + ", " + str(
        diff_global[2]) + ", " + str(
        z_error) + ", " + str(
        nav_cmd_z[0]) + ", " + str(
        nav_cmd_z[1]) + ", " + str(
        nav_cmd_z[2]) + ", " + str(
        sum(nav_cmd_z)) + ", " + str(
        msg.z) + ", " + str(
        pos_theta) + ", " + str(
        angle) + ", " + str(
        r_error) + ", " + str(
        nav_cmd_r[0]) + ", " + str(
        nav_cmd_r[1]) + ", " + str(
        nav_cmd_r[2]) + ", " + str(
        sum(nav_cmd_r)) + ", " + str(
        msg.r) + ", " + str(
        time.time()-t_log) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0)

    publisher_nav_log.publish(log_string)
    return msg


def navigate_jungle2():
    # navigation algorithm to exit us from inside jungle, fly only in gate direction, all navigation written by Derek
    bebop_position = bebop_odometry.pose.pose.position
    bebop_orientation = bebop_odometry.pose.pose.orientation

    bebop_p = [bebop_position.x, bebop_position.y, bebop_position.z]
    bebop_q = [bebop_orientation.x, bebop_orientation.y, bebop_orientation.z, bebop_orientation.w]
    bebop_x_vec = cr.qv_mult(bebop_q, [1, 0, 0])
    hdg = math.atan2(bebop_x_vec[1], bebop_x_vec[0])

    rospy.loginfo("fly from")
    rospy.loginfo([bebop_p[0], bebop_p[1], bebop_p[2], hdg])
    rospy.loginfo("fly to")
    rospy.loginfo(wp_select)

    diff_global_look = wp_look.pos - bebop_p
    pos_theta = math.atan2(diff_global_look[1], diff_global_look[0])
    angle = tfs.euler_from_quaternion(bebop_q)[2]

    r_error = -(angle - pos_theta)
    if r_error > math.pi:
        r_error = -2 * math.pi + r_error
    elif r_error < -math.pi:
        r_error = 2 * math.pi + r_error

    diff_global = wp_select.pos - bebop_p
    dist = math.hypot(diff_global[0], diff_global[1])

    # sum of command effort (hypoth)
    commanding = .25

    # "virtual" gate far away, so x error usually high. y error much smaller -> x3
    # transformation from error into vehicle frame
    x_pos_error = diff_global[0] * math.cos(-angle) - diff_global[1] * math.sin(-angle)
    y_pos_error = 3 * (diff_global[1] * math.cos(-angle) + diff_global[0] * math.sin(-angle))

    # create command from error normalized by distance from gate
    nav_cmd_x = (x_pos_error / math.hypot(x_pos_error, y_pos_error)) * commanding
    nav_cmd_y = (y_pos_error / math.hypot(x_pos_error, y_pos_error)) * commanding

    nav_cmd_r = cr.limit_value(r_error, nav_limit_r)

    msg = Auto_Driving_Msg()
    msg.x = nav_cmd_x
    msg.y = nav_cmd_y
    msg.z = 0
    msg.r = nav_cmd_r

    log_string = str(
         dist) + ", " + str(
        diff_global[0])+', ' + str(
        diff_global[1])+', ' + str(
        diff_global[2])+', ' + str(
        msg.x) + ", " + str(
        msg.y) + ", " + str(
        x_pos_error) + ", " + str(
        y_pos_error) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        time.time()-t_log) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0)

    publisher_nav_log.publish(log_string)

    return msg


def full_throttle_executer(duration):
    # prepare brake and thrust messages for faster execution
    msg_brake = Auto_Driving_Msg()
    msg_thrust = Auto_Driving_Msg()
    msg_thrust.x = 1.0

    # prepare end time
    time_end = time.time() + duration
    # send full thrust commands until end time
    while time.time() < time_end:
        rospy.loginfo("DYN - full thrust command")
        publisher_auto_drive.publish(msg_thrust)
        time.sleep(0.01)

    # send brake commands until hovering again
    while not state_bebop == cr.Bebop.HOVERING:
        rospy.loginfo("DYN - brake command")
        publisher_auto_drive.publish(msg_brake)
        time.sleep(0.1)


def navigate_dynamic():
    # navigation algorithm to shoot us through the dynamic gate. This algorithm used to do more but due to the
    # dynamic, this part has to execute much faster. Therefore most parts were moved to dynamic detection callback
    global detection_dynamic_data
    bebop_position = bebop_odometry.pose.pose.position
    bebop_orientation = bebop_odometry.pose.pose.orientation

    bebop_p = [bebop_position.x, bebop_position.y, bebop_position.z]
    bebop_q = [bebop_orientation.x, bebop_orientation.y, bebop_orientation.z, bebop_orientation.w]

    bebop_x_vec = cr.qv_mult(bebop_q, [1, 0, 0])
    hdg = math.atan2(bebop_x_vec[1], bebop_x_vec[0])
    rospy.loginfo("fly from")
    rospy.loginfo([bebop_p[0], bebop_p[1], bebop_p[2], hdg])
    rospy.loginfo("fly to")
    rospy.loginfo(wp_select)

    diff_global_look = wp_visual.pos - bebop_p
    angle = tfs.euler_from_quaternion(bebop_q)[2]
    pos_theta = math.atan2(diff_global_look[1], diff_global_look[0])
    r_error = -(angle - pos_theta)
    if r_error > math.pi:
        r_error = -2 * math.pi + r_error
    elif r_error < -math.pi:
        r_error = 2 * math.pi + r_error

    # create empty message that sends hover unless it is edited
    msg = Auto_Driving_Msg()

    # yaw rotation is planned
    if detection_dynamic_data.rotate_perform:
        # test if it is still required
        if abs(r_error) < .175:
            # don't rotate
            detection_dynamic_data.rotate_perform = False
        else:
            # do very slow rotation
            msg.r = cr.limit_value(r_error, .1)
    # timer is not started yet, so start timer
    elif detection_dynamic_data.timer is None:
        detection_dynamic_data.timer = time.time()
    # timer is started. wait until 0.4 seconds have passed
    elif time.time() - detection_dynamic_data.timer > .4:
        detection_dynamic_data.timer = None
        # measure if rotation is required
        if abs(r_error) < .175:
            # no rotation, turn off this navigation and enable fast navigation in callback
            global nav_active
            nav_active = "fast"
        else:
            # plan rotation
            detection_dynamic_data.rotate_perform = True

        log_string = str(
            r_error) + ", " + str(
            0) + ", " + str(
            0) + ', ' + str(
            0) + ', ' + str(
            0) + ', ' + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            time.time() - t_log) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0)

        publisher_nav_log.publish(log_string)

    return msg


# old dynamic navigation that failed because it doesn't see all dynamic measurements as it is at a low frequency (5Hz)
# that is why it was redesigned and is in dynamic measurement callback. Also got rid of state machine that looks ugly

#
# def navigate_dynamic():
#     global detection_dynamic_data
#     bebop_position = bebop_odometry.pose.pose.position
#     bebop_orientation = bebop_odometry.pose.pose.orientation
#
#     bebop_p = [bebop_position.x, bebop_position.y, bebop_position.z]
#     bebop_q = [bebop_orientation.x, bebop_orientation.y, bebop_orientation.z, bebop_orientation.w]
#
#     bebop_x_vec = cr.qv_mult(bebop_q, [1, 0, 0])
#     hdg = math.atan2(bebop_x_vec[1], bebop_x_vec[0])
#     rospy.loginfo("fly from")
#     rospy.loginfo([bebop_p[0], bebop_p[1], bebop_p[2], hdg])
#     rospy.loginfo("fly to")
#     rospy.loginfo(wp_select)
#
#     diff_global_look = wp_visual.pos - bebop_p
#     angle = tfs.euler_from_quaternion(bebop_q)[2]
#     pos_theta = math.atan2(diff_global_look[1], diff_global_look[0])
#     r_error = -(angle - pos_theta)
#     if r_error > math.pi:
#         r_error = -2 * math.pi + r_error
#     elif r_error < -math.pi:
#         r_error = 2 * math.pi + r_error
#
#     msg = Auto_Driving_Msg()
#
#     rospy.loginfo("DYN - dynamic state")
#     rospy.loginfo(detection_dynamic_data.state)
#
#     # stabilize
#     if detection_dynamic_data.state == 0:
#         # start a timer
#         if detection_dynamic_data.timer != 0.0:
#             # when timer has elapsed, check yaw error
#             if time.time()-detection_dynamic_data.timer > .8:
#                 rospy.loginfo("DYN - check yaw error")
#                 if abs(r_error) < .08:
#                     rospy.loginfo("DYN - yaw ok, state 2")
#                     detection_dynamic_data.state = 2  # error is small -> wait for gate rotatiom
#                 else:
#                     rospy.loginfo("DYN - yaw error high, state 1")
#                     detection_dynamic_data.state = 1  # error is large -> correct yaw
#                 detection_dynamic_data.timer = 0.0
#         else:
#             detection_dynamic_data.timer = time.time()
#             rospy.loginfo("DYN - timer started")
#
#     # rotate
#     elif detection_dynamic_data.state == 1:
#         msg = Auto_Driving_Msg()
#         # check error again before sending command
#         if abs(r_error) < .08:
#             rospy.loginfo("DYN - yaw ok, state 0")
#             detection_dynamic_data.state = 0
#         else:
#             rospy.loginfo("DYN - yawing")
#             msg.r = cr.limit_value(r_error, .1)
#
#     # wait for gate rotation
#     elif detection_dynamic_data.state == 2:
#         if detection_dynamic_data.period is None:
#             rospy.loginfo("DYN - no period yet, wait")
#         else:
#             angle_difference = abs(detection_dynamic_data.theta-detection_dynamic_data.theta_trigger())
#             if angle_difference < abs(2*math.pi/(detection_dynamic_data.period*5)*.7):
#                 rospy.loginfo("DYN - pointer angle triggered, state 3")
#                 detection_dynamic_data.state = 3
#                 # during execution of full throttle other instances of the callback go into mode 3 and wait there
#                 full_throttle_executer(1.3)
#                 detection_dynamic_data.state = 4
#                 # at this point a break message will be returned
#                 rospy.loginfo("DYN - second break command")
#             else:
#                 rospy.loginfo("DYN - wait for rotation")
#
#     # wait
#     elif detection_dynamic_data.state == 3:
#         # wait here and don't return a msg
#         while detection_dynamic_data.state == 3:
#             time.sleep(0.1)
#         rospy.loginfo("DYN - stacked break command")
#
#     # brake
#     elif detection_dynamic_data.state == 4:
#         if state_bebop == 2:
#             # advance own state to advance state machine
#             rospy.loginfo("DYN - completed")
#             detection_dynamic_data.state = 5
#         else:
#             # send brake command
#             rospy.loginfo("DYN - real break command")
#
#     try:
#         trig = detection_dynamic_data.theta_trigger()
#     except:
#         trig = 0
#
#     try:
#         diff = detection_dynamic_data.theta - trig
#     except:
#         diff = 0
#
#
#     publisher_nav_log.publish(log_string)
#
#     return msg


def calculate_distance():
    # calculate a distance based on which navigation is active

    if wp_select is None or bebop_odometry is None:
        # no waypoint or no odometry
        return 999

    bebop_position = bebop_odometry.pose.pose.position
    bebop_p = [bebop_position.x, bebop_position.y, bebop_position.z]
    diff_global = wp_select.pos - bebop_p

    if nav_active == "point":
        # use linear 3D distance between two points. Add penalty for angular offset
        linear_distance = np.linalg.norm(diff_global)

        bebop_orientation = bebop_odometry.pose.pose.orientation
        bebop_q = np.array([bebop_orientation.x, bebop_orientation.y, bebop_orientation.z, bebop_orientation.w])
        own_heading = tfs.euler_from_quaternion(bebop_q)[2]
        diff_global_look = wp_look.pos - bebop_p
        pos_theta = math.atan2(diff_global_look[1], diff_global_look[0])
        angular_diff = abs(own_heading - pos_theta)
        angular_diff = min(angular_diff, 2*math.pi-angular_diff)
        return linear_distance + angular_diff * (180.0 / 15 * 0.3) / math.pi  # 15deg offset equal 30cm

    elif nav_active == "through" or nav_active == "jungle" or nav_active == "jungle2":
        # in any through navigation, use a plane instead that is parallel to gate itself. Eventually it doesn't matter
        # how far we are from the gate in 3D but how far we are from the plane of the gate
        flat_distance = np.linalg.norm([diff_global[0], diff_global[1], 0])
        heading_to_gate = math.atan2(wp_select.pos[1] - bebop_position.y, wp_select.pos[0] - bebop_position.x)
        heading_of_gate = wp_select.hdg
        heading_difference = heading_to_gate - heading_of_gate
        return flat_distance * math.cos(heading_difference)


def wp_move(wp, dist, hdg):
    # move a waypoint by a certain distance into a specific heading
    vec = dist * np.array([math.cos(hdg), math.sin(hdg), 0])
    return cr.WP(wp.pos + vec, wp.hdg)


class State:
    # This is the ominous state machine
    def __init__(self, own_state=None, next_state=None, condition_type=None, condition_thres=None,
                 exit_clear_visual=None, detection_active_bool=None, special_detection=None, nav_active_str=None,
                 gate_size=None, gate_color=None, fly=None, look=None):
        self.own_state = own_state                              # own state id
        self.next_state = next_state                            # next state id
        self.condition_type = condition_type                    # type of state advancement
        self.condition_thres = condition_thres                  # value for state advancement (not req. for all)
        self.exit_clear_visual = bool(exit_clear_visual)        # on state exit, clear all visual data? (new gate)
        self.detection_active = bool(detection_active_bool)     # detection active?
        self.dynamic_on = (special_detection == "dynamic")      # use special dynamic gate detection?
        self.jungle_on = (special_detection == "jungle")        # use special jungle gate detection?
        self.nav_active = nav_active_str                        # which navigation should be active
        self.gate_size = gate_size                              # size of the gate
        self.gate_color = np.array(gate_color)                  # color of the gate
        self.fly = np.array(fly)                                # if blind waypoint should be calculated: offset for
        self.look = np.array(look)                              #   flying and looking waypoint into gate direction
        self.time = None                                        # timer for takeoff wait

    def enter(self):
        # do things when state is selected
        print("enter state " + str(self.own_state))
        # change global state id and also publish state id
        global current_state
        current_state = states[self.own_state]
        publisher_state_auto.publish(self.own_state)

        # reset all PID loops
        nav_point_PID_x_pos.reset()
        nav_point_PID_y_pos.reset()
        nav_point_PID_x_vel.reset()
        nav_point_PID_y_vel.reset()
        nav_point_PID_z_vel.reset()
        nav_point_PID_r_vel.reset()
        nav_through_PID_y_pos.reset()
        nav_through_PID_x_vel.reset()
        nav_through_PID_y_vel.reset()
        nav_through_PID_z_vel.reset()
        nav_through_PID_r_vel.reset()

        # if timer is required for condition, start timer
        if self.condition_type == "time":
            self.time = time.time()

        # if gate size is defined, publish gate size
        if self.gate_size is not None:
            publisher_gate_size.publish(self.gate_size)

        # if gate color is defined, publish gate color
        if self.gate_color.any():
            msg = Float32MultiArray()
            msg.data = self.gate_color
            publisher_gate_color.publish(msg)

        # set applicable navigation
        global nav_active
        nav_active = self.nav_active

        # set detection to on/off
        global detection_active
        detection_active = self.detection_active

        # publish if special detection is required
        publisher_dynamic_detection_on.publish(self.dynamic_on)
        publisher_jungle_detection_on.publish(self.jungle_on)

        # if special jungle detection is on, move all waypoints back 0.7m so that new measurements will only be taken
        # from second part of jungle gym gate (independent of the fact that the gate will be moved another meter for
        # a proper jungle2 navigation)
        if self.jungle_on:
            global wp_input_history
            for idx, wp in enumerate(wp_input_history):
                wp_input_history[idx] = wp_move(wp, 0.7, wp_average.hdg)
            global wp_average
            wp_average = cr.find_average(wp_input_history)

        # calculate blind waypoints
        if self.fly.any():
            rospy.loginfo("state " + str(self.own_state) + ": set blind waypoint")
            calculate_blind_waypoint(self.fly, self.look)

    def exit(self):
        # do things when state is finished
        print("exit state " + str(self.own_state))

        # if visuals should be cleared, do it now (preparation for next gate)
        if self.exit_clear_visual:
            global wp_average
            global wp_input_history
            wp_average = None
            wp_input_history = []

        # enter new state
        states[self.next_state].enter()

    def check(self, navigation_distance):
        # setup state advancement condition
        rospy.loginfo("state " + str(state_auto))

        # distance: if distance is smaller than value (could be scaled if there is a scale)
        if self.condition_type == "dist":
            if navigation_distance < self.condition_thres * (wp_scale or 1.0):
                self.exit()

        # waypoint: if average waypoint is found (not None anymore). visual_wp is "locked on"
        elif self.condition_type == "wp":
            if wp_average is not None:
                self.exit()

        # specific condition of bebop state machine (hover, land, ...)
        elif self.condition_type == "bebop":
            if state_bebop == self.condition_thres:
                self.exit()

        # some time has elapsed (on takeoff). Required because magnetometer changes a lot while motors spin up and drone
        # takes off
        elif self.condition_type == "time":
            if time.time() > self.time + self.condition_thres:
                self.exit()

        # if a certain navigation type is reached (only "fast" that turns itself "off")
        elif self.condition_type == "nav":
            if nav_active == self.condition_thres:
                self.exit()


def callback_bebop_odometry_changed(data):
    # main loop, every time odometry changes
    global bebop_odometry
    bebop_odometry = data

    global auto_driving_msg
    global wp_select
    global wp_visual
    global wp_visual_old
    global wp_input_history
    global wp_blind
    global wp_blind_old
    global wp_look
    global wp_takeoff
    global wp_scale
    global nav_active

    # calculate map scale factor
    # if there is no takeoff WP, set current position to takeoff
    if wp_takeoff is None:
        bebop_position = bebop_odometry.pose.pose.position
        wp_takeoff = [bebop_position.x, bebop_position.y, bebop_position.z]
    # else: calculate distance between takeoff and gate and compare to distance on map
    if wp_scale is None and wp_visual is None and wp_visual_old is not None:
        diff = wp_visual_old.pos - wp_takeoff
        wp_scale = math.sqrt(diff[0]*diff[0] + diff[1]*diff[1]) / 2.5
        wp_scale = 1
        rospy.loginfo("wp_scale")
        rospy.loginfo(wp_scale)

    # calculate distance from WP
    navigation_distance = calculate_distance()
    rospy.loginfo("navigation distance")
    rospy.loginfo(navigation_distance)

    # state_machine_advancement (if conditions are met: distances, states, ...)
    current_state.check(navigation_distance)

    # if there is no odometry, don't do anything
    if bebop_odometry is None:
        rospy.loginfo("No position")
        rospy.loginfo("publish empty driving msg")
        publisher_auto_drive.publish(Auto_Driving_Msg())
        return

    # calculate visual wp
    rospy.loginfo("calculate visual WP")
    calculate_visual_wp()

    # calculate blind wp (now done during state entering in state machine)

    # select applicable waypoint
    select_waypoint()

    # ensure there is a waypoint, otherwise return
    if wp_select is None:
        rospy.loginfo("No waypoints")
        rospy.loginfo("publish empty driving msg")
        publisher_auto_drive.publish(Auto_Driving_Msg())
        return

    # navigate to wp_select
    if nav_active == "off":
        # navigation off, send empty message and return
        rospy.loginfo("Navigation turned off")
        rospy.loginfo("publish empty driving msg")
        publisher_auto_drive.publish(Auto_Driving_Msg())
        return
    elif nav_active == "point":
        auto_driving_msg = navigate_point()
    elif nav_active == "through":
        auto_driving_msg = navigate_through()
    elif nav_active == "dynamic":
        auto_driving_msg = navigate_dynamic()
    elif nav_active == "jungle":
        auto_driving_msg = navigate_jungle()
    elif nav_active == "jungle2":
        auto_driving_msg = navigate_jungle2()
    elif nav_active == "fast":
        # if navigation fast, don't do anything
        return

    # publish command to driver
    publisher_auto_drive.publish(auto_driving_msg)
    rospy.loginfo("publish real driving msg")
    rospy.loginfo([auto_driving_msg.x, auto_driving_msg.y, auto_driving_msg.z, auto_driving_msg.r])


def emergency_shutdown(_):
    # if this is triggered, it shuts off node for performance reasons
    rospy.loginfo("emergency shutdown")
    rospy.signal_shutdown("emergency shutdown")


if __name__ == '__main__':
    # Enable killing the script with Ctrl+C.
    signal.signal(signal.SIGINT, signal_handler)

    # initialize node
    rospy.init_node('main_navigation', anonymous=False)

    # Variables
    autonomy_active = False                                     # autonomous mode is active
    bebop_odometry = None                                       # latest odometry from bebop
    state_auto = -1                                             # initialize state machine (number of state)
    current_state = None                                        # the actual state object with method "check"
    state_bebop = None                                          # state of drone itself (own state machine)
    wp_average = None                                           # wp that is the average from last measurements
    wp_visual = None                                            # almost same as above (see calculate_visual_wp)
    wp_visual_old = None                                        # backup of last visual wp
    wp_input_history = []                                       # history of gate measurements
    wp_blind = None                                             # wp calculated from map layout
    wp_blind_old = None                                         # backup of last blind wp
    wp_look = None                                              # where we are looking while flying to blind wp
    wp_select = None                                            # which one (blind or visual) are we flying to
    wp_takeoff = None                                           # remembers where takeoff occurred
    wp_scale = 1.0                                              # map scale (1 first, is computred after passing gate 1)
    detection_active = False                                    # gate detection active boolean
    detection_dynamic_data = cr.OpenloopData()                  # data req. for flight through dynamic gate
    detection_dynamic_input_history = np.empty((0, 2))          # dyn. gate pointer measurement history
    detection_dynamic_freqs = np.array([])                      # calculated frequencies from measurement history
    nav_active = "off"                                          # activated navigation algorithm
    nav_point_PID_x_pos = cr.PID2(.5, 0.1, 1.0)                 # position controller x in point navigation
    nav_point_PID_y_pos = cr.PID2(.5, 0.1, 1.0)
    nav_point_PID_x_vel = cr.PID2(0.4, 0, 0.4)                  # velocity controller x in point navigation
    nav_point_PID_y_vel = cr.PID2(0.4, 0, 0.4)
    nav_point_PID_z_vel = cr.PID(1.0, 0, 0.0)
    nav_point_PID_r_vel = cr.PID(0.5, 0, 1.0)
    nav_through_PID_y_pos = cr.PID2(.7, 0.1, 3.0)               # position controller y in through navigation
    nav_through_PID_x_vel = cr.PID(0.3, 0, 0.0)
    nav_through_PID_y_vel = cr.PID2(0.3, 0, 0.0)
    nav_through_PID_z_vel = cr.PID(1.0, 0, 0.0)
    nav_through_PID_r_vel = cr.PID(0.8, 0, 1.0)
    nav_limit_x_pt = .1  # .25                                  # absolute limit to all cmd before being sent, point_nav
    nav_limit_y_pt = .2  # .4
    nav_limit_x_thr = .1  # .25                                 # absolute limit to all cmd before being sent, thr_nav
    nav_limit_y_thr = .2  # .4
    nav_limit_z = .5  # .75
    nav_limit_r = 1.0  # 1
    dist_gate_blind = 1.0                                       # how exact go to blind wp before advancing state
    dist_gate_close = 0.5                                       # how close to gate before advancing state (approach)
    dist_exit_gate_wp = 15.0                                    # how far away is the exit waypoint
    dist_egw = dist_exit_gate_wp                                # abbreviation for above
    dist_exit_gate_min = 0.3                                    # how far behind the gate is it considered passed
    dist_gate_dyn = 0.2                                         # how accurate hover in front of dynamic gate
    dist_exit_gate = dist_exit_gate_wp - dist_exit_gate_min     # calculated distance to exit waypoint for normal gate
    dist_exit_jungle = dist_exit_gate_wp - (dist_exit_gate_min + 0.8)  # exit distance req. for jungle gate
    auto_driving_msg = Auto_Driving_Msg()                       # empty driving message (hover command)
    t_log = 1455208000                                          # time to subtract from all logs for better matlab vis

    # Publishers
    publisher_state_auto = rospy.Publisher("/auto/state_auto",     Int32,                queue_size=1, latch=True)
    publisher_auto_drive = rospy.Publisher("/auto/auto_drive",     Auto_Driving_Msg,     queue_size=1, latch=True)
    publisher_wp_average = rospy.Publisher("/auto/wp_average",     WP_Msg,               queue_size=1, latch=True)
    publisher_wp_visual = rospy.Publisher("/auto/wp_visual",       WP_Msg,               queue_size=1, latch=True)
    publisher_wp_blind = rospy.Publisher("/auto/wp_blind",         WP_Msg,               queue_size=1, latch=True)
    publisher_wp_look = rospy.Publisher("/auto/wp_look",           WP_Msg,               queue_size=1, latch=True)
    publisher_wp_current = rospy.Publisher("/auto/wp_current",     WP_Msg,               queue_size=1, latch=True)
    publisher_nav_log = rospy.Publisher("/auto/navigation_logger", String,               queue_size=1, latch=True)
    publisher_visual_log = rospy.Publisher("/auto/visual_logger",  String,               queue_size=1, latch=True)
    publisher_dev_log = rospy.Publisher("/auto/dev_logger",        String,               queue_size=1, latch=True)
    publisher_dynamic_detection_on = rospy.Publisher("/auto/dynamic_detection_on", Bool, queue_size=1, latch=True)
    publisher_jungle_detection_on = rospy.Publisher("/auto/jungle_detection_on", Bool,   queue_size=1, latch=True)
    publisher_gate_size = rospy.Publisher("/auto/gate_size",       Float32,              queue_size=1, latch=True)
    publisher_gate_color = rospy.Publisher("/auto/gate_color",     Float32MultiArray,    queue_size=1, latch=True)

    # abbreviations for state machine setup
    d = "dynamic"
    j = "jungle"
    j2 = "jungle2"
    p = "point"
    t = "through"
    o = "off"

    # Madrid color values for gates
    o1 = [100, 140, 50, 140, 255, 255]
    o2 = [110, 135, 50, 130, 255, 180]
    o3 = [100, 100, 70, 140, 255, 255]
    o4 = [70, 100, 40, 148, 255, 255]
    o5 = [70, 105, 50, 148, 255, 255]
    o6 = [110, 135, 50, 130, 255, 180]
    o7 = [100, 130, 50, 130, 255, 255]
    o8 = [112, 105, 20, 125, 255, 180]

    # own_state, next_state, condition_type, condition_thres, exit_clear_visual, detection_active_bool,
    # special_detection, nav_active_str, gate_size, gate_color, fly, look

    states = [State()] * 100
    states[02] = State(02, 03, "bebop", cr.Bebop.TAKEOFF,  0, 0, 0, o,  None, [], [], [])
    states[03] = State(03, 04, "bebop", cr.Bebop.HOVERING, 0, 0, 0, o,  None, [], [])
    states[04] = State(04, 10, "time",  1.0,               0, 0, 0, o,  None, o1, [], [])
    states[10] = State(10, 11, "dist",  0.4,               0, 0, 0, p,  1.4,  o1, [0.2, 0, 1.3], [4.4, 0, 0])
    states[11] = State(11, 12, "wp",    None,              0, 1, 0, p,  1.4,  o1, [0.0, 0, 0.4], [3.2, 0, 0])
    states[12] = State(12, 13, "dist",  dist_gate_close,   1, 1, 0, t,  None, [], [], [])
    states[13] = State(13, 21, "dist",  dist_exit_gate+.2, 0, 0, 0, p,  None, [], [dist_egw, 0, 0], [dist_egw, 0, 0])
    states[20] = State(20, 21, "dist",  dist_gate_blind,   0, 0, 0, p,  1.4,  o2, [1.7, 0, 0], [5.0, 0, 0])
    states[21] = State(21, 22, "wp",    None,              0, 1, 0, p,  None, o2, [3.25, 0, 0], [5.0, 0, 0])
    states[22] = State(22, 23, "dist",  dist_gate_close,   1, 1, 0, t,  None, [], [], [])
    states[23] = State(23, 31, "dist",  dist_exit_gate,    0, 0, 0, p,  None, [], [dist_egw, 0, 0], [dist_egw, 0, 0])
    states[30] = State(30, 31, "dist",  dist_gate_blind,   0, 0, 0, p,  1.4,  o3, [1.5, -0.25, 0], [0.7, -2.75, 0])
    states[31] = State(31, 32, "wp",    None,              0, 1, 0, p,  None, o3, [0.9, -0.5, -0.3], [0.7, -2.75, 0])
    states[32] = State(32, 33, "dist",  dist_gate_close,   1, 1, 0, t,  None, [], [], [])
    states[33] = State(33, 41, "dist",  dist_exit_gate,    0, 0, 0, p,  None, [], [dist_egw, 0, 0], [dist_egw, 0, 0])
    states[40] = State(40, 41, "dist",  dist_gate_blind,   0, 0, 0, p,  1.4,  o4, [0.5, -0.2, 0.0], [1.4, -2.1, 0])
    states[41] = State(41, 42, "wp",    None,              0, 1, 0, p,  None, o4, [0.9, -0.0, 0], [1.2, -2.1, 0])
    states[42] = State(42, 43, "dist",  dist_gate_close,   1, 1, 0, t,  None, [], [], [])
    states[43] = State(43, 51, "dist",  dist_exit_gate,    0, 0, 0, p,  None, [], [dist_egw, 0, 0], [dist_egw, 0, 0])
    states[50] = State(50, 51, "dist",  dist_gate_blind,   0, 0, 0, p,  1.4,  o5, [1.2, -0.2, 0], [5.0, 0, 0])
    states[51] = State(51, 52, "wp",    None,              0, 1, 0, p,  None, o5, [2.3, -0.2, 0], [5.0, 0, 0])
    states[52] = State(52, 53, "dist",  dist_gate_close,   1, 1, 0, t,  None, [], [], [])
    states[53] = State(53, 60, "dist",  dist_exit_gate,    0, 0, 0, p,  None, [], [dist_egw, 0, 0], [dist_egw, 0, 0])
    states[60] = State(60, 61, "dist",  dist_gate_blind,   0, 0, 0, p,  1.0,  o6, [2.7, -1.4, -0.1], [0.2, -1.9, 0])
    states[61] = State(61, 62, "wp",    None,              0, 1, 0, p,  None, o6, [2.7, -2.4, -.6], [0.2, -2.2, 0])
    states[62] = State(62, 63, "dist",  0.3,               0, 1, 0, j,  None, [], [], [])
    states[63] = State(63, 70, "dist",  0.9,               1, 1, j, j2, None, [], [], [])
    states[70] = State(70, 71, "dist",  dist_gate_blind,   0, 0, 0, p,  2.1,  o7, [1.6, -2.0, 0.3], [1.8, 0, 0])
    states[71] = State(71, 72, "wp",    None,              0, 1, 0, p,  None, o7, [1.9, -3.0, 0.3], [1.8, 0, 0])
    states[72] = State(72, 73, "dist",  dist_gate_dyn,     0, 1, 0, p,  None, [], [], [])
    states[73] = State(73, 81, "nav",   "off",             1, 1, d, d,  None, [], [], [])
    states[80] = State(80, 81, "dist",  dist_gate_blind,   0, 0, 0, p,  1.4,  o8, [2.0, 0.0, 0.7], [2.5, 2.0, 0])
    states[81] = State(81, 82, "wp",    None,              0, 1, 0, p,  None, o8, [2.5, -0.5, 0.7], [2.5, 2.0, 0])
    states[82] = State(82, 83, "dist",  dist_gate_close,   1, 1, 0, t,  None, [], [], [])
    states[83] = State(83, 90, "dist",  dist_exit_gate,    0, 0, 0, p,  None, [], [dist_egw, 0, 0], [dist_egw, 0, 0])
    states[90] = State(90, 91, "bebop", cr.Bebop.LANDING,  0, 0, 0, o,  None, [], [], [])
    states[91] = State(91, 91, "bebop", cr.Bebop.LANDED,   0, 0, 0, o,  None, [], [], [])

    # Subscribers
    rospy.Subscriber("/auto/state_auto", Int32, callback_states_changed, "state_auto")
    rospy.Subscriber("/bebop/odom", Odometry, callback_bebop_odometry_changed)
    rospy.Subscriber("/auto/gate_detection_result", Gate_Detection_Msg, callback_visual_gate_detection_changed)
    rospy.Subscriber("/auto/gate_detection_result_dynamic", Float64MultiArray, callback_visual_gate_dynamic_changed)
    rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged,
                     callback_states_changed, "state_bebop")
    rospy.Subscriber("/auto/emergency_shutdown", Empty, emergency_shutdown)

    # initializes startup by publishing state 0
    publisher_state_auto.publish(0)

    # Wait until connection between ground and air is established
    while state_auto is None or state_auto == -1:
        # send state 0 until it changes
        publisher_state_auto.publish(0)
        rospy.loginfo("waiting None")
        time.sleep(0.5)
    while state_auto == 0:
        # do nothing while state is 0 (ground has to increase)
        rospy.loginfo("waiting 0")
        time.sleep(0.5)
    while state_auto == 1:
        # send state 2 until it changes
        publisher_state_auto.publish(state_auto + 1)
        rospy.loginfo("waiting 1")
        time.sleep(0.5)

    # send one last state 2 and then enter this state in state machine
    publisher_state_auto.publish(2)
    states[02].enter()

    rospy.loginfo("Jetson communicating")

    rospy.spin()
