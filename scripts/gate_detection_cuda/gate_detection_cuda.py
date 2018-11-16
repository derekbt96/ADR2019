#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Input by bebop camera and stereo camera and state machine and merged_odometry. Depending on state machine, use algorithms to detect gates on the image. Afterwards, position gates on a global map based on the odometry as gate orientation matrices are relative to camera orientation. Output position and orientation of next gate
# Status:   06/19:  Uses cheap stereo camera left video stream to identify gate position and orientation. Does not incorporate any odometry and publishes gate position at (1,0,0) with 3 Hz. Coordainte transformation missing. CV algorithm might need to be improved if there is a performance issue
#           07/06: Publishes gate position as seen from the camera (tvec: translation from camera, rvec: rotation from camera, and applicable bebop odometry

import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, CameraInfo
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import math
import matplotlib.pyplot as plt
from bebop_auto.msg import Gate_Detection_Msg

import signal
import sys


def signal_handler(signal, frame):
    sys.exit(0)


def find_threshold_bimodal(array):
    array = np.array(array,dtype="double")
    array_sum = np.sum(array)
    var = []
    for n in range(array.size):
        partarray_sum = np.sum(array[:n])
        if partarray_sum == 0:
            s1 = 0
            var1 = 0
        else:
            s1 = np.sum(array[:n]) / array_sum
            mean1 = np.sum(array[:n] * range(n)) / partarray_sum
            var1 = np.sum(np.square(range(n) - mean1) * array[:n] / array_sum / s1)

        partarray_sum = np.sum(array[n:])
        if partarray_sum == 0:
            s2 = 0
            var2 = 0
        else:
            s2 = np.sum(array[n:]) / array_sum
            mean2 = np.sum(array[n:] * range(n, array.size)) / partarray_sum
            var2 = np.sum(np.square(range(n, array.size) - mean2) * array[n:] / array_sum / s2)
        var.append(int(s1 * var1 + s2 * var2))
    idx = (var.index(min(var)) + len(var) - 1 - var[::-1].index(min(var)))/2

    if idx >= 90:
        angle_thres = idx - 90
    else:
        angle_thres = idx

    return angle_thres


def isect_lines(line1, line2):
    for x1, y1, x2, y2 in line1:
        for x3, y3, x4, y4 in line2:
            try:
                s = float((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / float((x4 - x3) * (y2 - y1) - (x2 - x1) * (y4 - y3))

                x = x3 + s * (x4 - x3)
                y = y3 + s * (y4 - y3)

            except ZeroDivisionError:
                return -1,-1, -1
                print "ZeroDivisionError in isect_lines"
    return x, y


def mask_image(rgb, enc):
    # convert to HSV
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image to get only orange colors
    # lower_color = np.array([40, 0, 0])       # blue
    # upper_color = np.array([180, 150, 150])  # blue
    # lower_color = np.array([6, 230, 110])  # orange 2D
    # upper_color = np.array([14, 255, 200])  # orange 2D
    lower_color1 = np.array([0, 150, 100])  # orange 3D
    upper_color1 = np.array([20, 255, 255])  # orange 3D
    lower_color2 = np.array([170, 150, 100])  # orange 3D
    upper_color2 = np.array([180, 255, 255])  # orange 3D

    mask1 = cv2.inRange(hsv, lower_color1, upper_color1)
    mask2 = cv2.inRange(hsv, lower_color2, upper_color2)
    mask = mask1+mask2

    # t0 = time.time()
    # try thinning all colored pixels from above (runs at around 5Hz)
    # thin = cv2.ximgproc.thinning(mask, mask, cv2.ximgproc.THINNING_ZHANGSUEN)

    # try a sobel edge detector (runs at around 20 Hz)
    # sobelx64f = cv2.Sobel(mask, cv2.CV_64F, 1, 0, ksize=3)
    # sobely64f = cv2.Sobel(mask, cv2.CV_64F, 0, 1, ksize=3)
    # abs_sobel64f = np.absolute(sobelx64f) + np.absolute(sobely64f)
    # sobel_8u = np.uint8(abs_sobel64f)

    # using a laplacian is very fast, around 100 Hz
    # laplacian = cv2.Laplacian(mask, cv2.CV_8U)

    # t1 = time.time()
    # total = t1 - t0
    # print(total)

    # Bitwise-AND mask and original image only for fun
    global image_pub_dev1
    global bridge
    output_im = bridge.cv2_to_imgmsg(mask, encoding="8UC1")
    image_pub_dev1.publish(output_im)

    #show = cv2.resize(debug,(1280,720))
    #cv2.imshow("show",show)
    #if cv2.waitKey(1) & 0xFF == ord('q'):
    #   exit()


    global image_pub_threshold
    # res = cv2.bitwise_and(rgb, rgb, mask=sobel_8u)
    # output_im = bridge.cv2_to_imgmsg(laplacian, encoding="8UC1")
    # image_pub_threshold.publish(output_im)

    return mask


def stereo_callback(data):
    global valid_last_orientation
    global pose_pub
    global bridge
    global latest_pose
    global result_publisher
    global rvec
    global tvec

    if latest_pose is None:
        return

    this_pose = latest_pose

    # convert image msg to matrix
    rgb = bridge.imgmsg_to_cv2(data, desired_encoding=data.encoding)

    # HSV conversion and frame detection
    mask = mask_image(rgb, data.encoding)

    # probabilistic hough transform
    minLineLength = 200
    maxLineGap = 50

    lines = cv2.HoughLinesP(mask, 5, np.pi / 180, 2000, minLineLength=minLineLength, maxLineGap=maxLineGap)

    if lines is None:
        print "no lines"
        result_publisher.publish(Gate_Detection_Msg())
        return

    # lines have been found

    # calculate angles of all lines and create a border frame in the picture of the gate location
    angles = []

    borders = [5000, 0, 5000, 0]
    for counter, line in enumerate(lines):
        for x1, y1, x2, y2 in line:
            angles.append(math.atan2(y2 - y1, x2 - x1) * 180 / np.pi)  # between -90 and 90
            cv2.line(rgb, (x1, y1), (x2, y2), (0, 0, 255), 2)
            borders[0] = min(borders[0], x1, x2)
            borders[1] = max(borders[1], x1, x2)
            borders[2] = min(borders[2], y1, y2)
            borders[3] = max(borders[3], y1, y2)

    # border = np.array([[borders[0],borders[2]],[borders[0],borders[3]],[borders[1],borders[3]],[borders[1],borders[2]]], np.int32)
    # border = border.reshape((-1, 1, 2))
    # cv2.polylines(rgb, [border], True, (100,100,100), 5)

    # intersect every line with every other line
    corners_long = []
    for i1, l1 in enumerate(lines):
        for i2, l2 in enumerate(lines[:i1]):
            angles_diff = math.fabs(angles[i1] - angles[i2])
            if 10 < angles_diff < 170:  # only intersect if they are not clos to parallel
                x, y = isect_lines(l1, l2)
                # only use those intersections that lie within bounding box of gate
                if (borders[0] - 0.2 * (borders[1] - borders[0]) < x < borders[1] + 0.2 * (
                        borders[1] - borders[0]) and
                        borders[2] - 0.2 * (borders[3] - borders[2]) < y < borders[3] + 0.2 * (
                                borders[3] - borders[2])):
                    corners_long.append([x, y])
                    # cv2.circle(rgb, (int(x), int(y)), 1, (255, 255, 255), -1)

    if len(corners_long) == 0:  # no corners have been found
        print "no corners"
        result_publisher.publish(Gate_Detection_Msg())
        return

    # corners were found, find average and center
    # while there are still corners to sort, use the first in the list and look for all corners in vicinity
    corners = []
    while len(corners_long) > 0:
        xm, ym = corners_long.pop(0)
        votes = 1
        i = 0
        while i < len(corners_long):  # go through whole list once
            x1, y1 = corners_long[i]
            dist = math.sqrt((xm - x1) * (xm - x1) + (ym - y1) * (ym - y1))  # calculate distance of each point
            if dist < 60:  # if distance is small enough, recalculate point center and add one vote, then delete from list
                xm = (xm * votes + corners_long[i][0]) / (votes + 1)
                ym = (ym * votes + corners_long[i][1]) / (votes + 1)
                votes = votes + 1
                del corners_long[i]
            else:  # otherwise continue with next item
                i = i + 1
        corners.append([xm, ym, votes])

    for x, y, v in corners:
        cv2.circle(rgb, (int(x), int(y)), 10, (0, 0, 255), -1)

    # delete the corners with the least number of votes
    while len(corners) > 4:
        votes = zip(*corners)[2]
        del corners[votes.index(min(votes))]

    for x, y, v in corners:
        cv2.circle(rgb, (int(x), int(y)), 10, (255, 0, 0), -1)

    # Assume no lens distortion
    dist_coeffs = np.zeros((4, 1))

    square_side = 1.03
    square_side = .12
    if len(corners) < 3:
        print "Found only two points or less"
        valid_last_orientation = False
        result_publisher.publish(Gate_Detection_Msg())
        return
    elif len(corners) == 3 and not valid_last_orientation:
        print "3 points without a guess"
        result_publisher.publish(Gate_Detection_Msg())
        return

    if len(corners) == 3:
        corner_points = np.array([[corners[0][0], corners[0][1]], [corners[1][0], corners[1][1]],
                                  [corners[2][0], corners[2][1]]], dtype="double")
        # 3D model points.
        model_points = np.array([
            (+square_side / 2, +square_side / 2, 0.0),
            (+square_side / 2, -square_side / 2, 0.0),
            (-square_side / 2, +square_side / 2, 0.0)])

        (success, rvec, tvec) = cv2.solvePnP(model_points, corner_points,
                                             camera_matrix,
                                             dist_coeffs, rvec, tvec, True,
                                             flags=cv2.SOLVEPNP_ITERATIVE)
        valid_last_orientation = True
        print success

    elif len(corners) == 4:
        corner_points = np.array([[corners[0][0], corners[0][1]], [corners[1][0], corners[1][1]],
                                  [corners[2][0], corners[2][1]], [corners[3][0], corners[3][1]]],
                                 dtype="double")
        # 3D model points.
        model_points = np.array([
            (+square_side / 2, +square_side / 2, 0.0),
            (+square_side / 2, -square_side / 2, 0.0),
            (-square_side / 2, +square_side / 2, 0.0),
            (-square_side / 2, -square_side / 2, 0.0)])
        (success, rvec, tvec) = cv2.solvePnP(model_points, corner_points, camera_matrix,
                                             dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
        valid_last_orientation = True

    # print "Rotation Vector:\n {0}".format(rvec)
    # print "Translation Vector:\n {0}".format(tvec)

    # publish results
    msg = Gate_Detection_Msg()
    msg.tvec = tvec
    msg.rvec = rvec
    msg.bebop_pose = this_pose
    result_publisher.publish(msg)
    print("detected")

    # draw a line sticking out of the plane
    (center_point_2D_base, _) = cv2.projectPoints(np.array([(.0, .0, 0)]), rvec, tvec, camera_matrix, dist_coeffs)
    (center_point_2D_back, _) = cv2.projectPoints(np.array([(.0, .0, square_side)]), rvec, tvec, camera_matrix,
                                                  dist_coeffs)
    (center_point_2D_frnt, _) = cv2.projectPoints(np.array([(.0, .0, -square_side)]), rvec, tvec, camera_matrix,
                                                  dist_coeffs)

    p1 = (int(center_point_2D_back[0][0][0]), int(center_point_2D_back[0][0][1]))
    p2 = (int(center_point_2D_frnt[0][0][0]), int(center_point_2D_frnt[0][0][1]))
    p3 = (int(center_point_2D_base[0][0][0]), int(center_point_2D_base[0][0][1]))

    if max(p1) < 10000 and max(p2) < 10000 and min(p1) > 0 and min(p2) > 0:
        cv2.line(rgb, p1, p3, (0, 255, 255), 10)
        cv2.line(rgb, p2, p3, (0, 255, 255), 10)
    if max(p3) < 10000 and min(p3) > 0:
        cv2.circle(rgb, p3, 10, (0, 0, 0), -1)

    # Display the resulting frame
    # cv2.imshow('frame', rgb)

    global image_pub_gate
    output_im = bridge.cv2_to_imgmsg(rgb, encoding=data.encoding)
    image_pub_gate.publish(output_im)


def pose_callback(data):
    global latest_pose
    latest_pose = data


def camera_info_update(data):
    global camera_matrix
    camera_matrix = np.resize(np.asarray(data.K),[3,3])

    # camera_matrix = np.array(
    #     [[608.91474407, 0, 318.06264861]
    #      [0, 568.01764401, 242.18421071]
    #     [0, 0, 1]], dtype="double"
    # )


def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('gate_detection', anonymous=True)

    global camera_matrix
    camera_matrix = None
    rospy.Subscriber("/zed/left/camera_info", CameraInfo, camera_info_update)

    global valid_last_orientation
    valid_last_orientation = False

    global image_pub_threshold
    global image_pub_gate
    global image_pub_dev1
    global image_pub_dev2
    global result_publisher
    global latest_pose
    latest_pose = None

    rospy.Subscriber("/zed/left/image_rect_color", Image, stereo_callback)
    rospy.Subscriber("/auto/odometry_merged", Pose, pose_callback)

    image_pub_threshold = rospy.Publisher("/auto/gate_detection_threshold", Image, queue_size=1)
    image_pub_gate = rospy.Publisher("/auto/gate_detection_gate", Image, queue_size=1)
    image_pub_dev1 = rospy.Publisher("/auto/gate_detection_dev1", Image, queue_size=1)
    image_pub_dev2 = rospy.Publisher("/auto/gate_detection_dev2", Image, queue_size=1)

    result_publisher = rospy.Publisher("/auto/gate_detection_result", Gate_Detection_Msg, queue_size=1)

    global bridge
    bridge = CvBridge()

    rospy.loginfo("running")
    rospy.spin()

if __name__ == '__main__':
    main()
