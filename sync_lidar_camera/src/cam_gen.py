#!/usr/bin/env python
# -*- coding: utf-8 -*-


# Python 2/3 compatibility
from __future__ import print_function

# Built-in modules
import os
import sys
import time

# External modules
import cv2
import numpy as np
import ros_numpy
import rospy
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError

# ROS modules
from std_msgs.msg import String, Time

CV_BRIDGE = CvBridge()
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


def callback_fun(msg):
    # get the image topic header timestamp
    imgt = str(float(msg.header.stamp.secs) + (float(msg.header.stamp.nsecs) / (10**9)))

    # wait for the sync signal to be published
    virtualtime = rospy.wait_for_message("time_sync_signal", Time)

    # get the timestamp published by the sync signal
    timenow = float(virtualtime.data.secs) + float(virtualtime.data.nsecs) / (10**9)

    # compute the offset between the sync event generated clock and the image topic header timestamp
    off = float(timenow) - float(imgt)

    # add the offset to the image topic timestamp
    imgtnew = float(imgt) + float(off)

    # save the image using opencv with timestamp as the name
    img = CV_BRIDGE.imgmsg_to_cv2(msg, "bgr8")
    nametosave = str(imgtnew).ljust(13, "0")
    cv2.imwrite(
        "/home/user/offset/imgs/" + str(nametosave) + ".png", img
    )  # save the image with timestamp as the name


if __name__ == "__main__":
    rospy.init_node("calibrate_camera_lidar", anonymous=True)
    sub = rospy.Subscriber(
        "/pylon_camera_node/image_raw", Image, callback_fun
    )  # subscribe to image topic
    rospy.spin()
