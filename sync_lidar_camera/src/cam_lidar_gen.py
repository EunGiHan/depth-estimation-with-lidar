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


# ROS modules
from std_msgs.msg import Time,String

import rospy
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
import ros_numpy
CV_BRIDGE = CvBridge()
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
img = None

def callback_img(msg):
    global img
    img = CV_BRIDGE.imgmsg_to_cv2(msg, 'bgr8')

def callback_fun(msg):
    #get the lidar topic header timestamp
    lidt=str(float(msg.header.stamp.secs)+(float(msg.header.stamp.nsecs)/(10**9)))
    
    #wait for the sync signal to be published
    virtualtime=rospy.wait_for_message('time_sync_signal',Time)

    #get the timestamp published by the sync signal
    timenow=float(virtualtime.data.secs)+float(virtualtime.data.nsecs)/(10**9)

    #compute the offset between the sync event generated clock and the lidar topic header timestamp
    off=float(timenow)-float(lidt)

    #add the offset to the lidar topic timestamp
    lidtnew=float(lidt)+float(off)

    #save the lidar pointcloud using numpy(np) with timestamp as the name
    points = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
    nametosave=str(lidtnew).ljust(13,'0')
    np.save('/home/user/offset/imgs/'+str(nametosave),points)#save the lidar pointcloud with timestamp as the name
    cv2.imwrite('/home/user/offset/imgs/'+str(nametosave)+'.png',img)   #save the image with timestamp as the name 


if __name__ == '__main__':
    rospy.init_node('calibrate_camera_lidar', anonymous=True)
    sub=rospy.Subscriber('/rslidar_points',PointCloud2,callback_fun)#subscribe to lidar topic
    sub2=rospy.Subscriber('/pylon_camera_node/image_raw',Image,callback_img) #subscribe to image topic
    rospy.spin()
