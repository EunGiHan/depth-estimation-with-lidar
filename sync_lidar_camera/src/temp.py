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
import sensor_msgs.point_cloud2 as pc2
from time import time
import open3d as o3d
import ctypes
import struct

img = None
points = None
nametosave = 0

def callback_img(msg):
    global img, nametosave
    img = CV_BRIDGE.imgmsg_to_cv2(msg, 'bgr8')
    

def callback_fun(msg):
    global points, nametosave
    #save the lidar pointcloud using numpy(np) with timestamp as the name
    nametosave += 1
    points = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
    np.save('/home/user/offset/imgs/'+str(format(nametosave, '04')),points)#save the lidar pointcloud with timestamp as the name
    #cloud_points = list(pc2.read_points(msg, skip_nans=True, field_names = ("x", "y", "z")))
    #pcd = o3d.geometry.PointCloud()
    #pcd.points = o3d.utility.Vector3dVector(cloud_points)
    #o3d.io.write_point_cloud('/home/user/offset/imgs/'+str(nametosave)+'.pcd', pcd)
    #o3d.visualization.draw_geometries([pcd])
    cv2.imwrite('/home/user/offset/imgs/'+str(format(nametosave, '04'))+'.png',img)   #save the image with timestamp as the name 
    
sub=rospy.Subscriber('/rslidar_points',PointCloud2,callback_fun)#subscribe to lidar topic
sub2=rospy.Subscriber('/pylon_camera_node/image_raw',Image,callback_img) #subscribe to image topic

if __name__=='__main__':
    rospy.init_node("sync_event_publisher")
    rate=rospy.Rate(30)
    while not rospy.is_shutdown():
        #get the timestamp to be published on the sync event topic
        rate.sleep()