# -*- coding: utf-8 -*-

import logging
import sys
from datetime import datetime

import numpy as np
import pytest

from tools.transformations import *

time = (datetime.now()).strftime("%Y_%m_%d-%H_%M_%S")

# data paths
cam_calib_file = "dataset/KITTI/2011_09_26/2011_09_26_calib/calib_cam_to_cam.txt"  # "dataset/ACE/calibration.yaml"
lidar_calib_file = "dataset/KITTI/2011_09_26/2011_09_26_calib/calib_velo_to_cam.txt"  # "dataset/ACE/calibration.yaml"
image_file = ""  # TODO 이미지 파일 경로
point_cloud_file = "outputs/ex_lidar_raw.txt"  # TODO temp file (in real, *.pcd file)

# set save paths (without extension)
depth_gt_save_path = "./outputs/depth_gt-" + time
depth_map_save_path = "./outputs/depth_map-" + time
eval_result_save_path = "./outputs/eval_result-" + time

# parse calibration files -> get information
cam_calib = parse_cam_calib("kitti", cam_calib_file)
lidar_calib = parse_lidar_calib("kitti", lidar_calib_file)


def test_projection():
    logging.info(sys._getframe(0).f_code.co_name)
    depth_gt = project_lidar_to_cam(cam_calib_file, lidar_calib_file, point_cloud_file)
    logging.info(depth_gt)
    assert depth_gt.shape[1] == 3
    # TODO 실제 데이터로 한 번 더 해보기
