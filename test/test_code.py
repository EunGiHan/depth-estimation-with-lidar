# -*- coding: utf-8 -*-

"""
PyTest test funtions
====================

.. _execution commands & config file:
    https://velog.io/@sangyeon217/pytest-execution

.. _mark 데커레이터 사용
    https://velog.io/@sangyeon217/pytest-mark

.. _fixture 사용하기
    https://velog.io/@sangyeon217/pytest-fixture
    https://blog.live2skull.kr/python/testing/python-pytest/#2-%ED%85%8C%EC%8A%A4%ED%8A%B8-%EA%B0%92-%EC%A0%84%EB%8B%AC-%EB%B0%8F-fixture-%ED%99%9C%EC%9A%A9
    https://jangseongwoo.github.io/test/pytest_basic/#pytest%EC%9D%98-fixture-%EC%84%A4%EB%AA%85

.. _logging 사용하기
    https://velog.io/@sangyeon217/pytest-logging


Note:

"""


import logging
import random
import sys
from datetime import datetime

import numpy as np
import pytest

from tools.transformations import *


@pytest.fixture
def time():
    return (datetime.now()).strftime("%Y_%m_%d-%H_%M_%S")


@pytest.fixture
def file_paths(time):
    paths = {}
    # data paths
    paths[
        "cam_calib_file"
    ] = "dataset/ACE/calibration.yaml" #"dataset/KITTI/2011_09_26/2011_09_26_calib/calib_cam_to_cam.txt"  # 
    paths[
        "lidar_calib_file"
    ] = "dataset/ACE/calibration.yaml" #"dataset/KITTI/2011_09_26/2011_09_26_calib/calib_velo_to_cam.txt"  
    paths["image_file"] = ""  # TODO 이미지 파일 경로
    paths["point_cloud_file"] = "outputs/ex_point_cloud.pcd"

    # set save paths (without extension)
    paths["depth_gt_save_path"] = "./outputs/depth_gt-" + time
    paths["depth_map_save_path"] = "./outputs/depth_map-" + time
    paths["eval_result_save_path"] = "./outputs/eval_result-" + time
    return paths


@pytest.fixture
def cam_calib(file_paths):
    return parse_cam_calib("ace", file_paths["cam_calib_file"])


@pytest.fixture
def lidar_calib(file_paths):
    return parse_lidar_calib("ace", file_paths["lidar_calib_file"])

logger = logging.getLogger("logger")


class TestCalib:
    # @classmethod
    # def setup_class(cls):
    #     logger.info(sys._getframe(0).f_code.co_name)
    #     pass


    # @classmethod
    # def teardown_class(cls):
    #     logger.info(sys._getframe(0).f_code.co_name)
    #     pass


    @pytest.fixture
    def point_cloud(self, file_paths):
        yield convert_pcd_to_xyz(file_paths["point_cloud_file"])


    @pytest.mark.skip(reason="already verified")
    def test_convert_pcd_to_xyz(self, point_cloud):
        logger.info(sys._getframe(0).f_code.co_name)
        assert point_cloud.shape == (121080, 3)

    @pytest.mark.skip(reason="already verified")
    def test_in_image(self, cam_calib):
        assert in_image([1000, 852], cam_calib['size'])

    @pytest.mark.skip(reason="fail if out-range point is picked")
    def test_projection(self, cam_calib, lidar_calib, point_cloud):
        logger.info(sys._getframe(0).f_code.co_name)
        random_idx = random.randint(0, 121080)
        projected_pos = project_point("ace", point_cloud[random_idx], cam_calib, lidar_calib)

        logger.info("input point(xyz)\t: " + str(point_cloud[random_idx]))
        logger.info("output point(xy)\t: " + str(projected_pos[:-1]))
        logger.info("depth: " + str(projected_pos[-1]))

        assert in_image(projected_pos, cam_calib['size']) # 범위 벗어나면 Fail

    @pytest.mark.skip(reason="already verified")
    def test_project_all_points(self, cam_calib, lidar_calib, point_cloud):
        projections = project_lidar_to_cam("ace", cam_calib, lidar_calib, point_cloud)

        rows = np.bitwise_and(0<=projections[:,0], projections[:,0] <= cam_calib['size']['height']).sum() == len(projections)
        cols = np.bitwise_and(0<=projections[:,1], projections[:,1] <= cam_calib['size']['width']).sum() == len(projections)
        assert np.bitwise_and(rows, cols)
