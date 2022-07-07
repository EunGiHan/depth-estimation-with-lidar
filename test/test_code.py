# -*- coding: utf-8 -*-

import logging
import sys
from datetime import datetime

import numpy as np
import pytest

from tools.transformations import *

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

@pytest.fixture
def time():
    return (datetime.now()).strftime("%Y_%m_%d-%H_%M_%S")

@pytest.fixture
def file_paths():
    paths = {}
    # data paths
    paths['cam_calib_file'] = "dataset/KITTI/2011_09_26/2011_09_26_calib/calib_cam_to_cam.txt"  # "dataset/ACE/calibration.yaml"
    paths['lidar_calib_file'] = "dataset/KITTI/2011_09_26/2011_09_26_calib/calib_velo_to_cam.txt"  # "dataset/ACE/calibration.yaml"
    paths['image_file'] = ""  # TODO 이미지 파일 경로
    paths['point_cloud_file'] = "outputs/ex_lidar_raw.txt"  # TODO temp file (in real, *.pcd file)

    # set save paths (without extension)
    paths['depth_gt_save_path'] = "./outputs/depth_gt-" + time
    paths['depth_map_save_path'] = "./outputs/depth_map-" + time
    paths['eval_result_save_path'] = "./outputs/eval_result-" + time
    return paths

@pytest.fixture
def cam_calib(file_paths):
    return parse_cam_calib("kitti", file_paths['cam_calib_file'])

@pytest.fixture
def lidar_calib(file_paths):
    return parse_lidar_calib("kitti", file_paths['lidar_calib_file'])

P = cam_calib["P"]

R_cam = np.zeros((4, 4))
R_cam[0:3, 0:3] = cam_calib["R"]
R_cam[3, 3] = 1
# TODO 현재 구현은 KITTI 논문을 참고 하고 있음. 그러나 이 방식대로라면 cam_calib["T"]를 쓰지 않고 있음. 다시 한 번 캘리브레이션은 확인 필요함. KITTI는 스테레오 카메라를 여러 개 사용하기 때문에 단안카메라를 쓰는 Ace와는 다를 수 있음

t_velo_to_cam = np.array(lidar_calib["t"])
T_velo_to_cam = np.concatenate([lidar_calib["R"], t_velo_to_cam], axis=1)
T_velo_to_cam = np.concatenate([T_velo_to_cam, np.array([[0, 0, 0, 1]])], axis=0)

XYZ = [[0, 0, 0], [1, 4, 2], [4, 1, 2]]

logger = logging.getLogger('parser')

@pytest.mark.skip()
def test_projection():
    logger.info(sys._getframe(0).f_code.co_name)
    projected_pos, depth = project_point([1, 1, 0], P, R_cam, T_velo_to_cam)
    assert projected_pos.shape == (2,)
    # TODO 실제 데이터로 한 번 더 해보기

@pytest.mark.skip()
@pytest.mark.parametrize('point', XYZ)
def test_make_projected_point(point):
    logger.info(sys._getframe(0).f_code.co_name)
    projected_pos, depth = project_point(point, P, R_cam, T_velo_to_cam)
    projected_pos = np.append(projected_pos, [depth], axis=0)
    # logging.info(depth_gt)
    assert projected_pos.shape == (3,)
    # TODO 실제 데이터로 한 번 더 해보기

def test_log():
    logger.info(sys._getframe(0).f_code.co_name)
    assert True
