# -*- coding: utf-8 -*-

import numpy as np

from tools.parsers import *


def convert_pcd_to_xyz(point_cloud_file: str):
    """read pcd file and convert lidar data to XYZ format

    Args:
        point_cloud_file (str): lidar raw data file path (*.pcd)

    Returns:
        numpy.ndarray or list: format converted lidar data
    """
    point_cloud = []
    # TODO 만들어주세요
    return point_cloud


def project_lidar_to_cam(cam_calib: dict, lidar_calib: dict, point_cloud_file: str):
    """get all lidar [X, Y, Z] data and project them to image plane

    Args:
        cam_calib (dict): camera calibration
        lidar_calib (dict): lidar calibration
        point_cloud_file (str): lidar raw data file path (*.pcd)

    Returns:
        numpy.ndarray: projected points of lidar data into image plane
    """
    # process calibration values
    P = cam_calib["P"]

    R_cam = np.zeros((4, 4))
    R_cam[0:3, 0:3] = cam_calib["R"]
    R_cam[3, 3] = 1
    # TODO 현재 구현은 KITTI 논문을 참고 하고 있음. 그러나 이 방식대로라면 cam_calib["T"]를 쓰지 않고 있음. 다시 한 번 캘리브레이션은 확인 필요함. KITTI는 스테레오 카메라를 여러 개 사용하기 때문에 단안카메라를 쓰는 Ace와는 다를 수 있음

    t_velo_to_cam = np.array(lidar_calib["t"])
    T_velo_to_cam = np.concatenate([lidar_calib["R"], t_velo_to_cam], axis=1)
    T_velo_to_cam = np.concatenate([T_velo_to_cam, np.array([[0, 0, 0, 1]])], axis=0)

    # pcd to XYZ format
    point_cloud = convert_pcd_to_xyz(point_cloud_file)

    # XYZ to uv & depth format
    projections = []  # [[u, v, gt_depth], [u, v, gt_depth], [u, v, gt_depth], ...] for one image
    for p in point_cloud:
        projected_pos, depth = project_point(p, P, R_cam, T_velo_to_cam)
        # TODO 마찬가지로 현재 구현은 KITTI 논문을 참고 하고 있음. 그 좌표축에 맞추고 있으므로, ACE도 동일한지는 따져봐야 함. PCD 변환 결과가 [x, y, z]라 가정함.
        projected_pos = np.append(projected_pos, [depth], axis=0)  # shape: (3,)
        projections.append(projected_pos)

    return np.array(projections)


def project_point(lidar_point, P, R_cam, T_velo_to_cam):
    """get all lidar [X, Y, Z] data and project them to image plane

    Args:
        cam_calib (dict): camera calibration
        lidar_calib (dict): lidar calibration
        point_cloud_file (str): lidar raw data file path (*.pcd)

    Returns:
        numpy.ndarray: coordinate [u, v] in image plane
        float: depth value
    """
    x = np.append(np.array(lidar_point), [1], axis=0)
    x = np.transpose(x)  # (4, )

    y = np.matmul(P, R_cam)
    y = np.matmul(y, T_velo_to_cam)
    y = np.matmul(y, x)  # [sx, sy, s] 형태

    depth = y[-1]  # 깊이 정보 s / TODO 맞나..?
    y = y[:-1] / depth  # [x, y] 형태
    y = np.array(list(map(int, y)))  # [x, y] 형태 & 이미지 평면이므로 소숫점 버림

    return y, depth
