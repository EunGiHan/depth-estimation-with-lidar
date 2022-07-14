# -*- coding: utf-8 -*-

import logging

import numpy as np
import open3d

from tools.parsers import *


def convert_pcd_to_xyz(point_cloud_file: str):
    """read pcd file and convert lidar data to XYZ format

    Args:
        point_cloud_file (str): lidar raw data file path (*.pcd)

    Returns:
        numpy.ndarray: format converted lidar data
    """
    data = open3d.io.read_point_cloud(point_cloud_file)
    points = np.array(data.points)

    return points


def convert_npy_to_xyz(point_cloud_file: str):
    """read pcd file and convert lidar data to XYZ format

    Args:
        point_cloud_file (str): lidar raw data file path (*.pcd)

    Returns:
        numpy.ndarray: format converted lidar data
    """
    data = np.load(point_cloud_file)
    points = []

    for data_ in data:
        for x, y, z, i in data_:
            if np.isnan(x) or np.isnan(y) or np.isnan(z):
                continue
            else:
                points.append([x, y, z])

    # if you want use pcd type? try this!
    # points = np.array(points)
    # pcd = open3d.geometry.PointCloud()
    # pcd.points = open3d.utility.Vector3dVector(points)
    # open3d.io.write_point_cloud('/home/user/depth-estimation-with-lidar/outputs/'+'temp.pcd', pcd)

    return points


def convert_npy_to_xyz_only_depth(point_cloud_file: str):
    """read pcd file and convert lidar data to XYZ format

    Args:
        point_cloud_file (str): lidar raw data file path (*.pcd)

    Returns:
        numpy.ndarray: format converted lidar data
    """
    data = np.load(point_cloud_file)
    points = []

    for x, data_ in enumerate(data):
        temp = []
        for y, z in enumerate(data_):
            temp.append(z)
        points.append(temp)

    return points


def in_image(point: np.ndarray, size: dict) -> bool:
    """check if projected point is within image size

    Args
        point (numpy.ndarray): projected point (x, y, z)
        size (dict): image size

    Returns
        bool: whether point is within image
    """
    row = np.bitwise_and(0 <= point[0], point[0] < size["width"])
    col = np.bitwise_and(0 <= point[1], point[1] < size["height"])

    return np.bitwise_and(row, col)


def project_lidar_to_cam(
    dataset: str, cam_calib: dict, lidar_calib: dict, point_cloud: np.ndarray
) -> np.ndarray:
    """get all lidar [X, Y, Z] data and project them to image plane

    Args:
        cam_calib (dict): camera calibration
        lidar_calib (dict): lidar calibration
        point_cloud (numpy.ndarray): XYZ format lidar data

    Returns:
        numpy.ndarray: projected points of lidar data into image plane (ex) [[u, v, gt_depth], [u, v, gt_depth], [u, v, gt_depth], ...]
    """
    projections = []
    for p in point_cloud:
        projected_point = project_point(dataset, p, cam_calib, lidar_calib)
        if in_image(projected_point, cam_calib["size"]):
            projections.append(projected_point)  # 이미지 크기를 벗어난 것을 걸러냄
    logging.info(
        "# of projected lidar points: " + str(len(projections)) + " / " + str(len(point_cloud))
    )
    return np.array(projections)


def project_point(dataset: str, lidar_point: np.ndarray, cam_calib: dict, lidar_calib: dict):
    """get all lidar [X, Y, Z] data and project them to image plane

    Args:
        dataset (str): dataset(ace / kitti)
        lidar_point (numpy.ndarray): a lidar point with [x, y, z] format
        cam_calib (dict): camera calibration
        lidar_calib (dict): lidar calibration

    Returns:
        numpy.ndarray: coordinate & depth in [row(x), col(y), depth] format

    Notes:
        * ACE Calib
            * lidar [R|t]: lidar coord -> ego coord (3D)
                * 센서는 더 높이 더 앞쪽으로 달려 있으므로, 보정 값은 깊이&높이는 커지고 좌우는 큰 변화 없음
            * camera P X [R|t]: ego coord -> cam coord
            * 변환 절차: lidar -> ego -> cam
        * KITTI Calib
            * cam to cam: 4개의 카메라가 있어 서로 바뀌는 calib 정보가 있음
            * velo to cam: 라이다를 카메라(0번)로 바로 투영
    """
    lidar = np.append(lidar_point, [1], axis=0)  # [x(forward) y(left) z(up) 1]
    lidar = np.transpose(lidar)  # (4, )

    if dataset == "ace":
        # lidar coordinate -> ego coordinate
        ego = np.concatenate([lidar_calib["R"], lidar_calib["t"]], axis=1)  # [R|t] matrix
        ego = np.concatenate([ego, [[0, 0, 0, 1]]], axis=0)  # (3, 4) -> (4, 4)
        ego = np.matmul(ego, lidar)  # [R|t] X [x y z 1]^T -> [x(forward) y(left) z(up)] (3,)

        # ego coordinate -> camera coordinate
        # ego = np.append(ego, [1], axis=0)  # (3,) -> (4,) / [x(forward) y(left) z(up) 1]

        P = np.array(cam_calib["P"])  # (3, 4) / intrinsic
        cam_ = np.concatenate([cam_calib["R"], cam_calib["t"]], axis=1)  # [R|t] matrix (3, 4)
        cam_ = np.concatenate([cam_, [[0, 0, 0, 1]]], axis=0)  # (3, 4) -> (4, 4)
        cam = np.matmul(P, cam_)  # (3, 4) X (4, 4) -> (3, 4)
        cam = np.matmul(cam, ego)  # (3, 4) X (4, 1) -> (3, 1) / [sx, sy, s]

        # check coordinate systems
        # logging.info("lidar coordinate:\t", lidar_point, "\t", lidar_point.shape)
        # logging.info("ego coordinate:\t", ego, "\t", ego.shape)
        # logging.info("cam coordinate:\t", cam, "\t", cam.shape)

    elif dataset == "kitti":  # TODO 검증 필요!
        P = cam_calib["P"]
        R_cam = np.concatenate([cam_calib["R"], [[0, 0, 0, 1]]], axis=0)

        t_velo_to_cam = np.array(lidar_calib["t"])
        T_velo_to_cam = np.concatenate([lidar_calib["R"], t_velo_to_cam], axis=1)
        T_velo_to_cam = np.concatenate([T_velo_to_cam, np.array([[0, 0, 0, 1]])], axis=0)

        cam = np.matmul(P, R_cam)
        cam = np.matmul(cam, T_velo_to_cam)
        cam = np.matmul(cam, lidar)  # TODO 형태 확인 필요! [sx, sy, s(=depth)]가 맞는지

    # [sx, sy, s(=depth)] -> [x, y, depth]
    depth = cam[-1]
    cam[:-1] = cam[:-1] // (depth * 2)  # [x, y], float
    cam[:-1] = np.array(list(map(int, cam[:-1])))  # [x, y], int, image pixel position

    # check transformation result
    # logging.info("depth:\t", str(depth))
    # logging.info("x_im:\t", str(cam[0]), "\ty_im:\t", str(cam[1]))

    return cam
