# -*- coding: utf-8 -*-

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
    points = np.array(
        data.points
    )  # data.points type: 'open3d.cpu.pybind.utility.Vector3dVector' / shape: (# of points, 3)
    # points[:, [0, 2]] = points[:, [2, 0]] # swap columns XYZ -> ZYX / data.points[0] type: numpy.ndarray
    # TODO 축 안 바꿔도 될 것 같은데?
    return points


def in_image(point, size: dict) -> bool:
    """args 바꾸기"""
    row = np.bitwise_and(0 <= point[0], point[0] <= size["height"])
    col = np.bitwise_and(0 <= point[1], point[1] <= size["width"])
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

    Todo:
        * 매번 캘리브레이션 정보를 넘기는 것은 비효율적으로 보임!
        * PCD 시각화해보고, 카메라 영역 외 필터링이 맞게 되고 있는지 확인해보기
    """
    # projections = [project_point(dataset, p, cam_calib, lidar_calib) for p in point_cloud] # 이미지 크기 벗어나는 것 잘라내기 위해 아래처럼 반복문으로 만듦
    projections = []
    for p in point_cloud:
        projected_point = project_point(dataset, p, cam_calib, lidar_calib)
        if in_image(projected_point, cam_calib["size"]):
            projections.append(projected_point)  # 이미지 크기를 벗어난 것을 걸러냄

    print("# of projected lidar points: " + str(len(projections)) + " / " + str(len(point_cloud)))
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
        ego = np.matmul(ego, lidar)  # [R|t] X [x y z 1]^T -> [x(forward) y(left) z(up)] (3,)

        # ego coordinate -> camera coordinate
        ego = np.append(ego, [1], axis=0)  # (3,) -> (4,) / [x(forward) y(left) z(up) 1]
        P = np.array(cam_calib["P"])  # (3, 4) / intrinsic
        cam_ = np.concatenate([cam_calib["R"], cam_calib["t"]], axis=1)  # [R|t] matrix (3, 4)
        cam_ = np.concatenate([cam_, [[0, 0, 0, 1]]], axis=0)  # (3, 4) -> (4, 4)
        cam = np.matmul(P, cam_)  # (3, 4) X (4, 4) -> (3, 4)
        cam = np.matmul(cam, ego)  # (3, 4) X (4, 1) -> (3, 1) / [sx, sy, s]

        # check coordinate systems
        # print("lidar coordinate:\t", lidar_point, "\t", lidar_point.shape)
        # print("ego coordinate:\t", ego, "\t", ego.shape)
        # print("cam coordinate:\t", cam, "\t", cam.shape)

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
    depth = cam[-1]  # TODO 단위가 뭐야?????
    cam[:-1] = cam[:-1] / depth  # [x, y], float
    cam[:-1] = np.array(list(map(int, cam[:-1])))  # [x, y], int, image pixel position
    # TODO 나중에 어차피 depth 합칠 거면 변환 여기서 의미가 없음.
    # 소숫점 버린 형태로 주긴 하지만 나중에 접근 시에는 int로 다시 변환을 하긴 해야 함

    # check transformation result
    # print("depth:\t", str(depth))
    # print("x_im:\t", str(cam[0]), "\ty_im:\t", str(cam[1]))

    return cam
