#!/usr/bin/env python

import numpy as np

from parsers import *


def convert_lidar_to_cam(cam_calib_file, lidar_calib_file, raw_file_path, output_file_path):
    cam_calib = parse_cam_calib(cam_calib_file)
    lidar_calib = parse_lidar_calib(lidar_calib_file)

    y = []

    # TODO 저장하는 것은 utils에 합칠까
    with open(raw_file_path) as raw_file:
        with open(output_file_path, "w") as out_file:
            lines = raw_file.readlines()
            for line in lines:
                lidar_point = line.strip().split(" ")  # TODO TXT를 한 포인트씩 읽어오는 부분 여기에 추가
                lidar_point = [float(p) for p in lidar_point]
                projected_point = transform_lidar_to_cam(lidar_point, cam_calib, lidar_calib)[
                    :-1
                ]  # [x, y]
                y.append(projected_point)
                projected_point_str = " ".join(str(s) for s in projected_point) + "\n"
                out_file.write(projected_point_str)

    return y


def transform_lidar_to_cam(lidar_point, cam_calib, lidar_calib):
    x = np.append(np.array(lidar_point), [1], axis=0)
    x = np.transpose(x)  # (4, )

    R_cam = np.zeros((4, 4))
    R_cam[0:3, 0:3] = cam_calib["R"]  # cam_calib['R']은 (3, 3)
    R_cam[3, 3] = 1

    t_velo_to_cam = np.array(lidar_calib["t"])
    T_velo_to_cam = np.concatenate([lidar_calib["R"], t_velo_to_cam], axis=1)
    T_velo_to_cam = np.concatenate([T_velo_to_cam, np.array([[0, 0, 0, 1]])], axis=0)

    y = np.matmul(cam_calib["P"], R_cam)
    y = np.matmul(y, T_velo_to_cam)
    y = np.matmul(y, x)
    y = y / y[-1]

    return y  # (3,) # TODO 값 확인할 것. T가 필요한지


convert_lidar_to_cam(
    cam_calib_file="dataset/ACE/calibration.yaml",
    lidar_calib_file="dataset/ACE/calibration.yaml",
    raw_file_path="outputs/ex_lidar_raw.txt",
    output_file_path="outputs/ex_lidar_projection.txt",
)
