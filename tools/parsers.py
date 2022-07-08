# -*- coding: utf-8 -*-

import argparse
import sys

import numpy as np
import yaml


def parse_args():
    """parse commandline arguements

    Returns:
        argparse.Namespace: argument parsing data
    """
    parser = argparse.ArgumentParser(description="depth estimation & evaluation")
    parser.add_argument("--dataset", dest="dataset", help="kitti / ace", default="ace", type=str)
    parser.add_argument(
        "--visualize", dest="visualize", help="visualize results", default="False", type=bool
    )
    # TODO 필요한 것 추가하기

    # if no arguments
    # if len(sys.argv) == 1:
    #     parser.print_help()
    #     sys.exit()

    args = parser.parse_args()
    return args


def parse_cam_calib(dataset: str, file_path: str) -> dict:
    """pasre camera calibration file(yaml)

    Args:
        dataset (str): dataset(ace / kitti)
        file_path (str): file path to save result

    Returns:
        dict: parsed calibration values in dictionary format
    """
    with open(file_path) as f:
        cam_calib = {}

        if dataset == "ace":
            calib_info = yaml.load(f, Loader=yaml.FullLoader)["camera"]["front"]
            # cam_calib["D"] = calib_info["D"] # distortion coeefficients (5, 1)
            # cam_calib["H"] = calib_info["H"] # (3, 3)
            # cam_calib["K"] = calib_info["K"] # calibration matrices (3, 3)
            cam_calib["P"] = calib_info["P"]  # projection matrix (3, 4)
            cam_calib["R"] = calib_info["R"]  # rotation matrix (3, 3)
            cam_calib["t"] = calib_info["T"]  # translation (3, 1)
            cam_calib["size"] = calib_info["size"]  # image size {'height': 1086, 'width': 2040}
        elif dataset == "kitti":
            calib_info = yaml.load(f, Loader=yaml.FullLoader)
            cam_calib["P"] = np.reshape(
                [float(x) for x in calib_info["P_rect_00"].split(" ")], (3, 4)
            )  # projection matrix (3, 4)
            cam_calib["R"] = np.reshape(
                [float(x) for x in calib_info["R_rect_00"].split(" ")], (3, 3)
            )  # rotation matrix (3, 3)
            cam_calib["t"] = np.reshape(
                [float(x) for x in calib_info["T_00"].split(" ")], (3, 1)
            )  # translation (3, 1) # TODO ace의 T와 shape 맞나 확인
            # TODO 사이즈 추가하기!

    return cam_calib


def parse_lidar_calib(dataset: str, file_path: str) -> dict:
    """pasre lidar calibration file(yaml)

    Args:
        dataset (str): dataset(ace / kitti)
        file_path (str): file path to save result

    Returns:
        dict: parsed calibration values in dictionary format
    """
    with open(file_path) as f:
        lidar_calib = {}

        if dataset == "ace":
            calib_info = yaml.load(f, Loader=yaml.FullLoader)["lidar"]["rs80"]
            lidar_calib["R"] = calib_info["R"]
            lidar_calib["t"] = calib_info["T"]
        elif dataset == "kitti":
            calib_info = yaml.load(f, Loader=yaml.FullLoader)
            lidar_calib["R"] = np.reshape(
                [float(x) for x in calib_info["R"].split(" ")], (3, 3)
            )  # rotation matrix(lidar->cam) (3, 3) # TODO 확인!
            lidar_calib["t"] = np.reshape(
                [float(x) for x in calib_info["T"].split(" ")], (3, 1)
            )  # translation vector (3, 1)

    return lidar_calib
