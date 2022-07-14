# -*- coding: utf-8 -*-

import os

import cv2
import numpy as np
import open3d


def get_files_count(folder_path: str):
    """count number of files in this path
    
    Args:
        folder_path (str): path for count number of files
    """
    dirListing = os.listdir(folder_path)
    return len(dirListing)


def save_depth_txt(depth: np.ndarray, save_path: str) -> None:
    """save depth data in txt file

    Args:
        depth (numpy.ndarray): depth vectors for one frame ([x, y, depth]). It can be depth_gt(from lidar) or depth_map(from model)
        save_path (str): file path to save result
    """
    result = ""
    for d in depth:
        projected_pos = d[:-1]
        depth = d[-1]
        point = " ".join(str(int(coord)) for coord in projected_pos) + " " + str(depth) + "\n"
        result += point

    with open(save_path, "w") as f:
        f.write(result)


def save_depth_npy(depth: np.ndarray, save_path: str) -> None:
    """save depth map in NPY format

    Args:
        depth (numpy.ndarray): depth map (estimated)
        save_path (str): depth map saving path
    """
    np.save(save_path, depth)


def undistort_image(img_num: int, cam_calib: dict, raw_img_file_path: str):
    """undistort raw image

    Args:
        img_num (int): raw image number
        cam_calib (dict): camera calibration information
        raw_img_file_path (str): raw image file path
    
    Returns:
        numpy.ndarray: undistorted image (Mat)
    """
    raw_img = cv2.imread(raw_img_file_path + str(format(img_num, "04")) + ".png")
    height, width, _ = raw_img.shape

    int_param = np.array(cam_calib["K"])
    distortion = np.array(cam_calib["D"])
    int_param_scaling = np.array(cam_calib["P"]).reshape((3, 4))[:3, :3]
    rectification = np.eye(3)

    mapx, mapy = cv2.initUndistortRectifyMap(
        int_param,
        distortion,
        rectification,
        int_param_scaling,
        (width, height),
        cv2.CV_32FC1,
    )
    calibrated_img = cv2.remap(
        raw_img,
        mapx,
        mapy,
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
    )
    calibrated_img = cv2.resize(
        calibrated_img, (0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA
    )

    return calibrated_img


def save_depth_gt_img(
    img_num: int,
    depth_gt: np.ndarray,
    cam_calib: dict,
    depth_gt_save_path: str,
    undist_img_file: str,
    save_png: bool
) -> np.array:
    """save depth ground truth(projected points)

    Args:
        img_num (int): undistored image number
        depth_gt (numpy.ndarray): depth values(projected)
        cam_calib (dict): camera calibration information
        depth_gt_save_path (str): path to save depth_gt data
        undist_img_file (str): undistored image file path
        save_png (bool): whether save png images

    Returns:
        numpy.ndarray: projected point cloud data
    """
    img = np.zeros((cam_calib["size"]["height"], cam_calib["size"]["width"]), dtype=np.float32)
    backtorgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    undistored_img = cv2.imread(undist_img_file + str(format(img_num, "04")) + ".png")

    for x, y, depth in depth_gt:
        if depth < 0:
            depth = 0 # behind the car
            
        if img[int(y)][int(x)] == 0:
            img[int(y)][int(x)] = depth # if no depth data at this pixel, set by this value
            if 0 < depth <= 80:
                backtorgb[int(y)][int(x)] = (0, 0, 255) # max depth
        else:
            if img[int(y)][int(x)] > depth: # there is depth data, but bigger than this
                img[int(y)][int(x)] = depth  # if projected points are more than one, set the nearest value

    if save_png == True:
        undist = cv2.addWeighted(undistored_img, 0.8, backtorgb, 1.0, 0.0, dtype=cv2.CV_8U) # overlap gt on color image
        cv2.imwrite(depth_gt_save_path + "projection-" + str(format(img_num, "04")) + ".png", undist)
        cv2.imwrite(depth_gt_save_path + "depth_gt-" + str(format(img_num, "04")) + ".png", img) # only depth values

    return np.array(img)


def save_eval_result(eval_result: str, save_path: str) -> None:
    """save evaluation results in txt file

    Args:
        eval_result (str): text to save which describes evaluation results(metrcis)
        save_path (str): file path to save result
    """
    with open(save_path, "w") as f:
        f.write(eval_result)
