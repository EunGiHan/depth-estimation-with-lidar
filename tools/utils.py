# -*- coding: utf-8 -*-

import string
import os
import cv2
import depth
import numpy as np
import open3d
from cv2 import COLOR_BGR2GRAY
from matplotlib import font_manager

def get_files_count(folder_path):
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
    np.save(save_path, depth)


def load_pred_img(i: int, cam_calib: dict, model_npy_file: str):
    img2 = cv2.imread(model_npy_file + str(format(i, "04")) + ".png")
    height, width, channel = img2.shape

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
        img2,
        mapx,
        mapy,
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
    )

    calibrated_img = cv2.resize(
        calibrated_img, (0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA
    )

    return calibrated_img


def save_depth_gt_img(i: int, depth_gt: np.ndarray, cam_calib: dict, proj_save_path: str, img_save_path: str, undist_img_file: str) -> np.array:
    img = np.zeros((cam_calib["size"]["height"], cam_calib["size"]["width"]), dtype=np.float32)
    backtorgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    calibrated_img = cv2.imread(undist_img_file + str(format(i, "04")) + ".png")

    for x, y, depth in depth_gt:
        if depth < 0:
            depth = 0
        if img[int(y)][int(x)] == 0:
            img[int(y)][int(x)] = depth
            if 0 < depth <= 80:
                backtorgb[int(y)][int(x)] = (0, 0, 255)
        else:
            if img[int(y)][int(x)] > depth:
                img[int(y)][int(x)] = depth  # 투영된 3D 좌표가 여러개라면, 가까운 점이 우선 순위로 매김
    #print(np.max(img))

    undist = cv2.addWeighted(calibrated_img, 0.8, backtorgb, 1.0, 0.0, dtype=cv2.CV_8U)
    # cv2.imwrite(img_save_path + str(format(i, "04")) + ".png", calibrated_img)
    # cv2.imwrite(proj_save_path + "projection-" + str(format(i, "04")) + ".png", undist)
    # cv2.imwrite(proj_save_path + "depth_gt-" + str(format(i, "04")) + ".png", img)
    return np.array(img)


def save_depth_map_img(depth_map, save_path) -> None:
    pass


def save_depth_overlap_img(depth_gt, depth_map, save_path) -> None:
    pass


def save_eval_result(eval_result: str, save_path: str) -> None:
    """save evaluation results in txt file

    Args:
        eval_result (str): text to save which describes evaluation results(metrcis)
        save_path (str): file path to save result
    """
    with open(save_path, "w") as f:
        f.write(eval_result)


def make_eval_report(img_num: int, depth_gt: np.array, depth_map: np.array, cam_calib: dict):
    """make evaluation report in string

    Args:
        eval_result (dict): dictionary saving evaluation results

    Returns:
        str: report text to show in terminal and save in txt file
    """
    result = [] 
    result_rev = []
    new_gt = []
    new_pred = []
    min_depth = 0
    max_depth = 80

    for i in range(len(depth_gt)):
        temp_r = []
        temp_r_v = []
        for j in range(len(depth_gt[0])):
            if min_depth < depth_gt[i][j] <= max_depth:
                temp_r.append(depth_gt[i][j] / depth_map[i][j])
                temp_r_v.append(depth_map[i][j] / depth_gt[i][j])
                new_gt.append(depth_gt[i][j])
                new_pred.append(depth_map[i][j])
        result.append(temp_r)
        result_rev.append(temp_r_v)

    new_gt = np.array(new_gt)
    new_pred = np.array(new_pred)
    #print(np.max(new_gt), np.max(new_pred))
    thresh = np.maximum(np.array(result), np.array(result_rev))
    thresh = np.array(thresh)
    a1, a2, a3, length= 0, 0, 0, 0
    for row in thresh:
        for item in row:
            if item < 1.25:
                a1 += 1
            if item < 1.25**2:
                a2 += 1
            if item < 1.25**3:
                a3 += 1
            length += 1
    
    a1 /= length
    a2 /= length
    a3 /= length

    print(a1, a2, a3)

    # print(type(thresh))
    # print(type(thresh))
    # print(type(thresh[0]))
    # print(thresh)
    # a1 = (thresh < 1.25).mean()
    # a2 = (thresh < 1.25**2).mean()
    # a3 = (thresh < 1.25**3).mean()

    abs_rel = np.mean(np.abs(new_gt - new_pred) / new_gt)
    sq_rel = np.mean(((new_gt - new_pred) ** 2) / new_gt)

    rmse = (new_gt - new_pred) ** 2
    rmse = np.sqrt(rmse.mean())

    rmse_log = (np.log(new_gt) - np.log(new_pred)) ** 2
    rmse_log = np.sqrt(rmse_log.mean())

    err = np.log(new_pred) - np.log(new_gt)
    silog = np.sqrt(np.mean(err**2) - np.mean(err) ** 2) * 100

    log_10 = (np.abs(np.log10(new_gt) - np.log10(new_pred))).mean()
    # return dict(a1=a1, a2=a2, a3=a3, abs_rel=abs_rel, rmse=rmse, log_10=log_10, rmse_log=rmse_log, silog=silog, sq_rel=sq_rel)
    #print(
    #    dict(
    #        a1=a1,
    #        a2=a2,
    #        a3=a3,
    #        abs_rel=abs_rel,
    #        rmse=rmse,
    #        log_10=log_10,
    #        rmse_log=rmse_log,
    #        silog=silog,
    #        sq_rel=sq_rel,
    #    )
    #)
    report = dict(
        a1=a1,
        a2=a2,
        a3=a3,
        abs_rel=abs_rel,
        rmse=rmse,
        log_10=log_10,
        rmse_log=rmse_log,
        silog=silog,
        sq_rel=sq_rel,
    )

    return report, make_report_str(img_num, report)


def make_report_str(img_num: int, report: dict) -> str:
    info = "{0:>5}  {1:8.3f}  {2:8.3f}  {3:8.3f}  {4:8.3f}  {5:8.3f}  {6:8.3f}  {7:8.3f}  {8:8.3f}  {9:8.3f}\n".format(
        str(img_num).zfill(4),
        report["a1"],
        report["a2"],
        report["a3"],
        report["rmse"],
        report["rmse_log"],
        report["silog"],
        report["abs_rel"],
        report["sq_rel"],
        report["log_10"],
    )
    return info


def make_final_report_str(bag_num: int, report: list):
    f_report = "\n"
    f_report += "=" * 96
    f_report += "\n"
    f_report += "{:^96}\n".format("Final Report")
    f_report += "=" * 96
    f_report += "\n"

    f_report += (
        "{0:>5}  {1:>8}  {2:>8}  {3:>8}  {4:>8}  {5:>8}  {6:>8}  {7:>8}  {8:>8}  {9:>8}\n".format(
            "Bag", "thr1", "thr2", "thr3", "RMSE", "RMSElog", "SILog", "AbsRel", "SqRel", "log_10"
        )
    )
    f_report += "{0:>5}  {1:8.3f}  {2:8.3f}  {3:8.3f}  {4:8.3f}  {5:8.3f}  {6:8.3f}  {7:8.3f}  {8:8.3f}  {9:8.3f}\n".format(
        "0" + str(bag_num),
        report[0],
        report[1],
        report[2],
        report[3],
        report[4],
        report[5],
        report[6],
        report[7],
        report[8],
    )
    f_report += "=" * 96

    return f_report


def eval_header():
    head = "-" * 96
    head += "\n"
    head += (
        "{0:>5}  {1:>8}  {2:>8}  {3:>8}  {4:>8}  {5:>8}  {6:>8}  {7:>8}  {8:>8}  {9:>8}\n".format(
            "no.", "thr1", "thr2", "thr3", "RMSE", "RMSElog", "SILog", "AbsRel", "SqRel", "log_10"
        )
    )
    head += "-" * 96
    #print(head)
    head += "\n"
    return head
