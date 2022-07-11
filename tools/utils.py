# -*- coding: utf-8 -*-

import cv2
import numpy as np
import open3d
from cv2 import COLOR_BGR2GRAY
from matplotlib import font_manager


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



def save_depth_gt_img(i: int, depth_gt: np.ndarray, cam_calib: dict, save_path: str) -> np.array:
    img = np.zeros((cam_calib["size"]["height"], cam_calib["size"]["width"]), dtype=np.float32)
    backtorgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    img2 = cv2.imread("data/image_plane/" + str(format(i, "04")) + ".png")
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
    print(np.max(img))
    undist = cv2.addWeighted(calibrated_img, 0.8, backtorgb, 1.0, 0.0, dtype=cv2.CV_8U)
    dist = cv2.addWeighted(img2, 0.3, backtorgb, 1.0, 0.0, dtype=cv2.CV_8U)
    # calibrated_img = cv2.resize(calibrated_img, (0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    # img = cv2.resize(img, (0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    print(np.max(img))
    # cv2.imwrite(save_path+"image_plane/"+str(format(i, "04"))+".png", calibrated_img)
    # cv2.imwrite(save_path+str(format(i, "04"))+".png", img)
    cv2.imwrite(save_path + str(format(i, "04")) + "_undist.png", undist)
    cv2.imwrite(save_path + str(format(i, "04")) + "_dist.png", dist)
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


def make_eval_report(depth_gt: np.array, depth_map: np.array, cam_calib: dict) -> str:
    """make evaluation report in string

    Args:
        eval_result (dict): dictionary saving evaluation results

    Returns:
        str: report text to show in terminal and save in txt file
    """
    # TODO 예쁘게 꾸미기, 숫자 단위 확인해서 소숫점 맞추기
    result = np.zeros((cam_calib["size"]["height"], cam_calib["size"]["width"]), dtype=np.float32)
    result_rev = np.zeros(
        (cam_calib["size"]["height"], cam_calib["size"]["width"]), dtype=np.float32
    )
    new_gt = []
    new_pred = []
    min_depth = 0
    max_depth = 80

    for i in range(len(depth_gt)):
        for j in range(len(depth_gt[0])):
            if min_depth < depth_gt[i][j] <= max_depth:
                result[i][j] = depth_gt[i][j] / depth_map[i][j]
                result_rev[i][j] = depth_map[i][j] / depth_gt[i][j]
                new_gt.append(depth_gt[i][j])
                new_pred.append(depth_map[i][j])
                # print(depth_gt[i][j], depth_map[i][j])

    new_gt = np.array(new_gt)
    new_pred = np.array(new_pred)
    print(np.max(new_gt), np.max(new_pred))
    thresh = np.maximum(result, result_rev)
    a1 = (thresh < 1.25).mean()
    a2 = (thresh < 1.25**2).mean()
    a3 = (thresh < 1.25**3).mean()

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
    print(
        dict(
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
    )
    report = ""
    # for (method, value) in eval_result.items():
    #     report += "{0:<}\t\t{1:>2.3f}\n".format(method, value)

    return report
