# -*- coding: utf-8 -*-

from cv2 import COLOR_BGR2GRAY
from matplotlib import font_manager
import numpy as np
import open3d
import cv2


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


def save_depth_gt_img(i: int, depth_gt:  np.ndarray, cam_calib: dict, save_path: str) -> None:
    img = np.zeros((cam_calib["size"]["height"], cam_calib["size"]["width"]), dtype=np.float32)
    src = np.zeros((cam_calib["size"]["height"], cam_calib["size"]["width"]), dtype=np.uint8)
    img2 = cv2.imread("data/2/"+str(format(i, "04"))+".png")
    img2 = cv2.cvtColor(img2, COLOR_BGR2GRAY)
    for x, y, depth in depth_gt:
        if img[int(y)][int(x)] == 0:
            img[int(y)][int(x)] = depth
            src[int(y)][int(x)] = img2[int(y)][int(x)]
        else:
            if img[int(y)][int(x)] > depth:
                img[int(y)][int(x)] = depth #투영된 3D 좌표가 여러개라면, 가까운 점이 우선 순위로 매김
            
    cv2.imshow("temp", src)
    cv2.waitKey()
    cv2.imwrite(save_path+str(format(i, "04"))+".png", img)



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


def make_eval_report(eval_result: dict) -> str:
    """make evaluation report in string

    Args:
        eval_result (dict): dictionary saving evaluation results

    Returns:
        str: report text to show in terminal and save in txt file
    """
    # TODO 예쁘게 꾸미기, 숫자 단위 확인해서 소숫점 맞추기
    report = ""
    for (method, value) in eval_result.items():
        report += "{0:<}\t\t{1:>2.3f}\n".format(method, value)

    return report
