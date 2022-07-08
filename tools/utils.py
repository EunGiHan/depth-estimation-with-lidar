# -*- coding: utf-8 -*-

import numpy as np


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


def save_depth_gt_img(depth_gt, save_path) -> None:
    pass


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
