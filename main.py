# -*- coding: utf-8 -*-

"""main code

monocular depth estimation and evaluation with LiDAR
===========

Pipeline:
    1. (parse_args) get commandline parameters
    2. (main) get lidar raw data from dataset and project to image plane (+ save)
    3. (main) get image data from dataset
    4. (main) run depth estimation model and get estimates (+ save)
    5. (main) compare estimates & gt -> calculate errors with metrics (+ save)

Todo:
    * 아예 파라미터 클래스 만들까... proslam 클래스처럼
    * 빈칸들 채우기
    * TODO 표시된 부분 수정하기
    * 이미지 불러올 때 camera intrinsic 적용?!
    * kitti의 lidar 불러오는 것 처리하기!
    * txt, png 저장 이름도 이미지 & 라이다 데이터 이름과 맞출까?
"""

import sys
from datetime import datetime

from cv2 import mean
from typing_extensions import Self

from depth_estimation.inferencer import Inferencer
from tools.parsers import *
from tools.transformations import *
from tools.utils import *


def main(time):
    # number of bag file
    bag_num = 1
    # data paths
    cam_calib_file = "dataset/ACE/calibration.yaml"
    lidar_calib_file = "dataset/ACE/calibration.yaml"
    image_file = ""  # TODO 이미지 파일 경로
    point_cloud_file = "outputs/ex_point_cloud.pcd"
    lidar_npy_file = "data/" + str(bag_num) + "/"
    model_npy_file = "data/" + str(bag_num) + "/"
    infer_load = "data/"

    # set save paths (without extension)
    depth_gt_save_path = "./outputs/" + str(bag_num) + "/depth_gt-"
    depth_map_save_path = "./outputs/" + str(bag_num) + "/depth_map-"
    eval_result_save_path = "./outputs/" + str(bag_num) + "/eval_result-"

    # parse calibration files -> get information
    cam_calib = parse_cam_calib(command_args.dataset, cam_calib_file)
    lidar_calib = parse_lidar_calib(command_args.dataset, lidar_calib_file)
    inferencer = Inferencer(args=command_args, dataload_path=infer_load)
    pred_depths = inferencer.infer()
    eval_result = []  # 각종 수치자료 (예) dict {'SILog': xxxxx, 'MASE': xxxxx, ...}

    for i in range(1, len(pred_depths)):
        print(str(i) + " start!")
        # get lidar raw data from dataset and project to image plane (+ save)
        lidar_point_cloud = convert_npy_to_xyz(
            lidar_npy_file + str(format(i, "04")) + ".npy"
        )  # npy to XYZ format
        # point_cloud = convert_pcd_to_xyz(point_cloud_file)  # pcd to XYZ format
        depth_gt = project_lidar_to_cam(
            command_args.dataset, cam_calib, lidar_calib, lidar_point_cloud
        )
        save_depth_txt(depth_gt, depth_gt_save_path + str(format(i, "04")) + ".txt")
        save_depth_npy(depth_gt, depth_gt_save_path + str(format(i, "04")))
        resize_depth_gt = save_depth_gt_img(i, depth_gt, cam_calib, model_npy_file)

        # get image data from dataset
        images = None  # TODO 이미지 경로(image_file)에서 불러오기 / 데이터셋에 따라 다르다면 아래 주석을 풀어서 대신 사용
        if command_args.dataset == "ace":
            image = load_pred_img(i, cam_calib)
        elif command_args.dataset == "kitti":
            image = load_pred_img(i, cam_calib)

        # run depth estimation model and get estimates (+ save)
        # TODO 희평 님 이곳에 채워주세요. 리턴은 depth_map
        # depth_map = inferencer.infer_single_img(image)
        depth_map = pred_depths[i][0]
        print(depth_map.shape)
        save_depth_txt(depth_map, depth_map_save_path + str(format(i, "04")) + ".txt")
        save_depth_npy(depth_map, depth_map_save_path + str(format(i, "04")))
        save_depth_map_img(depth_gt, depth_map_save_path + ".png")

        # compare estimates & gt -> calculate errors with metrics (+ save)
        # TODO 현진 님 이곳에 채워주세요. 리턴은 eval_result
        report = make_eval_report(resize_depth_gt, depth_map, cam_calib)
        eval_result.append(report)
        # save_eval_result(report, eval_result_save_path)

    dict_list = ["a1", "a2", "a3", "abs_rel", "rmse", "log_10", "rmse_log", "silog", "sq_rel"]
    dict_eval = []
    for dic in dict_list:
        result = sum(item[dic] for item in eval_result) / len(eval_result)
        dict_eval.append(result)
    print(dict_eval)


if __name__ == "__main__":
    time = (datetime.now()).strftime("%Y_%m_%d-%H_%M_%S")
    command_args = parse_args()
    main(time)
