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


def main():
    # number of bag file
    bag_num = 1
    # data paths
    cam_calib_file = "dataset/ACE/calibration.yaml"
    lidar_calib_file = "dataset/ACE/calibration.yaml"
    image_file = ""  # 이미지 파일 경로
    point_cloud_file = "outputs/ex_point_cloud.pcd"
    lidar_npy_file = "outputs/" + str(bag_num) + "/raw_pc/"  # raw 라이다 데이터 불러옴
    model_npy_file = "outputs/" + str(bag_num) + "/raw_img/"  # raw 이미지 데이터 불러옴
    infer_load = "outputs/"  # "data"를 output으로 바꿉니다.

    # set save paths (without extension)
    depth_gt_save_path = "./outputs/" + str(bag_num) + "/depth_gt-"
    depth_map_save_path = "./outputs/" + str(bag_num) + "/depth_map-"
    eval_result_save_path = "./outputs/" + str(bag_num) + "/eval_result.txt"

    # parse calibration files -> get information
    cam_calib = parse_cam_calib(command_args.dataset, cam_calib_file)
    lidar_calib = parse_lidar_calib(command_args.dataset, lidar_calib_file)
    inferencer = Inferencer(args=command_args, dataload_path=infer_load)
    pred_depths = inferencer.infer()
    final_report_str = ""
    eval_result = []

    final_report_str += eval_header()  # print도 함

    for i in range(1, len(pred_depths)):
        if i % 50 == 0:
            final_report_str += eval_header()  # print도 함

        # get lidar raw data from dataset and project to image plane (+ save)
        lidar_point_cloud = convert_npy_to_xyz(
            lidar_npy_file + str(format(i, "04")) + ".npy"
        )  # npy to XYZ format
        depth_gt = project_lidar_to_cam(
            command_args.dataset, cam_calib, lidar_calib, lidar_point_cloud
        )
        save_depth_txt(depth_gt, depth_gt_save_path + str(format(i, "04")) + ".txt")
        save_depth_npy(depth_gt, depth_gt_save_path + str(format(i, "04")))
        resize_depth_gt = save_depth_gt_img(i, depth_gt, cam_calib, model_npy_file)

        # run depth estimation model and get estimates (+ save)
        # depth_map = inferencer.infer_single_img(image)
        depth_map = pred_depths[i][0]
        # print(depth_map.shape)
        save_depth_txt(depth_map, depth_map_save_path + str(format(i, "04")) + ".txt")
        save_depth_npy(depth_map, depth_map_save_path + str(format(i, "04")))
        save_depth_map_img(depth_gt, depth_map_save_path + ".png")

        # compare estimates & gt -> calculate errors with metrics (+ save)
        report, report_str = make_eval_report(i, resize_depth_gt, depth_map, cam_calib)
        print(report_str)  # 터미널 출력
        final_report_str += report_str
        eval_result.append(report)

    dict_list = ["a1", "a2", "a3", "rmse", "rmse_log", "silog", "abs_rel", "sq_rel", "log_10"]
    dict_eval = []
    for dic in dict_list:
        result = sum(item[dic] for item in eval_result) / len(eval_result)
        dict_eval.append(result)
    f_report = make_final_report_str(bag_num, dict_eval)
    print(f_report)  # 터미널 출력
    final_report_str += f_report

    save_eval_result(final_report_str, eval_result_save_path)


if __name__ == "__main__":
    command_args = parse_args()
    main()
