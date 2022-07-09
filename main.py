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

from depth_estimation.inferencer import Inferencer
from tools.parsers import *
from tools.transformations import *
from tools.utils import *


def main(time):
    # data paths
    cam_calib_file = "dataset/ACE/calibration.yaml"
    lidar_calib_file = "dataset/ACE/calibration.yaml"
    image_file = ""  # TODO 이미지 파일 경로
    point_cloud_file = "outputs/ex_point_cloud.pcd"
    npy_file = "data/1/"

    # set save paths (without extension)
    depth_gt_save_path = "./outputs/2/"
    depth_map_save_path = "./outputs/depth_map-" + time
    eval_result_save_path = "./outputs/eval_result-" + time

    # parse calibration files -> get information
    cam_calib = parse_cam_calib(command_args.dataset, cam_calib_file)
    lidar_calib = parse_lidar_calib(command_args.dataset, lidar_calib_file)
    for i in range(1, 908):
        print(str(i) + " start!")
        # get lidar raw data from dataset and project to image plane (+ save)
        point_cloud = convert_npy_to_xyz(
            npy_file + str(format(i, "04")) + ".npy"
        )  # npy to XYZ format
        # point_cloud = convert_pcd_to_xyz(point_cloud_file)  # pcd to XYZ format
        depth_gt = project_lidar_to_cam(command_args.dataset, cam_calib, lidar_calib, point_cloud)
        # save_depth_txt(depth_gt, depth_gt_save_path + ".txt")
        save_depth_gt_img(i, depth_gt, cam_calib, depth_gt_save_path)

    # get image data from dataset
    images = None  # TODO 이미지 경로(image_file)에서 불러오기 / 데이터셋에 따라 다르다면 아래 주석을 풀어서 대신 사용
    # if command_args.dataset == "ace":
    #     image = None
    # elif command_args.dataset == "kitti":
    #     image = None

    # run depth estimation model and get estimates (+ save)
    # TODO 희평 님 이곳에 채워주세요. 리턴은 depth_map

    inferencer = Inferencer(args=command_args)
    depth_map = (
        inferencer.infer()
    )  # 이미지에서 뽑은 depth map 배열 (예) [[u, v, gt_depth], [u, v, gt_depth], ...]
    # put image one by one
    # for img in images:
    #     output = inferencer.infer_single_img(img)
    #     depth_map.append(output)
    save_depth_txt(depth_map, depth_map_save_path + ".txt")
    # save_depth_map_img(depth_gt, depth_map_save_path + ".png")

    # compare estimates & gt -> calculate errors with metrics (+ save)
    # TODO 현진 님 이곳에 채워주세요. 리턴은 eval_result
    eval_result = None  # 각종 수치자료 (예) dict {'SILog': xxxxx, 'MASE': xxxxx, ...}
    report = make_eval_report(eval_result)
    save_eval_result(report, eval_result_save_path)


if __name__ == "__main__":
    time = (datetime.now()).strftime("%Y_%m_%d-%H_%M_%S")
    command_args = parse_args()
    main(time)
