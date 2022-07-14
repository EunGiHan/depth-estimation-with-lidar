# -*- coding: utf-8 -*-

"""main code

monocular depth estimation and evaluation with LiDAR
===========

Process:
    1. get commandline parameters, paths, calibration info
    2. undistort images
    3. run depth estimation model for ALL IMAGES IN DATASET
    4. evaluate estimation quality for EACH IMAGES
        4-1. get inference(depth estimation)
        4-2. project lidar point cloud and set as ground truth
        4-3. evaluate estimated depth value
"""

from depth_estimation.inferencer import Inferencer
from evaluation_using_lidar.evaluation import *
from tools.parsers import *
from tools.transformations import *
from tools.utils import *


def main():
    bag_num = 1  # number of bag file
    eval_result = []  # evaluation result (for calculation)
    eval_result_str = ""  # evaluation result (for print)

    # SET PATHS
    if command_args.dataset == "ace":
        cam_calib_file = "dataset/ACE/calibration.yaml"  # camera calibration file
        lidar_calib_file = "dataset/ACE/calibration.yaml"  # lidar calibration file
    elif command_args.dataset == "kitti":
        cam_calib_file = "dataset/KITTI/2011_09_26/2011_09_26_calib/calib_cam_to_cam.txt"
        lidar_calib_file = "dataset/KITTI/2011_09_26/2011_09_26_calib/calib_velo_to_cam.txt"
    lidar_npy_file = "outputs/" + str(bag_num) + "/raw_pc/"  # lidar raw data path
    raw_img_file = "outputs/" + str(bag_num) + "/raw_img/"  # raw image path
    infer_load = "outputs/"

    undist_img_file = "outputs/" + str(bag_num) + "/undist_img/"  # undistorted image path
    depth_gt_save_path = (
        "outputs/" + str(bag_num) + "/depth_gt/"
    )  # GT(projected points) path (png, txt, npy)
    depth_map_save_path = (
        "outputs/" + str(bag_num) + "/depth_map/"
    )  # depth map path (png, txt, npy)
    eval_result_save_path = (
        "outputs/" + str(bag_num) + "/eval_result.txt"
    )  # evaluation result file name

    # GET CALIBRATION INFO
    cam_calib = parse_cam_calib(command_args.dataset, cam_calib_file)
    lidar_calib = parse_lidar_calib(command_args.dataset, lidar_calib_file)

    # UNDISTORT IMAGE
    if get_files_count(undist_img_file) != get_files_count(
        raw_img_file
    ):  # check if already undistorted images
        for i in range(1, get_files_count(raw_img_file) + 1):
            undistored_img = undistort_image(i, cam_calib, raw_img_file)
            cv2.imwrite(undist_img_file + str(format(i, "04")) + ".png", undistored_img)

    # DEPTH ESTIMATION: model inference if depth maps don't exist
    inferencer = None
    pred_depths = None
    if (
        get_files_count(undist_img_file) != get_files_count(raw_img_file)
        or get_files_count(depth_map_save_path) < get_files_count(undist_img_file) * 3
    ):
        inferencer = Inferencer(args=command_args, dataload_path=infer_load)
        pred_depths = inferencer.infer(depth_map_save_path)

    # EVALUATION
    eval_result_str += eval_header()
    for img_num in range(1, get_files_count(raw_img_file) + 1):
        # print indices for each 50 images
        if img_num % 50 == 0:
            eval_result_str += eval_header()

        # if there already exist depth maps -> get depth estimation
        if inferencer == None:
            pred_depths = convert_npy_to_xyz_only_depth(
                depth_map_save_path + "depth_map-" + str(format(img_num, "04")) + ".npy"
            )

        # get lidar raw data from dataset and project to image plane (+ save)
        lidar_point_cloud = convert_npy_to_xyz(
            lidar_npy_file + str(format(img_num, "04")) + ".npy"
        )  # npy to XYZ format
        depth_gt = project_lidar_to_cam(
            command_args.dataset, cam_calib, lidar_calib, lidar_point_cloud
        )

        # SAVE GT DATA
        if command_args.save_array == True:
            save_depth_txt(
                depth_gt, depth_gt_save_path + "/depth_gt-" + str(format(img_num, "04")) + ".txt"
            )
            save_depth_npy(depth_gt, depth_gt_save_path + "/depth_gt-" + str(format(img_num, "04")))
        resize_depth_gt = save_depth_gt_img(
            img_num, depth_gt, cam_calib, depth_gt_save_path, undist_img_file, command_args.save_png
        )

        # GET DEPTH_MAP DATA
        if inferencer == None:
            depth_map = pred_depths  # for one image (result already exist)
        else:
            depth_map = pred_depths[img_num - 1][0]  # all images (run inference in this run)

        # SAVE DEPTH_MAP DATA
        if command_args.save_array == True:
            save_depth_txt(
                depth_map, depth_map_save_path + "depth_map-" + str(format(i, "04")) + ".txt"
            )
            save_depth_npy(depth_map, depth_map_save_path + "depth_map-" + str(format(i, "04")))
        if command_args.save_png == True:
            save_depth_map_img(depth_gt, depth_map_save_path + ".png")

        # EVALUATION: ground truth(lidar) & estimation(depth map)
        report, report_str = make_eval_report(img_num, resize_depth_gt, depth_map, cam_calib)

        # PRINT EVALUATION RESULT
        print(report_str)  # evaluation result for one image
        eval_result_str += report_str
        eval_result.append(report)

    # AVERAGE ALL IMAGES IN DATASET(BAG FILE)
    dict_list = ["a1", "a2", "a3", "rmse", "rmse_log", "silog", "abs_rel", "sq_rel", "log_10"]
    dict_eval = []
    for dic in dict_list:
        result = sum(item[dic] for item in eval_result) / len(eval_result)
        dict_eval.append(result)
    f_report = make_final_report_str(bag_num, dict_eval)
    print(f_report)  # evaluation result for all images
    eval_result_str += f_report
    save_eval_result(eval_result_str, eval_result_save_path)


if __name__ == "__main__":
    command_args = parse_args()
    main()
