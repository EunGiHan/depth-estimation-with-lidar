from datetime import datetime

from tools.parsers import *
from tools.utils import *
from tools.transformations import *

# TODO 여러 이미지를 넣어서 배치 처리 할 것인지? 여러 이미지 사용해(반복) 평균적인 통계를 낼 것인지?


def main(time):
    depth_gt_save_path = "./outputs/depth_gt_" + time  # TODO 변수명 헷갈릴 듯?
    depth_map_save_path = "./outputs/depth_map_" + time
    eval_result_save_path = "./outputs/eval_result_" + time

    """
    1. lidar data projection -> parse
    2. get image
    3. get depth estimation model output (save txt & image, either)
    4. compare 1 and 3 -> print termianl & save txt
    """

    # 1. lidar data projection -> parse
    lidar_points = convert_lidar_to_cam()
    # TODO param 넣기!
    # [[u, v, gt_depth], [u, v, gt_depth], [u, v, gt_depth], ...] for one image

    # 2. get image
    image = None  # 이미지 경로 지정해주고 불러오기

    # 3. get depth estimation model output (save txt & image, either)
    ### 채우기
    depth_map = (
        None  # 이미지에서 뽑은 depth map 배열 [[u, v, gt_depth], [u, v, gt_depth], [u, v, gt_depth], ...]
    )

    # 4. compare 1 and 3 -> print termianl & save txt
    ### 채우기
    eval_result = None  # 각종 수치자료 dict {'SILog': xxxxx, 'MASE': xxxxx, ...}
    save_eval_result(eval_result)


if __name__ == "__main__":
    time = (datetime.now()).strftime("%Y-%m-%d_%H-%M-%S")
    command_args = parse_args(time)
    main(time)
