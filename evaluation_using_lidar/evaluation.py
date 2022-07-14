import numpy as np


def make_eval_report(img_num: int, depth_gt: np.ndarray, depth_map: np.ndarray) -> tuple:
    """make evaluation report in string

    Args:
        img_num (int): image number
        depth_gt (numpy.ndarray): projected point data
        depth_map (numpy.ndarray): depth map(estimated depth)

    Returns:
        dict: evaluation result
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
    
    thresh = np.maximum(np.array(result), np.array(result_rev))
    thresh = np.array(thresh)
    a1, a2, a3, length = 0, 0, 0, 0
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

    abs_rel = np.mean(np.abs(new_gt - new_pred) / new_gt)
    sq_rel = np.mean(((new_gt - new_pred) ** 2) / new_gt)

    rmse = (new_gt - new_pred) ** 2
    rmse = np.sqrt(rmse.mean())

    rmse_log = (np.log(new_gt) - np.log(new_pred)) ** 2
    rmse_log = np.sqrt(rmse_log.mean())

    err = np.log(new_pred) - np.log(new_gt)
    silog = np.sqrt(np.mean(err**2) - np.mean(err) ** 2) * 100

    log_10 = (np.abs(np.log10(new_gt) - np.log10(new_pred))).mean()
    
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
    """make evaluation result text line

    Args
        img_num (str): image number
        report (dict): evaluation result

    Returns
        str: evaluation result as text with formatting
    """
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


def eval_header() -> str:
    """evaluation result's index header

    Returns
        str: evaluation result's index header string
    """
    head = "-" * 96
    head += "\n"
    head += (
        "{0:>5}  {1:>8}  {2:>8}  {3:>8}  {4:>8}  {5:>8}  {6:>8}  {7:>8}  {8:>8}  {9:>8}\n".format(
            "no.", "thr1", "thr2", "thr3", "RMSE", "RMSElog", "SILog", "AbsRel", "SqRel", "log_10"
        )
    )
    head += "-" * 96
    head += "\n"

    print(head)

    return head
