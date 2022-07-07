#!/usr/bin/env python

import numpy as np
import yaml

"""
Requirements: pip install pyyaml
"""


def parse_cam_calib(file_path):
    with open(file_path) as f:
        calib_info = yaml.load(f, Loader=yaml.FullLoader)['camera']['front']
        # print(calib_info)
        """
        'D': [[-0.3713184655742523], [0.1894083454473062], [0.0017443421254646307], [0.00037526691609012837], [-0.06081438434204424]], 
        'H': [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], 
        'K': [[1365.4887468866116, 0.0, 1026.5997744850633], [0.0, 1366.2954658193316, 468.9522311262687], [0.0, 0.0, 1.0]], 
        'P': [[1046.688720703125, 0.0, 1033.3313677806436, 0.0], [0.0, 1277.919921875, 460.2549448068021, 0.0], [0.0, 0.0, 1.0, 0.0]], 
        'R': [[0.0122169643644543, -0.9999223846340135, 0.0024434585213449255], [0.08370134546701578, -0.0014124149699195523, -0.9964898844699649], [0.996415992720274, 0.012378602173938212, 0.0836775935330984]], 
        'T': [[0.19247596939426734], [1.2632069552043956], [-2.8884357686348134]], 
        'model': 'pinhole', 
        'roi': {}, 
        'size': {'height': 1086, 'width': 2040}}
        """
    
        cam_calib = {}
        cam_calib['D'] = calib_info['D']
        cam_calib['H'] = calib_info['H']
        cam_calib['K'] = calib_info['K']
        cam_calib['P'] = calib_info['P']
        cam_calib['R'] = calib_info['R']
        cam_calib['T'] = calib_info['T']
        cam_calib['S'] = calib_info['size']

    return cam_calib


def parse_lidar_calib(file_path):
    with open(file_path) as f:
        calib_info = yaml.load(f, Loader=yaml.FullLoader)['lidar']['rs80']
    
        lidar_calib = {}
        lidar_calib['R'] = calib_info['R'] # [[0.9999999553735484, -3.4906585032797835e-05, 0.000296705968304882], [3.4906583496307106e-05, 0.9999999993907651, 1.0356992118682821e-08], [-0.0002967059684856456, 8.271806125530277e-25, 0.9999999559827831]]
        lidar_calib['t'] = calib_info['T'] # [[1.5], [0.03], [2]] # translation vector

    return lidar_calib
