# -*- coding: utf-8 -*-

import os
# from pathlib import Path
# import cv2
# import cv2.aruco as aruco
import numpy as np

from robot.FRArmRobot import FRArmRobot
from robot.CommonParam import *

ESC_KEY = 27

aruco_marker_length = 7  # - [cm]
# 相机内参矩阵
camera_matrix = np.array([
    [451.371351, 0, 329.290322],
    [0, 452.274542, 242.585600],
    [0, 0, 1]], dtype=np.float32)

# 畸变系数（如果有的话）
dist_coefficients = np.array([0.081189, -0.124491, 0.000076, 0.001919, 0.000000])

# cur_path = Path(os.path.realpath(__file__))
# save_path = cur_path.parent.joinpath('arm_cam_data')
# save_path.mkdir(exist_ok=True)


def main():
    # Arm
    robot = FRArmRobot('192.168.58.2')
    # 获取当前机械臂的关节角度
    cur_joint_angle = robot.get_cur_joint_angle(BlockMode.No)
    # 获取当前机械臂末端的笛卡尔坐标
    cur_cartesian_coordinate = robot.get_cur_cartesian_coordinate(BlockMode.No)
    print(cur_joint_angle)
    print(cur_cartesian_coordinate)


if __name__ == "__main__":
    main()
