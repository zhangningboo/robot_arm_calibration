import os
from pathlib import Path
import time
import multiprocessing as mp
from multiprocessing import Manager
from threading import Thread
import camera
from pyorbbecsdk import Pipeline, FrameSet
from pyorbbecsdk import Config
from pyorbbecsdk import OBSensorType, OBFormat
from pyorbbecsdk import OBError
from pyorbbecsdk import VideoStreamProfile
import cv2
import cv2.aruco as aruco
import numpy as np
from utils import frame_to_bgr_image

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

cur_path = Path(os.path.realpath(__file__))
save_path = cur_path.parent.joinpath('arm_cam_data')
save_path.mkdir(exist_ok=True)


def main():
    # Arm
    robot = FRArmRobot('192.168.33.211')
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)  # 尺寸需要和打印的ArUco标记保持一致
    # 初始化ArUco参数
    parameters = aruco.DetectorParameters_create()
    # 相机
    config = Config()
    pipeline = Pipeline()
    try:
        profile_list = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        try:
            color_profile: VideoStreamProfile = profile_list.get_video_stream_profile(640, 0, OBFormat.RGB, 30)
        except OBError as e:
            print(e)
            color_profile = profile_list.get_default_video_stream_profile()
        config.enable_stream(color_profile)
    except Exception as e:
        print(e)
        return
    pipeline.start(config)
    cnt = 0
    while True:
        try:
            frames: FrameSet = pipeline.wait_for_frames(100)
            if frames is None:
                continue
            color_frame = frames.get_color_frame()
            if color_frame is None:
                continue
            # covert to RGB format
            color_image = frame_to_bgr_image(color_frame)
            if color_image is None:
                print("failed to convert frame to image")
                continue
            image = color_image.copy()
            # 检测ArUco标记
            corners, ids, rejectedImgPoints = aruco.detectMarkers(image, aruco_dict, parameters=parameters)
            # 获取相机姿势
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, aruco_marker_length, camera_matrix,
                                                              dist_coefficients)
            image_with_markers = None
            if tvecs is not None:
                # 在图像上绘制检测到的标记及其坐标轴
                for i in range(len(ids)):
                    cv2.drawFrameAxes(image, camera_matrix, dist_coefficients, rvecs[i], tvecs[i], 1)
                # 在图像上绘制检测到的标记
                image_with_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)
                cv2.imshow("Color Viewer", image_with_markers)
            else:
                cv2.imshow("Color Viewer", color_image)
            key = cv2.waitKey(10)
            if key == ord('q') or key == ESC_KEY:
                break
            elif key == ord('s') or key == ord('S'):
                # 获取当前机械臂的关节角度
                _, *cur_joint_angle = robot.get_cur_joint_angle(BlockMode.No)
                # 获取当前机械臂末端的笛卡尔坐标
                _, *cur_cartesian_coordinate = robot.get_cur_cartesian_coordinate(BlockMode.No)
                data = {
                    'arm': {
                        'joint_angle': cur_joint_angle,
                        'cartesian_coordinate': cur_cartesian_coordinate,
                    },
                    'image': color_image,
                    'aruco': image_with_markers,
                    'aruco_marker_length': aruco_marker_length,
                    'aruco_dict': 'DICT_5X5_250',
                }
                np.save(save_path.joinpath(f'{cnt}.npy'), data)
                cv2.imwrite(save_path.joinpath(f'{cnt}.jpg').absolute().as_posix(), color_image)
                cnt += 1
                print(f'Record data file {cnt} success!')
        except KeyboardInterrupt:
            break
    pipeline.stop()


if __name__ == "__main__":
    main()
