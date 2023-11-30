"""
This demo calculates multiple things for different scenarios.

Here are the defined reference frames:

TAG:
                A y
                |
                |
                |tag center
                O---------> x

CAMERA:


                X--------> x
                | frame center
                |
                |
                V y

F1: Flipped (180 deg) tag frame around x axis
F2: Flipped (180 deg) camera frame around x axis

The attitude of a generic frame 2 respect to a frame 1 can obtained by calculating euler(R_21.T)

We are going to obtain the following quantities:
    > from aruco library we obtain tvec and Rct, position of the tag in camera frame and attitude of the tag
    > position of the Camera in Tag axis: -R_ct.T*tvec
    > Transformation of the camera, respect to f1 (the tag flipped frame): R_cf1 = R_ct*R_tf1 = R_cf*R_f
    > Transformation of the tag, respect to f2 (the camera flipped frame): R_tf2 = Rtc*R_cf2 = R_tc*R_f
    > R_tf1 = R_cf2 an symmetric = R_f


"""

import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math
import os
from pathlib import Path
import time
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

cur_path = Path(os.path.realpath(__file__))

# --- Define Tag
id_to_find = 2
marker_size = 7  # - [cm]
ESC_KEY = 27

save_path = cur_path.parent.joinpath('debug_camera_live')
save_path.mkdir(exist_ok=True)
# 相机内参矩阵
camera_matrix = np.array([
    [451.371351, 0, 329.290322],
    [0, 452.274542, 242.585600],
    [0, 0, 1]], dtype=np.float32)
# 畸变系数（如果有的话）
dist_coefficients = np.array([0.081189, -0.124491, 0.000076, 0.001919, 0.000000])


# ------------------------------------------------------------------------------
# ------- ROTATIONS https://www.learnopencv.com/rotation-matrix-to-euler-angles/
# ------------------------------------------------------------------------------
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


# # --- Get the camera calibration path
# calib_path = ""
# camera_matrix = np.loadtxt(calib_path + 'cameraMatrix_webcam.txt', delimiter=',')
# camera_distortion = np.loadtxt(calib_path + 'cameraDistortion_webcam.txt', delimiter=',')

# --- 180 deg rotation matrix around the x axis
R_flip = np.zeros((3, 3), dtype=np.float32)
R_flip[0, 0] = 1.0
R_flip[1, 1] = -1.0
R_flip[2, 2] = -1.0

# --- Define the aruco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
parameters = aruco.DetectorParameters_create()

# -- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN


def get_cam_pose_by_bord_pose(rvecs, tvecs):
    res = []
    for rvec, tvec in zip(rvecs, tvecs):
        # -- Obtain the rotation matrix tag->camera
        R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
        R_tc = R_ct.T

        # -- Get the attitude in terms of euler 321 (Needs to be flipped first)
        roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip * R_tc)

        # -- Now get Position and attitude f the camera respect to the marker
        pos_camera = -R_tc * np.matrix(tvec).T

        # -- Get the attitude of the camera respect to the frame
        roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip * R_tc)
        res.append([[*tvec[0], roll_marker, pitch_marker, yaw_marker],
                    [*[item[0] for item in pos_camera.tolist()], roll_camera, pitch_camera, yaw_camera]])
    return res


def main():
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
    cnt = 200
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
            # -- Convert in gray scale
            frame = color_image.copy()
            # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # -- remember, OpenCV stores color images in Blue, Green, Red

            # -- Find all the aruco markers in the image
            corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
            # print(ids)
            if ids is not None and id_to_find in ids:
                # -- ret = [rvec, tvec, ?]
                # -- array of rotation and position of each marker in camera frame
                # -- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
                # -- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame
                ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coefficients)

                # -- Unpack the output, get only the first
                rvecs, tvecs = ret[0], ret[1]
                rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]

                # -- Draw all the detected marker and put a reference frame over it
                aruco.drawDetectedMarkers(frame, corners)
                for i in range(len(ids)):
                    cv2.drawFrameAxes(frame, camera_matrix, dist_coefficients, rvecs[i], tvecs[i], 5)

                res = get_cam_pose_by_bord_pose(rvecs, tvecs)
                res = np.array(res)
                for i, [marker_pose, camera_pose] in enumerate(res):
                    i *= 2
                    camera_pose = np.array(camera_pose)
                    str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f" % (
                        camera_pose[0], camera_pose[1], camera_pose[2])
                    cv2.putText(frame, str_position, (0, 20 + i * 20), font, 1, (0, 255, 0), 1, cv2.LINE_AA)

                    str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f" % (
                        math.degrees(camera_pose[3]), math.degrees(camera_pose[4]),
                        math.degrees(camera_pose[5]))
                    cv2.putText(frame, str_attitude, (0, 20 + (i + 1) * 20), font, 1, (0, 255, 0), 1, cv2.LINE_AA)
                i = 8
                camera_pose = np.average(res[1], axis=0)
                str_position = "CAMERA Avg Position x=%4.0f  y=%4.0f  z=%4.0f" % (
                    camera_pose[0], camera_pose[1], camera_pose[2])
                cv2.putText(frame, str_position, (0, 20 + i * 20), font, 1, (0, 0, 255), 1, cv2.LINE_AA)

                str_attitude = "CAMERA Avg Attitude r=%4.0f  p=%4.0f  y=%4.0f" % (
                    math.degrees(camera_pose[3]), math.degrees(camera_pose[4]),
                    math.degrees(camera_pose[5]))
                cv2.putText(frame, str_attitude, (0, 20 + (i + 1) * 20), font, 1, (0, 0, 255), 1, cv2.LINE_AA)
            # --- Display the frame
            cv2.imshow('frame', frame)

            key = cv2.waitKey(10)
            if key == ord('q') or key == ESC_KEY:
                break
            if key == ord('s') or key == ord('S'):
                cv2.imwrite(save_path.joinpath(f'{cnt}.jpg').absolute().as_posix(), color_image)
                cnt += 1
                print(f'Record data file {cnt} success!')
        except KeyboardInterrupt:
            break
    pipeline.stop()


if __name__ == "__main__":
    main()
