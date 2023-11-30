import sys
import os
import platform
from pathlib import Path

cur_path = Path(os.path.realpath(__file__))
sdk_path = cur_path.parent.parent.joinpath('sdk')
if 'Linux' in platform.platform():
    fr_sdk_path = sdk_path.joinpath('frcobot_python_sdk-main').joinpath('linux').joinpath('lib_x86_64-linux-gnu')
else:
    fr_sdk_path = sdk_path.joinpath('frcobot_python_sdk-main').joinpath('windows').joinpath('lib64')

sys.path.append(fr_sdk_path.absolute().as_posix())

try:
    import frrpc
except Exception as e:
    print('fr_sdk_path: ', fr_sdk_path.absolute().as_posix())
    print(e)
    raise e

import functools
from typing import List
from .CommonParam import *
from .ArmRobot import ArmRobot


class FRArmRobot(ArmRobot):

    def __init__(self, ip):
        super().__init__(ip)
        self.ip = ip
        self.robot = frrpc.RPC(ip)

    def execution(func):
        @functools.wraps(func)
        def wrapper(self, *args, **kwargs):
            response = func(self, *args, **kwargs)
            if isinstance(response, List):
                code = response[0]
            else:
                code = response
            assert code == 0, "Error: {}, Error Code: {}, Response: {}.".format(func.__name__, code, response)
            return response

        return wrapper

    @execution
    def get_error_code(self):
        """
        获取机器人状态错误码
        :return: 0： 成功；否则失败
        """
        return self.robot.GetRobotErrorCode()

    @execution
    def clear_error(self) -> int:
        """
        错误状态清除，只能清除可复位的错误
        :return: 0： 成功；否则失败
        """
        return self.robot.ResetAllError()

    @execution
    def set_anti_collision_level(self, mode: CollisionMode, level: List[float], update_config: YesOrNo) -> int:
        """设置碰撞等级

        Args:
            mode (int): 0: 等级模式; 1: 百分比模式
            level (List[float]): 等级模式下，1级最高；百分比模式下，取值范围(0, 100]
            update_config (int): 0: 不更新配置文件; 1: 更新配置文件

        Returns:
            int: 0： 成功；否则失败
        """
        return self.robot.SetAnticollision(mode.value, level, update_config.value)

    @execution
    def set_collision_strategy(self, strategy: CollisionStrategy) -> int:
        """碰撞后策略

        Args:
            strategy (CollisionStrategy): 0: 碰撞后停止并报错； 1: 继续运行；

        Returns:
            int: 0： 成功；否则失败
        """
        return self.robot.SetCollisionStrategy(strategy.value)

    @execution
    def set_limit_positive(self, joint_angle: List[float]) -> int:
        """设置关节正向运转的最大角度

        Args:
            jointAngle 关节角度

        Returns:
            int: 0： 成功；否则失败
        """
        assert len(joint_angle) == 6, "Error: Function of setLimitPositive's parameter length should be six."
        return self.robot.SetLimitPositive(joint_angle)

    @execution
    def set_limit_negative(self, joint_angle: List[float]) -> int:
        """设置关节反向运转的最大角度

        Args:
            jointAngle 关节角度

        Returns:
            int: 0： 成功；否则失败
        """
        assert len(joint_angle) == 6, "Error: Function of setLimitPositive's parameter length should be six."
        return self.robot.SetLimitNegative(joint_angle)

    @execution
    def get_default_speed(self) -> float:
        """
        获取默认的运行速度 mm/s
        :return: 运行速度
        """
        return self.robot.GetDefaultTransVel()

    @execution
    def get_cur_cartesian_coordinate(self, block_mode: BlockMode) -> List[float]:
        """
        获取当前笛卡尔坐标系下位姿参数
        :param block_mode: 默认为非阻塞状态
        :return: [status_code, x, y, z, rx, ry, rz]
        """
        return self.robot.GetActualTCPPose(block_mode.value)

    #################################### 坐标查询 ######################################
    @execution
    def get_cur_joint_angle(self, block_mode: BlockMode) -> List[float]:
        """
        查询当前各个关节的角度
        :param block_mode: 是否为阻塞模式
        :return: [status_code, j1, j2, j3, j4, j5, j6]
        """
        return self.robot.GetActualJointPosDegree(block_mode.value)

    #################################### 坐标计算 ######################################
    @execution
    def get_inverse_kin_by_target_cartesian_coordinate(self, target_cartesian_coordinate: List[float],
                                                       coordinate_rel_type: CoordinateRelType = CoordinateRelType.AbsoluteBaseCoordinate,
                                                       ref_coordinate: int = -1) -> List[float]:
        """
        根据笛卡尔坐标系下的坐标，计算关节的角度值
        :param coordinate_rel_type: 参考坐标系
        :param target_cartesian_coordinate: 目标位置，笛卡尔坐标系描述 [x, y, z, rx, ry, rz]
        :param ref_coordinate: 参考的关节，默认-1（参考当前关节求解），[0, 7] 依据关节配置求解
        :return: [status_code, j1, j2, j3, j4, j5, j6]
        """
        assert len(target_cartesian_coordinate) == 6
        return self.robot.GetInverseKin(coordinate_rel_type.value, target_cartesian_coordinate, ref_coordinate)

    @execution
    def get_forward_kin_by_target_joint_angle(self, target_joint_angle):
        """
        目标关节角度
        :param target_joint_angle: [j1, j2, j3, j4, j5, j6]， 单位：°
        :return: 计算出的目标笛卡尔坐标 [x, y, z, rx, ry, rz]
        """
        assert len(target_joint_angle) == 6
        return self.robot.GetForwardKin(target_joint_angle)

    #################################### 执行动作 ######################################
    @execution
    def move_by_joint_target_angle(self, target_joint_angle, target_cartesian_coordinate, tool=0, user=0,
                                   vel=20.0, acc=0.0, ovl=100.0,
                                   exaxis_pos=[0.0, 0.0, 0.0, 0.0], blend_time=-1.0, offset_flag=0,
                                   offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
        """
        关节空间运动
        :param target_joint_angle: 六个关节的分别目标角度，单位：°
        :param target_cartesian_coordinate: 目标笛卡尔位姿，
        :param tool: 工具号
        :param user: 工件号
        :param vel: 速度
        :param acc: 加速度
        :param ovl: 速度缩放因子
        :param exaxis_pos: 外部轴 1 位置 ~ 外部轴 4 位置 默认[0.0,0.0,0.0,0.0];
        :param blend_time: 运动平滑时间，默认-1（运动到位立即停止），单位：ms
        :param offset_flag: [0]-不偏移，[1]-工件/基坐标系下偏移，[2]-工具坐标系下偏移 默认 0;
        :param offset_pos: 位姿偏移量，单位 [mm][°] 默认[0.0,0.0,0.0,0.0,0.0,0.0];
        :return:
        """
        assert len(target_joint_angle) == 6
        assert len(target_cartesian_coordinate) == 6
        return self.robot.MoveJ(target_joint_angle, target_cartesian_coordinate, tool, user, vel, acc, ovl, exaxis_pos,
                                blend_time, offset_flag, offset_pos)

    @execution
    def move_by_cartesian_coordinate_p2p(self, target_coordinate: List[float], tool_num=0, user_num=0, velocity=20.0,
                                         acc=0.0, ovl=100.0, blend_time=-1.0, config=-1):
        """
        笛卡尔空间点到点运动
        :param target_coordinate: 目标位姿
        :param tool_num: 工具号
        :param user_num: 工件号
        :param velocity: 速度百分比
        :param acc: 加速度，不开放
        :param ovl: 速度缩放因子
        :param blend_time: 平滑时间，单位：ms， -1.0： 运动到位阻塞； [0, 500]
        :param config: 关节配置， -1： 参考当前关节求解， [0, 7]: 根据关节配置求解
        :return: 0： 成功；否则失败
        """
        return self.robot.MoveCart(target_coordinate, tool_num, user_num, velocity, acc, ovl, blend_time, config)

    @execution
    def move_by_cartesian_coordinate_line(self, joint_pos: List[float],
                                          target_coordinate: List[float],
                                          tool_num=0,
                                          user_num=0,
                                          velocity=20.0,
                                          acc=0.0, ovl=100.0,
                                          blend_radius=-1.0,
                                          ex_axis_pos: List[float] = [0.0] * 4,
                                          search: int = 0,
                                          offset_flag: int = 0,
                                          offset_pos: List[float] = [0.0] * 6
                                          ):
        """
        笛卡尔空间点到点运动
        :param joint_pos: 目标关节位置 单位：°
        :param target_coordinate: 目标位姿, 单位: mm/°
        :param tool_num: 工具号
        :param user_num: 工件号
        :param velocity: 速度百分比
        :param acc: 加速度，不开放
        :param ovl: 速度缩放因子
        :param blend_radius: 平滑半径，单位：mm， -1.0： 运动到位阻塞； [0, 1000]
        :param ex_axis_pos: 四个外部轴位置
        :param search: 0: 不焊丝寻位； 1：焊丝寻位
        :param offset_flag: 0：不偏移；
        :param offset_pos: 位姿偏移量，单位：mm/°
        :return: 0： 成功；否则失败
        """
        return self.robot.MoveL(joint_pos, target_coordinate, tool_num, user_num, velocity, acc, ovl, blend_radius,
                                ex_axis_pos,
                                search, offset_flag, offset_pos)

    #################################### 参数设置 ######################################
    @execution
    def set_speed(self, speed: float) -> int:
        """
        设置全局运行速度
        :param speed: 百分比 [0, 100]
        :return: 0： 成功；否则失败
        """
        assert 0 <= speed <= 100, f"{speed}: Speed Value is invalid, the correct value range is [0, 100]."
        return self.robot.SetSpeed(speed)

    @execution
    def set_mode(self, mode: Mode):
        """
        设置机器人的模式
        :param mode: 自动 / 手动
        :return: 0： 成功；否则失败
        """
        return self.robot.Mode(mode.value)

    @execution
    def set_robot_enable(self):
        """
        设置机器人上使能
        :return:
        """
        return self.robot.RobotEnable(1)

    @execution
    def set_robot_disable(self):
        """
        设置机器人下使能
        :return:
        """
        return self.robot.RobotEnable(0)
