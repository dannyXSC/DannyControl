from .robot import RobotWrapper
from src.constants import *
from xarm.wrapper import XArmAPI

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Rotation, Slerp
from scipy.spatial.transform import Rotation as R


XARM_HOME_VALUES = [316.79242, 20.13295, 40.77933, 178.10439, 1.829454, 3.611926]


class Xarm(RobotWrapper):
    def __init__(self, ip):
        # is_radian: 设置用角度
        self._controller = XArmAPI(ip, is_radian=False)
        self._init_controller()

        self._data_frequency = TRANS_FREQ

    def _init_controller(self):
        # error and warn call back
        self._controller.register_error_warn_changed_callback(
            self.hangle_err_warn_changed
        )
        self._controller.connect()
        # enable motion
        self._controller.motion_enable(enable=True)
        # set mode: position control mode
        self._controller.set_mode(0)
        # set state: sport state
        self._controller.set_state(state=0)

    @property
    def name(self):
        return "xarm"

    @property
    def recorder_functions(self):
        return {
            "joint_states": self.get_joint_state,
            "cartesian_position": self.get_cartesian_position,
        }

    @property
    def data_frequency(self):
        return self._data_frequency

    def if_shutdown(self):
        code, state = self._controller.get_state()
        return code != 0

    def get_joint_state(self) -> np.ndarray:
        # (code, [position, velocity, effort])
        return np.array(self._controller.get_joint_states()[1], dtype=np.float32)

    def get_joint_position(self) -> np.ndarray:
        # TODO: 暂时用不到关节的位置
        pass
        # return np.array(self._controller.get_position()[1], dtype=np.float32)

    def get_cartesian_position(self) -> np.ndarray:
        return np.array(self._controller.get_position()[1], dtype=np.float32)

    # 齐次坐标系下的变换矩阵
    def get_rotation_matrix(self) -> np.ndarray:
        # [x,y,z,roll,pitch,yaw]
        cart = self.get_cartesian_position()

        # 外旋，绕固定坐标轴转动
        rotation = R.from_euler("xyz", cart[3:], degrees=True).as_matrix()
        translation = np.array(cart[:3])
        return np.block([[rotation, translation[:, np.newaxis]], [0, 0, 0, 1]])

    def get_joint_velocity(self) -> np.ndarray:
        return np.array(self._controller.get_joint_states()[1][1], dtype=np.float32)

    def get_joint_torque(self) -> np.ndarray:
        return np.array(self._controller.get_joints_torque()[1], dtype=np.float32)

    def home(self):
        self.move_coords(XARM_HOME_VALUES)

    def move(self, input_angles):
        self._controller.set_servo_angle(angle=input_angles)

    def move_coords(self, input_coords, speed=100, wait=True):
        # input_coords: [x,y,z,roll,pitch,yaw]
        x, y, z, roll, pitch, yaw = input_coords[:6]
        # TODO: 是否要阻塞
        self._controller.set_position(
            x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, wait=wait, speed=speed
        )

    def move_matrix(self, input_matrix, speed=100):
        t = input_matrix[:3, 3]
        R = Rotation.from_matrix(input_matrix[:3, :3]).as_euler("xyz", degrees=True)
        cart = np.concatenate([t, R], axis=0)

        self.move_coords(cart, speed=speed)

    def stop(self):
        self._controller.disconnect()

    def hangle_err_warn_changed(self, item):
        print(
            "Xarm ErrorCode: {}, WarnCode: {}".format(
                item["error_code"], item["warn_code"]
            )
        )
        # TODO：Do different processing according to the error code
