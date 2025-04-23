import numpy as np
from scipy.spatial.transform import Rotation as R
from abc import ABC, abstractmethod


class State(ABC):
    @abstractmethod
    def set(self):
        pass

    @abstractmethod
    def to_list(self):
        pass


class CartState(State):
    def __init__(self, position, orientation, use_quaternion=True):
        super().__init__()
        self.set(
            position=position, orientation=orientation, use_quaternion=use_quaternion
        )

    def euler_to_quaternion(self, euler_angles):
        r = R.from_euler("xyz", euler_angles, degrees=True)  # 假设欧拉角是以度为单位
        return r.as_quat()

    def __repr__(self):
        return f"EEFState(position={self.position}, orientation={self.orientation})"

    def to_list(self):
        return np.concatenate([self.position, self.orientation])

    def set(self, position, orientation, use_quaternion=True):
        self.position = np.array(position)

        if use_quaternion:
            # 使用四元数初始化
            self.orientation = np.array(orientation)
        else:
            # 使用欧拉角初始化，转换为四元数
            self.orientation = self.euler_to_quaternion(orientation)


class GripperState(State):
    def __init__(self, pos):
        super().__init__()
        self.set(pos)

    def set(self, pos):
        self.pos = np.array([pos])

    def to_list(self):
        return self.pos


class JointState(State):
    def __init__(self, joint_angles):
        super().__init__()
        self.set(joint_angles)

    def __repr__(self):
        return f"JointState(joint_angles={self.joint_angles})"

    def set(self, joint_angles):
        self.joint_angles = np.array(joint_angles)

    def to_list(self):
        return self.joint_angles


class RobotState(State):
    def __init__(self, arm_state, eef_state):
        super().__init__()
        self.set(arm_state=arm_state, eef_state=eef_state)

    def __repr__(self):
        return f"RobotState(arm_state={self.arm_state}, gripper_state={self.gripper_state})"

    def set(self, arm_state, eef_state):
        self.arm_state = arm_state
        self.eef_state = eef_state

    def to_list(self):
        return np.concatenate([self.arm_state.to_list(), self.eef_state.to_list()])
