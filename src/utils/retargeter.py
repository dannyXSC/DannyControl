import numpy as np
from dataclasses import dataclass
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Rotation as R

from src.utils.vectorops import *
from src.utils.timer import StationTimer


class Coordinate:
    def __init__(self, o, p, q, left_hand=False):
        self.o = np.array(o)

        self.x = np.array(p) - self.o
        self.y = np.array(q) - self.o

        self.x_len = np.linalg.norm(self.x)
        self.y_len = np.linalg.norm(self.y)

        # normalize
        self.y = normalize_vector(self.y)
        self.x = normalize_vector(orthogonalization(self.y, self.x))
        # - for meta quest is left hand coordinate
        if left_hand:
            self.z = -np.cross(self.x, self.y)
        else:
            self.z = np.cross(self.x, self.y)

    def get_column_matrix(self):
        # 获得列向量组成的矩阵
        return self.get_row_matrix().T

    def get_row_matrix(self):
        # 获得行向量组成的矩阵
        return np.array([self.x, self.y, self.z])

    def to_string(self):
        return (
            f"o:{self.o}\n"
            + f"x:{self.x}\n"
            + f"y:{self.y}\n"
            + f"z:{self.z}\n"
            + f"x_len:{self.x_len}\n"
            + f"y_len:{self.y_len}\n"
        )


class Retargeter:
    def __init__(self):
        pass

    def get_target(self, args):
        pass


class ConstantRet(Retargeter):
    def __init__(self):
        super().__init__()

    def get_target(self, args):
        pass


class PositionRet(Retargeter):

    def __init__(self, s_o, s_p, s_q, t_o, t_p, t_q, eta):
        super().__init__()
        # source coordinate
        self.source = Coordinate(s_o, s_p, s_q, left_hand=True)

        # target coordinate
        self.target = Coordinate(t_o, t_p, t_q)

        # scale factor for z
        # self.eta = eta
        # scale factor for x, y, z
        self.scale_factor = np.array(
            [
                self.target.x_len / self.source.x_len,
                self.target.y_len / self.source.y_len,
                eta,
            ]
        )
        print(self.scale_factor)

    def to_string(self):
        return (
            "-----source:-----\n"
            + self.source.to_string()
            + "\n"
            + "-----target:-----\n"
            + self.target.to_string()
        )

    def get_target(self, s_cur_pos):
        s_cur_pos = np.array(s_cur_pos)
        v = s_cur_pos - self.source.o

        shift = (
            self.scale_factor * (v @ self.source.get_column_matrix())
        ) @ self.target.get_row_matrix()
        final_position = self.target.o + shift
        # print(f"shift: {shift}")
        return final_position


class RotationRet(Retargeter):
    def __init__(self, s_m, t_m, P):
        super().__init__()
        self.source = s_m
        self.target = t_m
        self.P = P

    def get_target(self, s_cur_mat):
        return (
            self.target
            @ np.linalg.pinv(self.P)
            @ np.linalg.pinv(self.source)
            @ s_cur_mat
            @ self.P
        )


class RetargeterFactory:
    def __init__(self):
        pass

    def reset(self, hand_frame):
        pass

    def reset_completed(self):
        pass

    def get_target(self, args):
        pass


class PositionRetFactory(RetargeterFactory):
    def __init__(self, robot_o, robot_p1, robot_p2, eta):
        super().__init__()
        self._init_completed = None
        self._station_timer = StationTimer(2, 0.01)
        self.robot_o = robot_o
        self.robot_p1 = robot_p1
        self.robot_p2 = robot_p2
        self.eta = eta

        self.init()

    def init(self):
        self.pre_state = False
        self.anchors = []
        self.anchors_counts = 0
        self.position_retargeter = None
        pass

    def assign(self):
        self.position_retargeter = PositionRet(
            self.anchors[2],
            self.anchors[1],
            self.anchors[0],
            self.robot_o,
            self.robot_p2,
            self.robot_p1,
            self.eta,
        )

    def reset(self, hand_frame):
        if self._init_completed is None or self._init_completed == True:
            self._init_completed = False
            self.init()

        origin = hand_frame[0]
        state = self._station_timer.trigger(origin)
        if self.pre_state == False and state == True:
            self.anchors.append(self._station_timer.get_state())
            self.anchors_counts += 1
        self.pre_state = state
        if self.anchors_counts >= 3:
            # complete!
            self._init_completed = True
            # assign value
            self.assign()
        return self.anchors_counts

    def reset_completed(self):
        return self._init_completed is not None and self._init_completed == True

    def get_target(self, s_cur):
        if self.reset_completed():
            return self.position_retargeter.get_target(s_cur)
        # error
        print("PositionRetFactory error")


class RotationRetFactory(RetargeterFactory):

    def __init__(self, robot_mat, P):
        super().__init__()
        self.robot_mat = robot_mat
        self.P = P
        self.rotation_retargeter = None

    def reset(self, hand_frame):
        hand_mat = frame_to_mat3(hand_frame)
        self.rotation_retargeter = RotationRet(hand_mat, self.robot_mat, self.P)
        return True

    def reset_completed(self):
        return self.rotation_retargeter is not None

    def get_target(self, s_cur):
        if self.reset_completed():
            return self.rotation_retargeter.get_target(s_cur)
        # error
        print("RotationRetFactory error")


class AnchorBasedRetFactory(RetargeterFactory):
    def __init__(self, o, p1, p2, eta, euler_angles, P):
        super().__init__()
        self.position_retargeter = PositionRetFactory(
            robot_o=o, robot_p1=p1, robot_p2=p2, eta=eta
        )
        rotation_mat = Rotation.from_euler(
            "xyz", euler_angles, degrees=True
        ).as_matrix()
        self.rotation_retargeter = RotationRetFactory(robot_mat=rotation_mat, P=P)

    def reset(self, hand_frame):
        anchors_counts = self.position_retargeter.reset(hand_frame)
        if self.position_retargeter.reset_completed():
            self.rotation_retargeter.reset(hand_frame)
        return anchors_counts

    def reset_completed(self):
        return (
            self.position_retargeter.reset_completed()
            and self.rotation_retargeter.reset_completed()
        )

    def get_target(self, hand_frame):
        origin = hand_frame[0]
        rotation_mat = frame_to_mat3(hand_frame)
        # x, y, z
        position = self.position_retargeter.get_target(origin)
        # mat
        rotation_mat = self.rotation_retargeter.get_target(rotation_mat)
        rotation = Rotation.from_matrix(rotation_mat).as_euler("xyz", degrees=True)
        return dict(position=position, rotation=rotation)
