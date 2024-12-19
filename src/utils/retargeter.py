import numpy as np
from dataclasses import dataclass

from src.utils.vectorops import *


class Coordinate:
    def __init__(self, o, p, q):
        self.o = np.array(o)

        self.x = np.array(p) - self.o
        self.y = np.array(q) - self.o

        self.x_len = np.linalg.norm(self.x)
        self.y_len = np.linalg.norm(self.y)

        # normalize
        self.y = normalize_vector(self.y)
        self.x = normalize_vector(orthogonalization(self.y, self.x))
        # - for meta quest is left hand coordinate
        self.z = -np.cross(self.x, self.y)

    def get_column_matrix(self):
        # 获得列向量组成的矩阵
        return self.get_row_matrix.T

    def get_row_matrix(self):
        # 获得行向量组成的矩阵
        return np.array([self.x, self.y, self.z])


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
        self.source = Coordinate(s_o, s_p, s_q)

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

    def get_target(self, s_cur):
        s_cur = np.array(s_cur)
        v = s_cur - self.source.o
        final_position = self.scale_factor * (
            v @ self.source.get_column_matrix() @ self.target.get_row_matrix()
        )
        # final_position = (
        #     self.target.o
        #     + (self.target.x_len / self.source.x_len)
        #     * self.source.get_projection_x(s_cur)
        #     * self.target.x
        #     + (self.target.y_len / self.source.y_len)
        #     * self.source.get_projection_y(s_cur)
        #     * self.target.y
        #     + self.eta * self.source.get_projection_z(s_cur) * self.target.z
        # )
        return final_position


class RotationRet(Retargeter):
    def __init__(self, s_m, t_m, P):
        super().__init__()
        self.source = s_m
        self.target = t_m
        self.P = P

    def get_target(self, s_cur):
        return (
            self.target
            @ np.linalg.pinv(self.P)
            @ np.linalg.pinv(self.source)
            @ s_cur
            @ self.P
        )
