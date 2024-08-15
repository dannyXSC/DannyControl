from .operator import Operator
from src.utils.network import ZMQKeypointSubscriber
from src.components.robot.xarm import Xarm
from src.utils.timer import FrequencyTimer, StationTimer
from src.utils.vectorops import *
from src.constants import *

from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Rotation, Slerp
from scipy.spatial.transform import Rotation as R
import numpy as np
from copy import deepcopy as copy
import zmq


class XarmOperator(Operator):
    def __init__(self, host, transformed_keypoints_port, xarm_ip, log=False):
        self.notify_component_start("xarm operator")
        self.log = log
        self._transformed_arm_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host, port=transformed_keypoints_port, topic="transformed_hand_frame"
        )
        self._transformed_hand_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host, port=transformed_keypoints_port, topic="transformed_hand_coords"
        )

        self._robot = Xarm(xarm_ip)
        # Frequency timer
        self._timer = FrequencyTimer(VR_FREQ)
        self._station_timer = StationTimer(1, 0.1)

        self.scale_matrix = np.diag([50, 50, 50, 1])

        self.is_first_frame = True

    @property
    def timer(self):
        return self._timer

    @property
    def robot(self):
        return self._robot

    @property
    def transformed_arm_keypoint_subscriber(self):
        return self._transformed_arm_keypoint_subscriber

    @property
    def transformed_hand_keypoint_subscriber(self):
        return self._transformed_hand_keypoint_subscriber

    # Get the hand frame
    def _get_hand_frame(self):
        for i in range(10):
            data = self.transformed_arm_keypoint_subscriber.recv_keypoints(
                flags=zmq.NOBLOCK
            )
            if not data is None:
                break
        if data is None:
            return None
        return np.asanyarray(data).reshape(4, 3)

        # Converts a frame to a homogenous transformation matrix

    def _turn_frame_to_homo_mat(self, frame):
        t = frame[0]
        R = frame[1:]

        homo_mat = np.zeros((4, 4))
        homo_mat[:3, :3] = np.transpose(R)
        homo_mat[:3, 3] = t
        homo_mat[3, 3] = 1

        return homo_mat

        # Convert Homogenous matrix to cartesian vector

    def _homo2cart(self, homo_mat):
        t = homo_mat[:3, 3]
        R = Rotation.from_matrix(homo_mat[:3, :3]).as_euler("xyz", degrees=True)

        cart = np.concatenate([t, R], axis=0)

        return cart

    # Reset the teleoperation
    def _reset_teleop(self):
        print("****** RESETTING TELEOP ****** ")
        self.robot_init_H = self.robot.get_rotation_matrix()

        first_hand_frame = self._get_hand_frame()
        while first_hand_frame is None:
            first_hand_frame = self._get_hand_frame()
        self.hand_init_origin = first_hand_frame[0]
        self.hand_init_H = self._turn_frame_to_homo_mat(first_hand_frame)
        self.hand_init_t = copy(self.hand_init_H[:3, 3])
        self.is_first_frame = False

        return first_hand_frame

    # Reset the teleoperation
    # def _reset_teleop(self):
    #     print("****** RESETTING TELEOP ****** ")
    #     self.robot_init_H = self.robot.get_rotation_matrix()
    #     self.robot_init_cart = self.robot.get_cartesian_position()

    #     anchors = []
    #     anchors_counts = 0
    #     pre_state = False

    #     while True:
    #         frame = self._get_hand_frame()
    #         while frame is None:
    #             frame = self._get_hand_frame()
    #         origin = frame[0]
    #         state = self._station_timer.trigger(origin)
    #         if pre_state == False and state == True:
    #             anchors.append(self._station_timer.get_state())
    #             anchors_counts += 1
    #             print(f"anchor {anchors_counts} set complete!")

    #             if anchors_counts >= 3:
    #                 break
    #         pre_state = state
    #     print(anchors)
    #     self.hand_init_origin = anchors[0]
    #     x = normalize_vector(anchors[1] - anchors[0])
    #     y = normalize_vector(anchors[2] - anchors[0])
    #     z = np.cross(x, y)
    #     self.hand_init_matrix = np.transpose([x, y, z])
    #     self.is_first_frame = False

    # Apply retargeted angles
    def _apply_retargeted_angles(self):
        if self.is_first_frame:
            self._reset_teleop()
            return

        moving_hand_frame = self._get_hand_frame()
        if moving_hand_frame is None:
            return  # It means we are not on the arm mode yet instead of blocking it is directly returning
        # cur_origin = moving_hand_frame[0]

        # m_rotation = np.identity(3)
        # v_transition = (
        #     (cur_origin - self.hand_init_origin) @ self.hand_init_matrix * 100
        # )
        # print(v_transition)
        # # m_homo = np.block(
        # #     [[m_rotation, v_transition.reshape(3, 1)], [np.array([0, 0, 0]), 1]]
        # # )
        # final_pose = list(v_transition + np.array([466.6, 72.2, 297.8])) + [
        #     178.10439,
        #     1.829454,
        #     3.611926,
        # ]
        # self.robot.move_coords(final_pose)

        # hand current transform matrix
        self.hand_moving_H = self._turn_frame_to_homo_mat(moving_hand_frame)
        H_R_V = [[0, -1, 0, 0], [0, 0, 1, 0], [1, 0, 0, 0], [0, 0, 0, 1]]

        m_transition = (
            np.linalg.pinv(H_R_V)
            @ np.linalg.pinv(self.hand_init_H)
            @ self.hand_moving_H
            @ H_R_V
        )

        m_transition[:3, 3] *= 300

        final_pose = self.robot_init_H @ m_transition

        self.robot.move_matrix(final_pose, speed=300)

        # # hand current transform matrix
        # self.hand_moving_origin = moving_hand_frame[0]
        # self.hand_moving_matrix = np.transpose(moving_hand_frame[1:])

        # m_transition = np.linalg.pinv(self.hand_init_matrix) @ self.hand_moving_matrix
        # v_transform = (
        #     (self.hand_moving_origin - self.hand_init_origin)
        #     @ self.hand_init_matrix
        #     * 100
        # )
        # m_homo = np.block(
        #     [[m_transition, v_transform.reshape(3, 1)], [np.array([0, 0, 0]), 1]]
        # )

        # final_pose = self.robot_init_H @ m_homo

        # self.robot.move_matrix(final_pose)

    # NOTE: This is for debugging should remove this when needed
    def stream(self):
        self.notify_component_start("{} control".format(self.robot.name))
        print("Start controlling the robot hand using the Oculus Headset.\n")

        # Assume that the initial position is considered initial after 3 seconds of the start
        while True:
            try:
                if not self.robot.if_shutdown():
                    self.timer.start_loop()

                    # Retargeting function
                    self._apply_retargeted_angles()

                    self.timer.end_loop()
            except KeyboardInterrupt:
                break

        self.transformed_arm_keypoint_subscriber.stop()
        self.transformed_hand_keypoint_subscriber.stop()
        self.robot.stop()
        print("Stopping the teleoperator!")
