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
import time


# Rotation should be filtered when it's being sent
class Filter:
    def __init__(self, state, comp_ratio=0.6):
        self.pos_state = state[:3]
        self.ori_state = state[3:6]
        self.comp_ratio = comp_ratio

    def __call__(self, next_state):
        self.pos_state = self.pos_state[:3] * self.comp_ratio + next_state[:3] * (
            1 - self.comp_ratio
        )
        ori_interp = Slerp(
            [0, 1],
            Rotation.from_euler(
                "xyz", np.stack([self.ori_state, next_state[3:6]], axis=0), degrees=True
            ),
        )
        self.ori_state = ori_interp([1 - self.comp_ratio])[0].as_euler(
            "xyz", degrees=True
        )
        return np.concatenate([self.pos_state, self.ori_state])


class XarmOperator(Operator):

    def __init__(
        self,
        host,
        transformed_keypoints_port,
        xarm_ip,
        scale_vector=np.full((3,), 300),
        comp_ratio=0,
        log=False,
    ):
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

        self._scale_vector = np.array(scale_vector)
        self._P = [[0, -1, 0, 0], [0, 0, 1, 0], [1, 0, 0, 0], [0, 0, 0, 1]]

        # filter ratio
        self.comp_ratio = comp_ratio

        self.is_first_frame = True

        if self.log:
            self.pre_time = time.time()

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

    def get_gripper_state(self):
        transformed_hand_coords = (
            self._transformed_hand_keypoint_subscriber.recv_keypoints()
        )
        distance = np.linalg.norm(
            transformed_hand_coords[OCULUS_JOINTS["index"][-1]]
            - transformed_hand_coords[OCULUS_JOINTS["thumb"][-1]]
        )
        if distance > 0.05:
            return 1
        else:
            return 0

    # Reset the teleoperation
    def _reset_teleop(self):
        print("****** RESETTING TELEOP ****** ")
        self.robot.home()

        self.robot_init_H = self.robot.get_rotation_matrix()
        robot_init_cart = self._homo2cart(self.robot_init_H)
        self.comp_filter = Filter(robot_init_cart, comp_ratio=self.comp_ratio)

        first_hand_frame = self._get_hand_frame()
        while first_hand_frame is None:
            first_hand_frame = self._get_hand_frame()
        self.hand_init_origin = first_hand_frame[0]
        self.hand_init_H = self._turn_frame_to_homo_mat(first_hand_frame)
        self.hand_init_t = copy(self.hand_init_H[:3, 3])
        self.is_first_frame = False

        # gripper settings
        # 1 for open
        self.gripper_state = 1
        self.robot.move_gripper_percentage(1)
        # abcs
        return first_hand_frame

    # Apply retargeted angles
    def _apply_retargeted_angles(self):
        if self.is_first_frame:
            self._reset_teleop()
            return

        moving_hand_frame = self._get_hand_frame()
        if moving_hand_frame is None:
            return  # It means we are not on the arm mode yet instead of blocking it is directly returning

        # hand current transform matrix
        self.hand_moving_H = self._turn_frame_to_homo_mat(moving_hand_frame)

        m_transition = (
            np.linalg.pinv(self._P)
            @ np.linalg.pinv(self.hand_init_H)
            @ self.hand_moving_H
            @ self._P
        )

        # transform scale
        m_transition[:3, 3] *= self._scale_vector

        # robot arm space move
        final_pose = self.robot_init_H @ m_transition

        pose_cart = self._homo2cart(final_pose)
        final_pose_cart = self.comp_filter(pose_cart)

        if self.log:
            cur_time = time.time()
            if cur_time > self.pre_time + 1:
                print(f"pose_cart      : {pose_cart}")
                print(f"final_pose_cart: {final_pose_cart}")
                self.pre_time = cur_time

        self.robot.move_coords(final_pose_cart, speed=300)

        gripper_state = self.get_gripper_state()
        if gripper_state != self.gripper_state:
            self.robot.move_gripper_percentage(
                gripper_state, wait=False, wait_motion=False
            )
            self.gripper_state = gripper_state

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
