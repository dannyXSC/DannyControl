from .operator import Operator
from src.utils.network import ZMQKeypointSubscriber, create_response_socket
from src.components.robot.xarm import Xarm
from src.utils.timer import FrequencyTimer, StationTimer, LogTimer
from src.utils.vectorops import *
from src.constants import *

from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Rotation, Slerp
from scipy.spatial.transform import Rotation as R
import numpy as np
from copy import deepcopy as copy
import zmq
import time

XARM_ANCHOR_O_VALUES = [366, -39.1, 0, 180, 0, 90]
XARM_ANCHOR_P1_VALUES = [374, 158, 0, 180, 0, 90]
XARM_ANCHOR_P2_VALUES = [638, -36.2, 0, 180, 0, 90]


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
        operation_stage_port,
        xarm_ip,
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
        self._operation_response_socket = create_response_socket(
            host, operation_stage_port
        )

        self._robot = Xarm(xarm_ip)
        # Frequency timer
        self._timer = FrequencyTimer(VR_FREQ)
        self._station_timer = StationTimer(2, 0.01)
        self._log_timer = LogTimer(1)

        self._P = [[1, 0, 0], [0, 0, 1], [0, 1, 0]]

        # filter ratio
        self.comp_ratio = comp_ratio

        self.is_first_frame = True

        if self.log:
            self.pre_time = time.time()

        self.robot_o = np.array(XARM_ANCHOR_O_VALUES[:3])
        self.robot_y = np.array(XARM_ANCHOR_P1_VALUES[:3]) - self.robot_o
        self.robot_x = np.array(XARM_ANCHOR_P2_VALUES[:3]) - self.robot_o
        # orthogonalization
        self.robot_x = self._orthogonalization(self.robot_y, self.robot_x)
        self.robot_z = normalize_vector(np.cross(self.robot_x, self.robot_y))

        self.robot_init_rotation = R.from_euler(
            "xyz", XARM_ANCHOR_O_VALUES[3:], degrees=True
        ).as_matrix()

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

    # Get the hand frame
    def _get_hand_frame_block(self):
        return self.transformed_arm_keypoint_subscriber.recv_keypoints()

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

    def _turn_frame_to_rotation_mat(self, frame):
        R = frame[1:]

        return np.transpose(R)

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

    def _orthogonalization(self, v1, v2):
        # orthogonalization v2 base on v1
        return v2 - np.dot(v1, v2) * v1 / np.dot(v1, v1)

    # Reset the teleoperation
    def _reset_teleop(self):
        print("****** RESETTING TELEOP ****** ")
        self.robot.move_coords(XARM_ANCHOR_O_VALUES)
        # robot_init_cart = self._homo2cart(self.robot_init_H)
        # self.comp_filter = Filter(robot_init_cart, comp_ratio=self.comp_ratio)

        anchors_counts = 0
        anchors = []
        pre_state = False
        # self.hand_init_origin = first_hand_frame[0]
        # self.hand_init_H = self._turn_frame_to_homo_mat(first_hand_frame)
        # self.hand_init_t = copy(self.hand_init_H[:3, 3])

        # wait for VR request
        self._operation_response_socket.recv()
        print("received! ")
        self._operation_response_socket.send_string(f"{anchors_counts}")

        # for three anchor
        while anchors_counts < 3:
            # hand_frame = self._get_hand_frame()
            # while hand_frame is None:
            #     hand_frame = self._get_hand_frame()
            hand_frame = self._get_hand_frame_block()
            origin = hand_frame[0]

            state = self._station_timer.trigger(origin)
            if pre_state == False and state == True:
                anchors.append(self._station_timer.get_state())
                anchors_counts += 1
                print(anchors_counts, anchors)

                # send stage infomation
                self._operation_response_socket.recv()
                self._operation_response_socket.send_string(f"{anchors_counts}")

                # init rotation matrix
                # last hand position
                if anchors_counts == 3:
                    self.hand_init_rotation = self._turn_frame_to_rotation_mat(
                        hand_frame
                    )
                    # close _operation_response_socket
                    self._operation_response_socket.recv()
                    self._operation_response_socket.close()

            self._log_timer.trigger(
                (
                    f"anchors_counts: {anchors_counts}\n"
                    f"pre_state: {pre_state}\n"
                    f"state: {state}\n"
                )
            )
            pre_state = state
        self.hand_origin = np.array(anchors[2])
        self.hand_y = np.array(anchors[0]) - self.hand_origin
        self.hand_x = np.array(anchors[1]) - self.hand_origin
        self.hand_length_y = np.linalg.norm(self.hand_y)
        self.hand_length_x = np.linalg.norm(self.hand_x)
        self.hand_y_normalized = normalize_vector(self.hand_y)
        self.hand_x_normalized = normalize_vector(
            self._orthogonalization(self.hand_y, self.hand_x)
        )
        # 左手系，所以是相反的
        self.hand_z_normalized = -np.cross(
            self.hand_x_normalized, self.hand_y_normalized
        )

        self.is_first_frame = False

        # gripper settings
        # 1 for open
        self.gripper_state = 1
        self.robot.move_gripper_percentage(1)

    # Apply retargeted angles
    def _apply_retargeted_angles(self):
        if self.is_first_frame:
            self._reset_teleop()
            return

        # moving_hand_frame = self._get_hand_frame()
        # while moving_hand_frame is None:
        #     moving_hand_frame = self._get_hand_frame()
        moving_hand_frame = self._get_hand_frame_block()

        # hand current transform matrix
        self.hand_cur_rotation = self._turn_frame_to_rotation_mat(moving_hand_frame)

        m_transition = (
            np.linalg.pinv(self._P)
            @ np.linalg.pinv(self.hand_init_rotation)
            @ self.hand_cur_rotation
            @ self._P
        )

        # robot arm space move
        final_rotation_matrix = self.robot_init_rotation @ m_transition

        final_rotation = Rotation.from_matrix(final_rotation_matrix).as_euler(
            "xyz", degrees=True
        )

        # pose_cart = self._homo2cart(final_pose)
        # final_pose_cart = self.comp_filter(pose_cart)

        # TODO: 简化
        origin = moving_hand_frame[0]
        final_position = (
            self.robot_o
            + np.dot(origin - self.hand_origin, self.hand_x_normalized)
            / self.hand_length_x
            * self.robot_x
            + np.dot(origin - self.hand_origin, self.hand_y_normalized)
            / self.hand_length_y
            * self.robot_y
            + np.dot(origin - self.hand_origin, self.hand_z_normalized)
            * 1100
            * self.robot_z
        )
        final_pose = [*final_position] + [*final_rotation]

        if np.linalg.norm(origin) < 1e-5:
            print(f"error {origin}")
            return

        # print(f"origin: {origin}")
        # print(f"final_pose: {final_pose}")
        # with open("./log.txt", "a") as f:
        #     f.write(f"origin: {origin}\nfinal_pose: {final_pose}\n")

        self.robot.move_coords(final_pose, speed=1000)

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
