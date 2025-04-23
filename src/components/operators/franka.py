from .operator import Operator
from src.utils.network import (
    ZMQKeypointSubscriber,
    create_response_socket,
    create_request_socket,
    ZMQKeypointPublisher,
)
from src.components.robot.franka_gripper import FrankaGripper
from src.utils.timer import FrequencyTimer, StationTimer, LogTimer
from src.data.eef_state import CartState, JointState, State
from src.utils.vectorops import *
from src.utils.retargeter import Coordinate, RotationRetFactory
from src.constants import *

from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Rotation as R


class FrankaOperator(Operator):
    def __init__(
        self,
        host,
        transformed_keypoints_port,
        operation_stage_port,
        action_port,
    ):
        self.notify_component_start("franka operator")
        self._transformed_arm_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host, port=transformed_keypoints_port, topic="transformed_hand_frame"
        )
        self._transformed_hand_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host, port=transformed_keypoints_port, topic="transformed_hand_coords"
        )
        self._operation_response_socket = create_response_socket(
            host, operation_stage_port
        )
        self.action_publisher = ZMQKeypointPublisher(host, action_port)

        self._robot = FrankaGripper()
        self._timer = FrequencyTimer(VR_FREQ)
        self.is_first_frame = True
        self.scale_factor = [1.5, 1.5, 2]
        self._P = [[0, -1, 0], [0, 0, 1], [1, 0, 0]]

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
    def _get_hand_frame_block(self):
        return self.transformed_arm_keypoint_subscriber.recv_keypoints()

    def get_gripper_state(self):
        transformed_hand_coords = (
            self._transformed_hand_keypoint_subscriber.recv_keypoints()
        )
        distance = np.linalg.norm(
            transformed_hand_coords[OCULUS_JOINTS["index"][-1]]
            - transformed_hand_coords[OCULUS_JOINTS["thumb"][-1]]
        )
        if distance > 0.06:
            return 1
        else:
            return 0

    # Reset the teleoperation
    def _reset_teleop(self):
        print("****** RESETTING TELEOP ****** ")
        self.robot.move(
            JointState(
                [
                    -4.29251469e-04,
                    -7.75564400e-01,
                    1.70081349e-03,
                    -2.35577936e00,
                    2.32602405e-04,
                    1.57109480e00,
                    7.85885839e-01,
                ]
            )
        )
        # wait for VR request
        self._operation_response_socket.recv()
        self._operation_response_socket.send_string(f"{0}")
        self._operation_response_socket.recv()
        self._operation_response_socket.send_string(f"{1}")
        self._operation_response_socket.recv()
        self._operation_response_socket.send_string(f"{2}")
        # close _operation_response_socket
        self._operation_response_socket.recv()
        self._operation_response_socket.close()

        self.init_hand_frame = self._get_hand_frame_block()
        o, x, y, z = self.init_hand_frame
        self.init_o = Coordinate(o, o + x, o + z, left_hand=True)

        robot_state = self.robot.get_cartesian_position()
        robot_pos = robot_state.position
        robot_rot = robot_state.orientation
        self.init_robot_o = Coordinate(
            robot_pos, robot_pos + np.array([0, 1, 0]), robot_pos + np.array([1, 0, 0])
        )

        rotation_mat = R.from_quat(robot_rot).as_matrix()
        self.rotation_ret = RotationRetFactory(rotation_mat, self._P)
        self.rotation_ret.reset(self.init_hand_frame)

        self.is_first_frame = False
        self.gripper_state = 1
        self.robot.move_gripper_percentage(1)

    # Apply retargeted angles
    def _apply_retargeted_angles(self):
        if self.is_first_frame:
            self._reset_teleop()
            return

        moving_hand_frame = self._get_hand_frame_block()
        cur_o, cur_x, cur_y, cur_z = moving_hand_frame

        v = np.array(cur_o) - self.init_o.o
        shift = (
            self.scale_factor
            * (v @ self.init_o.get_column_matrix())
            @ self.init_robot_o.get_row_matrix()
        )
        final_position = self.init_robot_o.o + shift

        final_rotation = self.rotation_ret.get_target(moving_hand_frame)
        final_rotation = Rotation.from_matrix(final_rotation).as_quat()

        if np.linalg.norm(moving_hand_frame[0]) < 1e-5:
            print(f"error {moving_hand_frame[0]}")
            return

        self.robot.move_coords(
            CartState(final_position, final_rotation), velocity_scale=1
        )
        # TODO:
        gripper_state = self.get_gripper_state()
        if gripper_state != self.gripper_state:
            self.robot.move_gripper_percentage(gripper_state, speed=1)
            self.gripper_state = gripper_state
        # # publish pose + gripper astate
        # self.action_publisher.pub_keypoints(
        #     final_pose + [gripper_state], topic_name="action"
        # )

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

        self._operation_response_socket.close()
        self.transformed_arm_keypoint_subscriber.stop()
        self.transformed_hand_keypoint_subscriber.stop()
        self.robot.stop()
        print("Stopping the teleoperator!")
