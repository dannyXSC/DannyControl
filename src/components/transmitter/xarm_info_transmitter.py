from src.components import Component
from src.utils.timer import FrequencyTimer, LogTimer
from src.constants import *
from src.utils.network import ZMQKeypointPublisher, ZMQKeypointSubscriber
from src.components.robot.xarm import Xarm
from src.data.xarm import XarmInfo
from src.components.transmitter.video_receiver import VideoReceiver
from src.components.operators.xarm import XarmOperator
import hydra
import time, json
import zmq


class XarmInfoNotifier(Component):
    def __init__(
        self,
        host,
        transformation_port,
        xarm_info_transmitter_port,
        action_port,
        xarm_ip,
    ):
        self.notify_component_start("xarm notifier")

        hydra.initialize(config_path="../../../configs", version_base="1.2")
        self.configs = hydra.compose("server")
        self.video_receiver = VideoReceiver(self.configs)
        self._transformed_arm_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host, port=transformation_port, topic="transformed_hand_frame"
        )
        self._transformed_hand_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host, port=transformation_port, topic="transformed_hand_coords"
        )
        self.xarm_publisher = ZMQKeypointPublisher(host, xarm_info_transmitter_port)
        self.action_subscriber = ZMQKeypointSubscriber(host, action_port, "action")

        self.robot = Xarm(xarm_ip)
        self.timer = FrequencyTimer(VR_FREQ)

        self.pre_action = None

    def _get_action(self):
        data = self.action_subscriber.recv_keypoints(flags=zmq.NOBLOCK)
        return data

    def stream(self):
        step = 0
        logger = LogTimer(1)
        start_time = None
        while True:
            if self.robot.if_shutdown():
                continue

            try:
                self.timer.start_loop()

                # receive anchors

                payload = {}
                gripper_state = self.robot.get_gripper_state()
                qpos = self.robot.get_joint_state()[0]
                qvel = self.robot.get_joint_velocity()
                transformed_hand_frames = (
                    self._transformed_arm_keypoint_subscriber.recv_keypoints()
                )
                transformed_hand_coords = (
                    self._transformed_hand_keypoint_subscriber.recv_keypoints()
                )
                end_position = self.robot.get_cartesian_position()

                # get action
                action = None
                if step == 0:
                    action = self.action_subscriber.recv_keypoints()
                    start_time = time.time()
                else:
                    action = self._get_action()

                if action is not None:
                    self.pre_action = action
                else:
                    action = self.pre_action

                print(action)

                # if int(time.time() - start_time) > 0:
                #     logger.trigger(f"{step / int(time.time() - start_time)}")

                cam_data = {}
                timestamp = time.time()
                for id in range(len(self.configs.camera_info)):
                    rgb_data, cam_name = self.video_receiver.get_cam_streamer(
                        id
                    ).get_image_tensor()

                    cam_data[cam_name] = rgb_data

                payload["step"] = step
                payload["timestamp"] = timestamp
                payload["gripper_state"] = gripper_state
                payload["qpos"] = qpos
                payload["qvel"] = qvel
                payload["transformed_hand_frames"] = transformed_hand_frames
                payload["transformed_hand_coords"] = transformed_hand_coords
                payload["end_position"] = end_position
                payload["cam_data"] = cam_data
                # payload["action"] = action

                self.xarm_publisher.pub_keypoints(
                    payload, topic_name=XARM_NOTIFIER_TOPIC
                )

                # End the timer
                self.timer.end_loop()
                step += 1

                # logger.trigger(f"{step/(time.time()-start_time)}")

            except Exception as e:
                print(e)
                break

        self._transformed_arm_keypoint_subscriber.stop()
        self._transformed_hand_keypoint_subscriber.stop()
        self.xarm_publisher.stop()
        self.robot.stop()
        print("Stopping the xarm notifier process.")
